"""
routes_page.py — Routes screen v2.5

[NEW so với v2.4]
  - [D] Stuck-handling: khi 1 waypoint thất bại giữa lộ trình, thay vì
    kết thúc mission ngay, hiện banner cảnh báo với 3 lựa chọn: Thử
    lại/Tiếp tục, Bỏ qua điểm này, Huỷ toàn bộ. Xử lý thật sự nằm ở
    MainWindow — RoutesPage chỉ hiện UI và emit signal.
  - [E] Auto / Manual mode: segmented toggle phía trên Mission Control.
    Auto = hành vi cũ (chạy liên tục). Manual = nút Start đổi nhãn theo
    từng bước ("Đến WPx" / "Thực hiện Task WPx" / "Xác nhận & đi tiếp"),
    mỗi lần bấm chỉ thực hiện đúng 1 hành động — tiện theo dõi/bảo trì.
    Pause/Resume bị khoá khi ở Manual (không cần dùng đến).
    Khoá 2 nút Auto/Manual trong lúc mission đang chạy.

Giữ nguyên v2.4:
  - Repeat / Lặp lại lộ trình: chạy 1 lần / N lần / vô hạn, mỗi chu kỳ
    ghi 1 record log riêng, chặn đổi route khi đang chạy/lặp.
  - JoystickWidget overlay lên map_widget (góc trái dưới).
  - Pose Estimate toggle button + MODE_POSE trên map_widget.
  - pose_estimate_signal / velocity_signal emit lên MainWindow.
"""

import os
import copy
import csv
import math
from datetime import datetime

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter,
    QLabel, QPushButton, QGroupBox, QFrame,
    QScrollArea, QMessageBox, QTabWidget,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QFileDialog, QAbstractItemView,
    QComboBox, QSpinBox,      # cho repeat combo/spin
    QButtonGroup,             # [NEW] cho Auto/Manual toggle
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
from PyQt6.QtGui import QPixmap, QColor

from agv_hmi.ui.map_widget import MapWidget, MODE_GOAL, MODE_POSE
from agv_hmi.ui.error_header import ErrorHeader
from agv_hmi.ui.joystick_widget import JoystickWidget
from agv_hmi.ui.i18n import tr
import agv_hmi.ui.route_manager as RM
import agv_hmi.ui.mission_logger as ML


WP_PENDING = "pending"
WP_MOVING  = "moving"
WP_TASK    = "task"
WP_DONE    = "done"
WP_STUCK   = "stuck"   # [NEW] waypoint đang gặp sự cố, chờ operator xử lý

WP_STATUS_ICON = {
    WP_PENDING: "□", WP_MOVING: "●",
    WP_TASK:    "◆", WP_DONE:   "✓",
    WP_STUCK:   "⚠",
}
WP_STATUS_COLOR = {
    WP_PENDING: "#8B949E", WP_MOVING: "#58A6FF",
    WP_TASK:    "#E3B341", WP_DONE:   "#3FB950",
    WP_STUCK:   "#F85149",
}
STATUS_COLOR = {
    "success": "#3FB950", "failed": "#F85149",
    "cancelled": "#E3B341", "running": "#58A6FF",
}
STATUS_LABEL = {
    "success": "Thành công", "failed": "Thất bại",
    "cancelled": "Đã huỷ",   "running": "Đang chạy",
}


def _btn(text, obj="", h=32, enabled=True):
    b = QPushButton(text)
    if obj: b.setObjectName(obj)
    b.setFixedHeight(h); b.setEnabled(enabled)
    return b


def _status_label(text):
    lbl = QLabel(text)
    lbl.setObjectName("StatusLabel")
    lbl.setWordWrap(True)
    return lbl


# ── Route card ────────────────────────────────────────────────────────

class _RouteCard(QFrame):
    run_clicked  = pyqtSignal(dict)
    edit_clicked = pyqtSignal(dict)
    del_clicked  = pyqtSignal(dict)

    def __init__(self, route: dict, show_edit: bool = False):
        super().__init__()
        self._route = copy.deepcopy(route)
        self.setObjectName("Card")
        self.setMaximumHeight(120)
        self.setStyleSheet(
            "QFrame#Card{background:#161B22;border:1px solid #30363D;"
            "border-radius:8px;}"
            "QFrame#Card:hover{border-color:#58A6FF;}")
        self._build(show_edit)

    def _build(self, show_edit):
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 8); root.setSpacing(6)

        top = QHBoxLayout(); top.setSpacing(8)
        thumb = QLabel()
        thumb.setFixedSize(72, 48)
        thumb.setAlignment(Qt.AlignmentFlag.AlignCenter)
        thumb.setStyleSheet(
            "background:#0D1117;border:1px solid #21262D;"
            "border-radius:5px;color:#484F58;font-size:20px;")
        tp = self._route.get("thumbnail_path", "")
        if tp and os.path.exists(tp):
            pix = QPixmap(tp)
            if not pix.isNull():
                thumb.setPixmap(pix.scaled(
                    70, 46,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation))
            else:
                thumb.setText("MAP")
        else:
            thumb.setText("MAP")
        top.addWidget(thumb)

        info = QVBoxLayout(); info.setSpacing(2)
        name_lbl = QLabel(self._route.get("name", "—"))
        name_lbl.setWordWrap(True)
        name_lbl.setStyleSheet(
            "font-size:12px;font-weight:700;"
            "color:#E6EDF3;background:transparent;")
        info.addWidget(name_lbl)

        map_name = os.path.basename(self._route.get("map_path", "")) or "—"
        wp_count = len(self._route.get("waypoints", []))
        try:
            stats = ML.get_stats(self._route.get("id", ""))
            total = stats.get("total", 0)
            cargo = stats.get("cargo_total", 0)
        except Exception:
            total = cargo = 0
        meta = QLabel(
            f"Map: {map_name} | {wp_count} WP | "
            f"Run {total} | Cargo {cargo}")
        meta.setStyleSheet(
            "font-size:10px;color:#8B949E;background:transparent;")
        info.addWidget(meta)

        top.addLayout(info); top.addStretch()
        del_btn = QPushButton("X")
        del_btn.setFixedSize(24, 24)
        del_btn.setObjectName("BtnIcon")
        del_btn.clicked.connect(
            lambda: self.del_clicked.emit(copy.deepcopy(self._route)))
        top.addWidget(del_btn, alignment=Qt.AlignmentFlag.AlignTop)
        root.addLayout(top)

        row = QHBoxLayout(); row.setSpacing(6)
        sel_btn = _btn("▶ Chọn", "BtnPrimary", h=26)
        sel_btn.clicked.connect(
            lambda: self.run_clicked.emit(copy.deepcopy(self._route)))
        row.addWidget(sel_btn)
        if show_edit:
            edit_btn = _btn(tr("route_edit"), h=26)
            edit_btn.clicked.connect(
                lambda: self.edit_clicked.emit(copy.deepcopy(self._route)))
            row.addWidget(edit_btn)
        row.addStretch()
        root.addLayout(row)


# ── History tab ───────────────────────────────────────────────────────

class _HistoryTab(QWidget):
    def __init__(self):
        super().__init__()
        self._filter_route_id = ""
        self._build()

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 12); lay.setSpacing(10)

        hdr = QHBoxLayout()
        self._filter_lbl = QLabel("Tất cả lộ trình")
        self._filter_lbl.setStyleSheet(
            "font-size:12px;font-weight:600;"
            "color:#E6EDF3;background:transparent;")
        hdr.addWidget(self._filter_lbl); hdr.addStretch()
        for text, slot in [
            ("Xem tất cả",    self._clear_filter),
            ("📤 Xuất CSV",   self._export_csv),
            ("🗑 Xoá lịch sử", self._clear_history),
        ]:
            b = _btn(text, "BtnDanger" if "Xoá" in text else "", h=28)
            b.clicked.connect(slot); hdr.addWidget(b)
        lay.addLayout(hdr)

        sf = QFrame()
        sf.setStyleSheet(
            "QFrame{background:#161B22;border:1px solid #21262D;"
            "border-radius:8px;}")
        sfl = QHBoxLayout(sf)
        sfl.setContentsMargins(12, 8, 12, 8); sfl.setSpacing(20)
        self._stat_labels: dict[str, QLabel] = {}
        for key, text in [
            ("total","Tổng"),("success","Thành công"),
            ("failed","Thất bại"),("cancelled","Đã huỷ"),("cargo","Hàng"),
        ]:
            col = QVBoxLayout()
            val = QLabel("0")
            val.setStyleSheet(
                "font-size:18px;font-weight:700;"
                "color:#58A6FF;background:transparent;")
            lbl = QLabel(text)
            lbl.setStyleSheet(
                "font-size:10px;color:#8B949E;background:transparent;")
            col.addWidget(val, alignment=Qt.AlignmentFlag.AlignHCenter)
            col.addWidget(lbl, alignment=Qt.AlignmentFlag.AlignHCenter)
            sfl.addLayout(col); self._stat_labels[key] = val
        lay.addWidget(sf)

        self._table = QTableWidget()
        self._table.setColumnCount(8)
        self._table.setHorizontalHeaderLabels([
            "Lộ trình","Bắt đầu","Kết thúc","Thời gian",
            "Trạng thái","WP","Hàng","Ghi chú",
        ])
        self._table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeMode.Stretch)
        self._table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.verticalHeader().setVisible(False)
        lay.addWidget(self._table)

        bot = QHBoxLayout(); bot.addStretch()
        self._del_row_btn = _btn("🗑 Xoá dòng", "BtnDanger", h=28, enabled=False)
        self._del_row_btn.clicked.connect(self._del_selected_row)
        bot.addWidget(self._del_row_btn); lay.addLayout(bot)
        self._table.itemSelectionChanged.connect(
            lambda: self._del_row_btn.setEnabled(
                len(self._table.selectedItems()) > 0))

    def filter_by_route(self, route_id, route_name):
        self._filter_route_id = route_id
        self._filter_lbl.setText(f"Lộ trình: {route_name}")
        self.refresh()

    def _clear_filter(self):
        self._filter_route_id = ""
        self._filter_lbl.setText("Tất cả lộ trình")
        self.refresh()

    def refresh(self):
        try: records = ML.load_all()
        except Exception: records = []
        if self._filter_route_id:
            records = [r for r in records
                       if r.get("route_id") == self._filter_route_id]
        try: stats = ML.get_stats(self._filter_route_id or None)
        except Exception:
            stats = {"total":0,"success":0,"failed":0,
                     "cancelled":0,"cargo_total":0}
        for k, sk in [("total","total"),("success","success"),
                      ("failed","failed"),("cancelled","cancelled"),
                      ("cargo","cargo_total")]:
            self._stat_labels[k].setText(str(stats.get(sk, 0)))

        self._table.setRowCount(len(records))
        for row, rec in enumerate(records):
            status  = rec.get("status", "")
            dur_sec = int(rec.get("duration_sec", 0) or 0)
            dur_txt = f"{dur_sec//60}m {dur_sec%60}s" if dur_sec > 0 else "—"
            wp_txt  = f"{rec.get('waypoints_done',0)}/{rec.get('waypoints_total',0)}"
            vals = [
                rec.get("route_name","—"),
                rec.get("started_at","—")[:19].replace("T"," "),
                (rec.get("finished_at","") or "—")[:19].replace("T"," "),
                dur_txt, STATUS_LABEL.get(status, status),
                wp_txt, str(rec.get("cargo_count",0)), rec.get("notes",""),
            ]
            for col, val in enumerate(vals):
                item = QTableWidgetItem(val)
                item.setData(Qt.ItemDataRole.UserRole, rec.get("id",""))
                if col == 4:
                    item.setForeground(
                        QColor(STATUS_COLOR.get(status,"#8B949E")))
                self._table.setItem(row, col, item)
        self._table.resizeColumnsToContents()
        self._table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeMode.Stretch)

    def _del_selected_row(self):
        row = self._table.currentRow()
        if row < 0: return
        item = self._table.item(row, 0)
        if item:
            try: ML.delete_record(item.data(Qt.ItemDataRole.UserRole))
            except Exception: pass
            self.refresh()

    def _clear_history(self):
        if QMessageBox.question(
            self, "Xoá lịch sử",
            "Xoá lịch sử chạy route?\nKhông thể hoàn tác.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        ) != QMessageBox.StandardButton.Yes: return
        try:
            if self._filter_route_id:
                ML.save_all([r for r in ML.load_all()
                             if r.get("route_id") != self._filter_route_id])
            else: ML.clear_all()
        except Exception: pass
        self.refresh()

    def _export_csv(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Xuất lịch sử",
            os.path.expanduser(
                f"~/agv_history_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"),
            "CSV (*.csv)")
        if not path: return
        try:
            records = ML.load_all()
            if self._filter_route_id:
                records = [r for r in records
                           if r.get("route_id") == self._filter_route_id]
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                w = csv.writer(f)
                w.writerow(["Lộ trình","Bắt đầu","Kết thúc","Thời gian(s)",
                             "Trạng thái","WP done","WP total","Hàng","Ghi chú"])
                for rec in records:
                    w.writerow([
                        rec.get("route_name",""), rec.get("started_at",""),
                        rec.get("finished_at",""), rec.get("duration_sec",0),
                        rec.get("status",""), rec.get("waypoints_done",0),
                        rec.get("waypoints_total",0), rec.get("cargo_count",0),
                        rec.get("notes",""),
                    ])
            QMessageBox.information(self, "Xuất CSV", f"Đã lưu:\n{path}")
        except Exception as e:
            QMessageBox.warning(self, "Lỗi", str(e))


# ── RoutesPage ────────────────────────────────────────────────────────

class RoutesPage(QWidget):
    # [1] Thêm 2 signal mới cho joystick và pose estimate
    run_route_signal      = pyqtSignal(dict)
    pause_signal          = pyqtSignal()
    resume_signal         = pyqtSignal()
    stop_signal           = pyqtSignal()
    confirm_signal        = pyqtSignal()
    edit_route_signal     = pyqtSignal(dict)
    velocity_signal       = pyqtSignal(float, float)        # [1] joystick → /cmd_vel
    pose_estimate_signal  = pyqtSignal(float, float, float) # [2] → /initialpose

    # [E] Auto/Manual mode + bước tiếp theo trong Manual
    mode_changed          = pyqtSignal(bool)   # True = manual
    manual_continue_signal = pyqtSignal()

    # [D] Stuck-handling: Thử lại / Bỏ qua / Huỷ
    retry_wp_signal    = pyqtSignal()
    skip_wp_signal     = pyqtSignal()
    cancel_stuck_signal = pyqtSignal()

    def __init__(self, role: str = "operator"):
        super().__init__()
        self._role = role
        self._current_route: dict | None = None
        self._wp_status_labels: list[QLabel] = []
        self._bumper_labels: dict[str, QLabel] = {}
        self._active_record: dict | None = None
        self._cargo_count = 0

        # [NEW] Repeat state
        self._repeat_active = False
        self._repeat_done   = 0
        self._repeat_pause_ms = 3000        # nghỉ giữa 2 chuyến
        self._repeat_timer = QTimer(self)
        self._repeat_timer.setSingleShot(True)
        self._repeat_timer.timeout.connect(self._repeat_next_cycle)

        # [E] Auto/Manual state
        self._manual_mode = False
        self._awaiting_manual_step = False

        self._build()
        self.refresh()

    # ── Build ─────────────────────────────────────────────────────────

    def _build(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0); root.setSpacing(0)
        root.addWidget(self._build_list_panel())

        right = QWidget()
        right_lay = QVBoxLayout(right)
        right_lay.setContentsMargins(0, 0, 0, 0); right_lay.setSpacing(0)

        self.error_header = ErrorHeader()
        right_lay.addWidget(self.error_header)

        self._tabs = QTabWidget()
        self._tabs.setStyleSheet(
            "QTabWidget::pane{border:none;background:#0D1117;}"
            "QTabBar::tab{background:#161B22;color:#8B949E;"
            "padding:6px 18px;border:none;"
            "border-bottom:2px solid transparent;}"
            "QTabBar::tab:selected{"
            "color:#58A6FF;border-bottom:2px solid #58A6FF;"
            "background:#0D1117;}"
            "QTabBar::tab:hover{color:#E6EDF3;background:#1C2D40;}")

        map_tab = QWidget()
        map_lay = QHBoxLayout(map_tab)
        map_lay.setContentsMargins(0, 0, 0, 0); map_lay.setSpacing(0)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._build_map_area())   # [1][2] map + joystick + pose
        splitter.addWidget(self._build_status_panel())
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        splitter.setSizes([850, 270])
        map_lay.addWidget(splitter)
        self._tabs.addTab(map_tab, "🗺  Bản đồ")

        self._history_tab = _HistoryTab()
        self._tabs.addTab(self._history_tab, "📋  Lịch sử")
        self._tabs.currentChanged.connect(self._on_tab_change)

        right_lay.addWidget(self._tabs)
        root.addWidget(right, 1)

    def _build_list_panel(self):
        panel = QWidget()
        panel.setFixedWidth(300)
        panel.setStyleSheet(
            "background:#010409;border-right:1px solid #21262D;")
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(10, 14, 10, 14); lay.setSpacing(8)

        hdr = QHBoxLayout()
        title = QLabel(tr("route_title"))
        title.setStyleSheet(
            "font-size:14px;font-weight:700;"
            "color:#E6EDF3;background:transparent;")
        hdr.addWidget(title); hdr.addStretch()
        rb = _btn("↺", h=26); rb.setFixedWidth(28)
        rb.clicked.connect(self.refresh); hdr.addWidget(rb)
        lay.addLayout(hdr)

        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self._scroll.setStyleSheet(
            "QScrollArea{border:none;background:transparent;}"
            "QScrollBar:vertical{width:5px;}"
            "QScrollBar::handle:vertical{"
            "background:#30363D;border-radius:2px;min-height:20px;}")
        self._cards_widget = QWidget()
        self._cards_widget.setStyleSheet("background:transparent;")
        self._cards_lay = QVBoxLayout(self._cards_widget)
        self._cards_lay.setContentsMargins(0, 2, 2, 2)
        self._cards_lay.setSpacing(6)
        self._cards_lay.addStretch()
        self._scroll.setWidget(self._cards_widget)
        lay.addWidget(self._scroll, 1)

        self._empty_lbl = QLabel(tr("route_no_routes"))
        self._empty_lbl.setObjectName("EmptyHint")
        self._empty_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._empty_lbl.setWordWrap(True)
        lay.addWidget(self._empty_lbl)
        return panel

    def _build_map_area(self):
        """
        [1] JoystickWidget overlay góc trái dưới map_widget.
        [2] map_widget connect pose_estimate_set → emit pose_estimate_signal.
        """
        container = QWidget()
        container.setMinimumWidth(400)
        container.setMinimumHeight(300)

        self.map_widget = MapWidget()
        self.map_widget.setMinimumSize(400, 300)

        # [2] Pose estimate: khi user kéo thả trên map → emit signal lên MainWindow
        self.map_widget.pose_estimate_set.connect(self.pose_estimate_signal)

        lay = QVBoxLayout(container)
        lay.setContentsMargins(0, 0, 0, 0); lay.setSpacing(0)
        lay.addWidget(self.map_widget)

        # [1] Joystick overlay
        self._joystick = JoystickWidget(container)
        self._joystick.velocity_signal.connect(self.velocity_signal)
        self._joystick.show()
        self._joystick.raise_()

        return container

    def _build_status_panel(self):
        sa = QScrollArea()
        sa.setWidgetResizable(True)
        sa.setFixedWidth(270)
        sa.setObjectName("Panel")

        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(10, 12, 10, 12); lay.setSpacing(10)

        # Velocity
        vel_box = QGroupBox("VẬN TỐC")
        vl = QVBoxLayout(vel_box)
        self._vel_lin = QLabel("Linear:  0.000 m/s")
        self._vel_lin.setObjectName("MonoVal")
        self._vel_ang = QLabel("Angular: 0.000 rad/s")
        self._vel_ang.setObjectName("MonoVal")
        vl.addWidget(self._vel_lin); vl.addWidget(self._vel_ang)
        lay.addWidget(vel_box)

        # [2] Pose Estimate button
        pose_box = QGroupBox(tr("nav_pose_estimate"))
        pl = QVBoxLayout(pose_box)
        hint = QLabel(tr("nav_pose_hint"))
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#8B949E;font-size:11px;background:transparent;")
        self._pose_btn = _btn(tr("nav_pose_enable"), h=30)
        self._pose_btn.setCheckable(True)
        self._pose_btn.toggled.connect(self._toggle_pose_mode)
        pl.addWidget(hint); pl.addWidget(self._pose_btn)
        lay.addWidget(pose_box)

        # Waypoints
        wp_box = QGroupBox("WAYPOINTS")
        self._wp_lay = QVBoxLayout(wp_box)
        self._empty_wp_lbl = _status_label("— Chưa chọn lộ trình —")
        self._wp_lay.addWidget(self._empty_wp_lbl)
        lay.addWidget(wp_box)

        # Task hiện tại
        task_box = QGroupBox("TASK HIỆN TẠI")
        tl = QVBoxLayout(task_box)
        self._task_lbl = _status_label("—")
        self._confirm_btn = _btn(tr("nav_confirm"), "BtnSuccess",
                                 h=30, enabled=False)
        self._confirm_btn.clicked.connect(self._on_confirm)
        tl.addWidget(self._task_lbl); tl.addWidget(self._confirm_btn)
        lay.addWidget(task_box)

        # Bumper
        bumper_box = QGroupBox(tr("bumper_title"))
        bl = QVBoxLayout(bumper_box); br = QHBoxLayout()
        for side in ["front", "rear", "left", "right"]:
            col = QVBoxLayout()
            nm = QLabel(tr(f"bumper_{side}"))
            nm.setAlignment(Qt.AlignmentFlag.AlignCenter)
            nm.setStyleSheet(
                "color:#8B949E;font-size:10px;background:transparent;")
            st = QLabel(tr("bumper_ok"))
            st.setAlignment(Qt.AlignmentFlag.AlignCenter)
            st.setStyleSheet(
                "color:#3FB950;font-size:11px;"
                "font-weight:600;background:transparent;")
            col.addWidget(nm); col.addWidget(st)
            br.addLayout(col); self._bumper_labels[side] = st
        bl.addLayout(br); lay.addWidget(bumper_box)

        # Live stats
        self._live_box = QGroupBox("CHUYẾN HIỆN TẠI")
        ll = QVBoxLayout(self._live_box)
        self._live_cargo_lbl = QLabel("📦 Hàng đã vận chuyển: 0")
        self._live_cargo_lbl.setStyleSheet(
            "font-size:12px;color:#3FB950;"
            "font-weight:600;background:transparent;")
        self._live_elapsed_lbl = QLabel("⏱ Thời gian: —")
        self._live_elapsed_lbl.setStyleSheet(
            "font-size:11px;color:#8B949E;background:transparent;")
        ll.addWidget(self._live_cargo_lbl); ll.addWidget(self._live_elapsed_lbl)
        lay.addWidget(self._live_box)
        self._live_box.setVisible(False)

        self._elapsed_timer = QTimer()
        self._elapsed_timer.setInterval(1000)
        self._elapsed_timer.timeout.connect(self._update_elapsed)

        # [E] Auto / Manual mode toggle
        mode_box = QGroupBox(tr("mode_title"))
        mode_lay = QHBoxLayout(mode_box); mode_lay.setSpacing(6)
        self._mode_group = QButtonGroup(self)
        self._mode_group.setExclusive(True)
        self._auto_btn = QPushButton(tr("mode_auto"))
        self._auto_btn.setCheckable(True)
        self._auto_btn.setChecked(True)
        self._auto_btn.setFixedHeight(32)
        self._auto_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._manual_btn = QPushButton(tr("mode_manual"))
        self._manual_btn.setCheckable(True)
        self._manual_btn.setFixedHeight(32)
        self._manual_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._mode_group.addButton(self._auto_btn)
        self._mode_group.addButton(self._manual_btn)
        self._auto_btn.toggled.connect(self._on_mode_toggled)
        mode_lay.addWidget(self._auto_btn)
        mode_lay.addWidget(self._manual_btn)
        lay.addWidget(mode_box)

        # [NEW] Repeat / Lặp lại lộ trình
        repeat_box = QGroupBox(tr("route_repeat_title"))
        rl = QVBoxLayout(repeat_box); rl.setSpacing(6)

        self._repeat_combo = QComboBox()
        self._repeat_combo.addItem(tr("route_repeat_once"),     "once")
        self._repeat_combo.addItem(tr("route_repeat_count"),    "count")
        self._repeat_combo.addItem(tr("route_repeat_infinite"), "infinite")
        self._repeat_combo.currentIndexChanged.connect(self._on_repeat_mode_changed)
        rl.addWidget(self._repeat_combo)

        count_row = QHBoxLayout()
        count_row.addWidget(QLabel(tr("route_repeat_times")))
        self._repeat_spin = QSpinBox()
        self._repeat_spin.setRange(2, 999)
        self._repeat_spin.setValue(3)
        self._repeat_spin.setFixedWidth(80)
        count_row.addStretch(); count_row.addWidget(self._repeat_spin)
        self._repeat_count_row = QWidget(); self._repeat_count_row.setLayout(count_row)
        self._repeat_count_row.setVisible(False)
        rl.addWidget(self._repeat_count_row)

        self._repeat_progress_lbl = QLabel("")
        self._repeat_progress_lbl.setStyleSheet(
            "color:#58A6FF;font-size:12px;font-weight:700;background:transparent;")
        self._repeat_progress_lbl.setVisible(False)
        rl.addWidget(self._repeat_progress_lbl)

        lay.addWidget(repeat_box)

        # [D] Stuck banner — ẩn mặc định, chỉ hiện khi mission gặp sự cố
        self._stuck_frame = QFrame()
        self._stuck_frame.setStyleSheet(
            "QFrame{background:#2E2000;border:1px solid #9E6A03;"
            "border-radius:8px;}")
        sfl = QVBoxLayout(self._stuck_frame)
        sfl.setContentsMargins(10, 8, 10, 8); sfl.setSpacing(6)
        self._stuck_lbl = QLabel("")
        self._stuck_lbl.setWordWrap(True)
        self._stuck_lbl.setStyleSheet(
            "color:#E3B341;font-size:12px;font-weight:700;background:transparent;")
        sfl.addWidget(self._stuck_lbl)
        stuck_btns = QHBoxLayout(); stuck_btns.setSpacing(6)
        self._stuck_retry_btn  = _btn(tr("stuck_retry"),  "BtnPrimary", h=30)
        self._stuck_skip_btn   = _btn(tr("stuck_skip"),   h=30)
        self._stuck_cancel_btn = _btn(tr("stuck_cancel"), "BtnDanger",  h=30)
        self._stuck_retry_btn.clicked.connect(self.retry_wp_signal)
        self._stuck_skip_btn.clicked.connect(self.skip_wp_signal)
        self._stuck_cancel_btn.clicked.connect(self.cancel_stuck_signal)
        stuck_btns.addWidget(self._stuck_retry_btn)
        stuck_btns.addWidget(self._stuck_skip_btn)
        stuck_btns.addWidget(self._stuck_cancel_btn)
        sfl.addLayout(stuck_btns)
        self._stuck_frame.setVisible(False)
        lay.addWidget(self._stuck_frame)

        # Mission control
        ctrl_box = QGroupBox("MISSION CONTROL")
        cl = QVBoxLayout(ctrl_box); cl.setSpacing(6)

        r1 = QHBoxLayout()
        self._start_btn = _btn(tr("nav_start"), "BtnSuccess", h=34, enabled=False)
        self._stop_btn  = _btn(tr("nav_stop"),  "BtnDanger",  h=34)
        self._start_btn.clicked.connect(self._on_start)
        self._stop_btn.clicked.connect(self._on_stop)
        r1.addWidget(self._start_btn); r1.addWidget(self._stop_btn)
        cl.addLayout(r1)

        r2 = QHBoxLayout()
        self._pause_btn  = _btn(tr("nav_pause"),  "BtnPrimary", h=28, enabled=False)
        self._resume_btn = _btn(tr("nav_resume"), "BtnPrimary", h=28, enabled=False)
        self._pause_btn.clicked.connect(self._on_pause)
        self._resume_btn.clicked.connect(self._on_resume)
        r2.addWidget(self._pause_btn); r2.addWidget(self._resume_btn)
        cl.addLayout(r2)

        self._status_lbl = _status_label(tr("nav_waiting"))
        cl.addWidget(self._status_lbl)
        lay.addWidget(ctrl_box)

        lay.addStretch()
        sa.setWidget(inner)
        return sa

    # ── Resize / Show — reposition joystick ──────────────────────────

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._reposition_joystick()

    def showEvent(self, ev):
        super().showEvent(ev)
        self._reposition_joystick()

    def _reposition_joystick(self):
        if not hasattr(self, "_joystick"): return
        mw = self.map_widget
        ph = mw.height()
        jh = self._joystick.height()
        self._joystick.move(16, max(16, ph - jh - 16))
        self._joystick.raise_()
        self._joystick.show()

    # ── [2] Pose Estimate toggle ──────────────────────────────────────

    def _toggle_pose_mode(self, checked: bool):
        if checked:
            self.map_widget.set_mode(MODE_POSE)
            self._pose_btn.setText(tr("nav_pose_active"))
        else:
            self.map_widget.set_mode(MODE_GOAL)
            self.map_widget.clear_pose_estimate()
            self._pose_btn.setText(tr("nav_pose_enable"))

    # ── [E] Auto / Manual mode toggle ───────────────────────────────────

    def _on_mode_toggled(self, auto_checked: bool):
        """auto_checked=True nghĩa là nút AUTO đang được chọn."""
        self._manual_mode = not auto_checked
        self.mode_changed.emit(self._manual_mode)

    def manual_prompt(self, text: str):
        """
        [E] Gọi bởi MainWindow khi Manual mode đang chờ operator bấm
        bước kế tiếp. Đổi nhãn nút Start và bật lại (nút này bị disable
        khi mission đang chạy bình thường ở Auto).
        """
        self._awaiting_manual_step = True
        self._start_btn.setText(text)
        self._start_btn.setEnabled(True)

    def manual_busy(self):
        """Operator vừa bấm bước kế — khoá nút lại để tránh bấm lặp
        trong lúc robot đang di chuyển / task đang chạy."""
        self._awaiting_manual_step = False
        self._start_btn.setEnabled(False)

    # ── [D] Stuck-handling ───────────────────────────────────────────────

    def on_mission_stuck(self, wp_label: str):
        """Gọi bởi MainWindow khi 1 waypoint thất bại — hiện banner với
        3 lựa chọn xử lý, không kết thúc mission."""
        self._stuck_lbl.setText(tr("stuck_title", wp_label))
        self._stuck_frame.setVisible(True)
        self._start_btn.setEnabled(False)
        self._pause_btn.setEnabled(False)
        self._resume_btn.setEnabled(False)

    def clear_stuck(self):
        self._stuck_frame.setVisible(False)

    # ── [NEW] Repeat logic ────────────────────────────────────────────

    def _on_repeat_mode_changed(self, _idx):
        mode = self._repeat_combo.currentData()
        self._repeat_count_row.setVisible(mode == "count")

    def _should_repeat_more(self) -> bool:
        mode = self._repeat_combo.currentData()
        if mode == "infinite":
            return True
        if mode == "count":
            return self._repeat_done < self._repeat_spin.value()
        return False

    def _update_repeat_progress_label(self):
        mode = self._repeat_combo.currentData()
        if mode == "once":
            self._repeat_progress_lbl.setVisible(False)
            return
        self._repeat_progress_lbl.setVisible(True)
        n = self._repeat_done + 1
        if mode == "infinite":
            self._repeat_progress_lbl.setText(tr("route_repeat_progress_inf", n))
        else:
            self._repeat_progress_lbl.setText(
                tr("route_repeat_progress", n, self._repeat_spin.value()))

    def _repeat_next_cycle(self):
        if not self._repeat_active or not self._current_route:
            return
        self._on_start()

    # ── Tab ───────────────────────────────────────────────────────────

    def _on_tab_change(self, idx):
        if idx == 1:
            self._history_tab.refresh()

    # ── Public API ────────────────────────────────────────────────────

    def refresh(self):
        while self._cards_lay.count() > 1:
            item = self._cards_lay.takeAt(0)
            w = item.widget()
            if w: w.deleteLater()
        try: routes = RM.load_all()
        except Exception as e:
            print(f"[RoutesPage] load routes failed: {e}"); routes = []
        self._empty_lbl.setVisible(not routes)
        for route in routes:
            card = _RouteCard(route, show_edit=(self._role == "engineer"))
            card.run_clicked.connect(self._on_run)
            card.edit_clicked.connect(self.edit_route_signal)
            card.del_clicked.connect(self._on_delete)
            self._cards_lay.insertWidget(self._cards_lay.count() - 1, card)

    def update_pose(self, x, y, yaw):
        self.map_widget.update_pose(x, y, yaw)

    def update_scan(self, world_pts):
        self.map_widget.update_scan(world_pts)

    def update_velocity(self, linear, angular):
        self._vel_lin.setText(f"Linear:  {linear:.3f} m/s")
        self._vel_ang.setText(f"Angular: {angular:.3f} rad/s")

    def update_error(self, msg):
        self.error_header.update_from_ros(msg)

    def update_bumper(self, side, triggered):
        lbl = self._bumper_labels.get(side.lower().strip())
        if not lbl: return
        if triggered:
            lbl.setText(tr("bumper_triggered"))
            lbl.setStyleSheet(
                "color:#F85149;font-size:11px;"
                "font-weight:600;background:transparent;")
        else:
            lbl.setText(tr("bumper_ok"))
            lbl.setStyleSheet(
                "color:#3FB950;font-size:11px;"
                "font-weight:600;background:transparent;")

    def set_status(self, text):
        self._status_lbl.setText(text)

    def set_task_status(self, text):
        self._task_lbl.setText(text)

    def set_nav2_error(self, message, is_error=True):
        if is_error: self.error_header.set_nav2_error(message)
        else: self.error_header.set_nav2_warning(message)

    def clear_nav2_error(self):
        self.error_header.clear_nav2()

    def request_confirm(self, message: str = ""):
        self.set_task_status(message or tr("task_confirm"))
        self.set_status(message or tr("nav_confirm"))
        self._confirm_btn.setEnabled(True)

    def clear_confirm(self):
        if hasattr(self, "_confirm_btn"):
            self._confirm_btn.setEnabled(False)

    def _on_confirm(self):
        self.clear_confirm(); self.confirm_signal.emit()

    def _current_route_is_ready(self) -> bool:
        if not self._current_route: return False
        mp = self._normalize_map_path(
            self._current_route.get("map_path", ""))
        return bool(mp and os.path.exists(mp)
                    and self._current_route.get("waypoints", []))

    def set_mission_done(self):
        self._start_btn.setEnabled(self._current_route_is_ready())
        self._start_btn.setText(tr("nav_start"))
        self._pause_btn.setEnabled(False)
        self._resume_btn.setEnabled(False)
        self.clear_confirm()
        # [NEW] Reset trạng thái lặp — chuỗi lặp coi như kết thúc.
        self._repeat_timer.stop()
        self._repeat_active = False
        self._repeat_combo.setEnabled(True)
        self._repeat_spin.setEnabled(True)
        self._repeat_progress_lbl.setVisible(False)
        # [D][E] Reset Manual step + banner Stuck + mở khoá Auto/Manual
        self._awaiting_manual_step = False
        self.clear_stuck()
        self._auto_btn.setEnabled(True)
        self._manual_btn.setEnabled(True)

    def set_wp_status(self, idx, status):
        if not (0 <= idx < len(self._wp_status_labels)): return
        icon  = WP_STATUS_ICON.get(status, "□")
        color = WP_STATUS_COLOR.get(status, "#8B949E")
        lbl   = self._wp_status_labels[idx]
        rest  = lbl.text().split(" ", 1)[1] if " " in lbl.text() else lbl.text()
        lbl.setText(f"{icon} {rest}")
        lbl.setStyleSheet(
            f"color:{color};font-size:12px;background:transparent;")
        if status == WP_DONE and self._active_record is not None:
            self._active_record["waypoints_done"] = idx + 1

    def notify_cargo_delivered(self, count=1):
        self._cargo_count += count
        self._live_cargo_lbl.setText(
            f"📦 Hàng đã vận chuyển: {self._cargo_count}")

    def on_mission_success(self):
        """
        [NEW] Ghi log chu kỳ vừa xong. Nếu đang trong chuỗi lặp và còn
        lượt (đếm) hoặc chưa Stop (vô hạn) → nghỉ _repeat_pause_ms rồi
        tự chạy lại. Ngược lại → kết thúc chuỗi (set_mission_done).
        """
        self._stop_logging("success")
        self._repeat_done += 1
        if self._repeat_active and self._should_repeat_more():
            self._update_repeat_progress_label()
            wait_s = self._repeat_pause_ms // 1000
            self.set_status(tr("route_repeat_waiting", wait_s))
            self._repeat_timer.start(self._repeat_pause_ms)
        else:
            self.set_mission_done()

    def on_mission_failed(self):
        """[NEW] Thất bại giữa chừng → DỪNG LUÔN chuỗi lặp, không lặp lại lỗi."""
        self._stop_logging("failed")
        self.set_mission_done()

    # ── Logging ───────────────────────────────────────────────────────

    def _start_logging(self):
        if not self._current_route: return
        self._cargo_count = 0
        wps = self._current_route.get("waypoints", [])
        try:
            self._active_record = ML.start_record(
                route_id=self._current_route.get("id", ""),
                route_name=self._current_route.get("name", "—"),
                waypoints_total=len(wps),
            )
        except Exception as e:
            print(f"[RoutesPage] start logging failed: {e}")
            self._active_record = None; return
        self._live_box.setVisible(True)
        self._live_cargo_lbl.setText("📦 Hàng đã vận chuyển: 0")
        self._elapsed_timer.start()

    def _stop_logging(self, status):
        self._elapsed_timer.stop()
        if self._active_record is None: return

        # [NEW] Ghi chú rõ đây là chu kỳ lặp thứ mấy, để phân biệt trong
        # tab Lịch sử khi nhiều chu kỳ cùng route_id chồng lên nhau.
        notes = ""
        if self._repeat_active and self._repeat_combo.currentData() != "once":
            if self._repeat_combo.currentData() == "infinite":
                notes = f"Chu kỳ lặp #{self._repeat_done + 1} (vô hạn)"
            else:
                notes = f"Chu kỳ lặp {self._repeat_done + 1}/{self._repeat_spin.value()}"

        try:
            ML.finish_record(
                self._active_record, status=status,
                waypoints_done=self._active_record.get("waypoints_done", 0),
                cargo_count=self._cargo_count,
                notes=notes,
            )
        except Exception as e:
            print(f"[RoutesPage] stop logging failed: {e}")
        self._active_record = None
        self._live_box.setVisible(False)
        self.refresh()

    def _update_elapsed(self):
        if not self._active_record: return
        try:
            started = datetime.fromisoformat(self._active_record["started_at"])
            elapsed = int((datetime.now() - started).total_seconds())
            self._live_elapsed_lbl.setText(
                f"⏱ Thời gian: {elapsed//60}m {elapsed%60}s")
        except Exception: pass

    # ── Route selection ───────────────────────────────────────────────

    def _normalize_map_path(self, path: str) -> str:
        path = os.path.expanduser(str(path or "").strip())
        if not path: return ""
        if os.path.exists(path): return path
        root, ext = os.path.splitext(path)
        if ext.lower() == ".pgm" and os.path.exists(root + ".yaml"):
            return root + ".yaml"
        if not ext and os.path.exists(path + ".yaml"):
            return path + ".yaml"
        return path

    def _on_run(self, route):
        # [NEW] Chặn đổi route khi đang chạy/lặp — tránh log lẫn lộn
        # giữa route cũ đang chạy nền và route mới vừa chọn.
        if self._repeat_active or self._pause_btn.isEnabled():
            QMessageBox.warning(self, tr("route_repeat_title"),
                                 tr("route_busy_warning"))
            return

        self._current_route = copy.deepcopy(route)
        mp = self._normalize_map_path(self._current_route.get("map_path",""))
        self._current_route["map_path"] = mp
        map_ok = False

        if mp and os.path.exists(mp):
            try:
                print(f"[RoutesPage] Loading map: {mp}")
                self.map_widget.load_map_file(mp)
                self.map_widget.update()
                map_ok = True
                print("[RoutesPage] Map loaded OK")
            except Exception as e:
                print(f"[RoutesPage] Load map failed: {e}")
                self._status_lbl.setText(f"Không load được map: {e}")
        else:
            print(f"[RoutesPage] Invalid map_path: {mp!r}")
            self._status_lbl.setText(f"Route thiếu map .yaml: {mp}")

        waypoints = self._current_route.get("waypoints", [])
        pts = [(float(wp.get("x",0.0)), float(wp.get("y",0.0)),
                str(wp.get("label","?"))) for wp in waypoints]
        try:
            self.map_widget.set_waypoints(pts)
            self.map_widget.update()
        except Exception as e:
            print(f"[RoutesPage] set_waypoints failed: {e}")

        self._build_wp_status_list(waypoints)
        ready = bool(waypoints) and map_ok
        self._start_btn.setEnabled(ready)
        self._pause_btn.setEnabled(False)
        self._resume_btn.setEnabled(False)
        self.clear_confirm()

        # [2] Reset pose mode khi chọn route mới
        self._pose_btn.setChecked(False)

        name = self._current_route.get("name", "?")
        self._status_lbl.setText(
            f"Đã chọn: {name} — Nhấn Start để chạy"
            if ready else "Route thiếu map hoặc waypoint.")
        self._history_tab.filter_by_route(
            self._current_route.get("id",""), name)
        self._tabs.setCurrentIndex(0)

    def _build_wp_status_list(self, waypoints):
        while self._wp_lay.count():
            item = self._wp_lay.takeAt(0)
            w = item.widget()
            if w: w.deleteLater()
        self._wp_status_labels.clear()
        if not waypoints:
            self._wp_lay.addWidget(
                _status_label("— Route không có waypoint —")); return
        for wp in waypoints:
            x = float(wp.get("x", 0.0)); y = float(wp.get("y", 0.0))
            n = len(wp.get("tasks", []))
            text = f"□ {wp.get('label','?')} ({x:.2f}, {y:.2f})"
            if n: text += f" | {n}T"
            lbl = QLabel(text)
            lbl.setStyleSheet(
                "color:#8B949E;font-size:11px;background:transparent;")
            self._wp_lay.addWidget(lbl)
            self._wp_status_labels.append(lbl)

    # ── Mission buttons ───────────────────────────────────────────────

    def _on_start(self):
        # [E] Đang chờ operator bấm bước kế trong Manual mode — KHÔNG
        # phải bắt đầu mission mới, mà là thực hiện đúng bước đang chờ.
        if self._awaiting_manual_step:
            self.manual_busy()
            self.manual_continue_signal.emit()
            return

        if not self._current_route:
            self._status_lbl.setText("Chưa chọn lộ trình."); return
        mp = self._normalize_map_path(
            self._current_route.get("map_path",""))
        if not mp or not os.path.exists(mp):
            self._status_lbl.setText(
                "Route thiếu map .yaml hợp lệ, không thể chạy.")
            self._start_btn.setEnabled(False); return
        if not self._current_route.get("waypoints", []):
            self._status_lbl.setText("Route không có waypoint."); return
        self._current_route["map_path"] = mp

        # [NEW] Bắt đầu (hoặc tiếp tục) chuỗi lặp. Nếu đây là lần Start
        # đầu tiên của chuỗi (không phải do _repeat_next_cycle gọi lại)
        # thì reset bộ đếm và khoá combo/spin lại để tránh đổi giữa chừng.
        if not self._repeat_active:
            self._repeat_done = 0
            self._repeat_active = True
            self._repeat_combo.setEnabled(False)
            self._repeat_spin.setEnabled(False)
        self._update_repeat_progress_label()

        self._start_btn.setText(tr("nav_start"))
        self._start_btn.setEnabled(False)
        # [E] Manual mode không dùng Pause (mỗi bước đã tự "pause" sẵn).
        self._pause_btn.setEnabled(not self._manual_mode)
        self._resume_btn.setEnabled(False)
        # [E] Khoá Auto/Manual trong lúc mission đang chạy
        self._auto_btn.setEnabled(False)
        self._manual_btn.setEnabled(False)
        # MainWindow gọi _start_logging sau khi Nav2 ready
        self.run_route_signal.emit(copy.deepcopy(self._current_route))

    def _on_stop(self):
        # [NEW] Dừng hẳn chuỗi lặp (không chỉ dừng chu kỳ hiện tại).
        self._repeat_timer.stop()
        self._repeat_active = False
        # [D][E] Dọn banner Stuck + trạng thái chờ bước Manual (phòng khi
        # operator bấm Stop ngay lúc đang stuck/đang chờ bước kế).
        self._awaiting_manual_step = False
        self.clear_stuck()
        if self._active_record is not None:
            self._stop_logging("cancelled")
        self.stop_signal.emit()

    def _on_pause(self):
        self._pause_btn.setEnabled(False)
        self._resume_btn.setEnabled(True)
        self.pause_signal.emit()

    def _on_resume(self):
        self._pause_btn.setEnabled(True)
        self._resume_btn.setEnabled(False)
        self.resume_signal.emit()

    def _on_delete(self, route):
        name = route.get("name","?")
        if QMessageBox.question(
            self, tr("delete_route"),
            tr("route_del_confirm", name),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        ) != QMessageBox.StandardButton.Yes: return
        try: RM.delete_route(route["id"])
        except Exception as e:
            print(f"[RoutesPage] delete route failed: {e}")
        self.refresh()
        if (self._current_route
                and self._current_route.get("id") == route.get("id")):
            self._current_route = None
            try: self.map_widget.clear_waypoints()
            except Exception: pass
            self._build_wp_status_list([])
            self._start_btn.setEnabled(False)

    # ── i18n ─────────────────────────────────────────────────────────

    def retranslate(self):
        self.error_header.retranslate()
        # [E] Không ghi đè nhãn động ("Đến WP2"...) nếu đang chờ bước Manual
        if not self._awaiting_manual_step:
            self._start_btn.setText(tr("nav_start"))
        self._stop_btn.setText(tr("nav_stop"))
        self._pause_btn.setText(tr("nav_pause"))
        self._resume_btn.setText(tr("nav_resume"))
        self._confirm_btn.setText(tr("nav_confirm"))
        self._pose_btn.setText(
            tr("nav_pose_active") if self._pose_btn.isChecked()
            else tr("nav_pose_enable"))
        self._empty_lbl.setText(tr("route_no_routes"))
        # [E] Retranslate Auto/Manual toggle + banner Stuck
        self._auto_btn.setText(tr("mode_auto"))
        self._manual_btn.setText(tr("mode_manual"))
        self._stuck_retry_btn.setText(tr("stuck_retry"))
        self._stuck_skip_btn.setText(tr("stuck_skip"))
        self._stuck_cancel_btn.setText(tr("stuck_cancel"))
        # [NEW] Retranslate repeat controls
        cur_idx = self._repeat_combo.currentIndex()
        self._repeat_combo.blockSignals(True)
        self._repeat_combo.clear()
        self._repeat_combo.addItem(tr("route_repeat_once"),     "once")
        self._repeat_combo.addItem(tr("route_repeat_count"),    "count")
        self._repeat_combo.addItem(tr("route_repeat_infinite"), "infinite")
        self._repeat_combo.setCurrentIndex(max(0, cur_idx))
        self._repeat_combo.blockSignals(False)
        self._update_repeat_progress_label()
        self.refresh()


__all__ = [
    "RoutesPage",
    "WP_PENDING", "WP_MOVING", "WP_TASK", "WP_DONE", "WP_STUCK",
]
