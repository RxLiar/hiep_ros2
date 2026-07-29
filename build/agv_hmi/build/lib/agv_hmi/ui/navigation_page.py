"""
navigation_page.py — Màn hình Navigation

Engineer:  click map thêm waypoint, đặt multi-task, lưu lộ trình
Operator:  load lộ trình sẵn (read-only), chạy mission

Layout:
  [ErrorHeader]
  [MapWidget + Joystick overlay]  |  [Panel phải]

Panel phải:
  - Pose robot
  - VelocityInput (ô số)
  - Map load
  - Pose Estimate AMCL
  - Waypoints (Engineer: edit, Operator: read-only)
    → Chọn waypoint → TaskEditor hiện ra
  - Lưu lộ trình (Engineer)
  - Mission control: Start / Pause / Run / Stop
"""
import os
import copy
import math
import json

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter,
    QGroupBox, QLabel, QPushButton, QScrollArea,
    QListWidget, QListWidgetItem, QLineEdit,
    QFileDialog, QMessageBox, QFrame
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer

from agv_hmi.ui.map_widget import MapWidget, MODE_GOAL, MODE_POSE
from agv_hmi.ui.error_header import ErrorHeader
from agv_hmi.ui.joystick_widget import JoystickWidget
from agv_hmi.ui.velocity_input import VelocityInputPanel
from agv_hmi.ui.task_editor import TaskEditor
from agv_hmi.ui.i18n import tr
import agv_hmi.ui.route_manager as RM


def _mono(t):
    w = QLabel(t); w.setObjectName("MonoVal"); return w

def _lbl(t, obj="StatusLabel"):
    w = QLabel(t); w.setObjectName(obj); return w

def _btn(t, obj="", h=32, enabled=True):
    b = QPushButton(t)
    if obj: b.setObjectName(obj)
    b.setFixedHeight(h); b.setEnabled(enabled)
    return b


class NavigationPage(QWidget):
    # ── Signals ──────────────────────────────────────────────────────
    velocity_signal      = pyqtSignal(float, float)          # joystick → /cmd_vel
    pose_estimate_signal = pyqtSignal(float, float, float)   # → /initialpose
    run_mission_signal   = pyqtSignal(list)                  # waypoints → MainWindow
    stop_signal          = pyqtSignal()
    pause_signal         = pyqtSignal()   # FIX: thêm signal pause
    resume_signal        = pyqtSignal()   # FIX: thêm signal resume
    save_route_signal    = pyqtSignal(str, str, list, str)   # name, map_path, wps, route_id
    nav_launch_toggle_signal = pyqtSignal(bool)               # True=start, False=stop

    def __init__(self, role: str = "operator"):
        super().__init__()
        self._role = role
        self._waypoints: list[dict] = []   # [{label,x,y,tasks:[...]}, ...]
        self._selected_wp_idx  = -1
        self._selected_map_path = ""
        self._editing_route_id = ""

        self._build()

    # ── Build ────────────────────────────────────────────────────────

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Error header
        self.error_header = ErrorHeader()
        root.addWidget(self.error_header)

        # Body
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._build_map_area())
        splitter.addWidget(self._build_panel())
        splitter.setStretchFactor(0, 1)
        splitter.setSizes([980, 320])
        root.addWidget(splitter)

    def _build_map_area(self):
        container = QWidget()
        container.setMinimumWidth(400)

        self.map_widget = MapWidget()
        self.map_widget.pose_estimate_set.connect(self.pose_estimate_signal)

        if self._role == "engineer":
            self.map_widget.goal_selected.connect(self._on_map_click)

        lay = QVBoxLayout(container)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.map_widget)

        # Joystick overlay trên MapWidget
        self._joystick = JoystickWidget(self.map_widget)
        self._joystick.velocity_signal.connect(self.velocity_signal)
        self._joystick.show()
        self._joystick.raise_()

        return container

    def _build_panel(self):
        sa = QScrollArea()
        sa.setWidgetResizable(True)
        sa.setFixedWidth(330)
        sa.setObjectName("Panel")

        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(12, 14, 12, 14)
        lay.setSpacing(10)

        # ── NAV launch control ───────────────────────────────────
        nav_box = QGroupBox("NAV2")
        nav_lay = QVBoxLayout(nav_box)
        self._nav_toggle_btn = _btn("▶ Bật NAV", "BtnSuccess", h=34)
        self._nav_toggle_btn.setCheckable(True)
        self._nav_toggle_btn.toggled.connect(self._on_nav_toggle)
        self._nav_launch_lbl = _lbl("navigation.launch.py chưa chạy")
        self._nav_launch_lbl.setWordWrap(True)
        nav_lay.addWidget(self._nav_toggle_btn)
        nav_lay.addWidget(self._nav_launch_lbl)
        lay.addWidget(nav_box)

        # ── Pose ──────────────────────────────────────────────────
        pose_box = QGroupBox(tr("pose_robot"))
        pl = QVBoxLayout(pose_box)
        self._x_lbl   = _mono("X:    0.000 m")
        self._y_lbl   = _mono("Y:    0.000 m")
        self._yaw_lbl = _mono("Yaw:  0.0 °")
        for l in (self._x_lbl, self._y_lbl, self._yaw_lbl): pl.addWidget(l)
        lay.addWidget(pose_box)

        # ── Velocity input ─────────────────────────────────────────
        self._vel_panel = VelocityInputPanel()
        self._vel_panel.max_velocity_changed.connect(
            lambda l, a: self._joystick.set_max_velocity(l, a))
        lay.addWidget(self._vel_panel)

        # ── Map load ───────────────────────────────────────────────
        map_box = QGroupBox("MAP")
        ml = QVBoxLayout(map_box)
        self._map_lbl = _lbl(tr("route_map_none"))
        lb = _btn(tr("route_select_map"), "BtnPrimary", h=30)
        lb.clicked.connect(self._load_map)
        ml.addWidget(self._map_lbl); ml.addWidget(lb)
        lay.addWidget(map_box)

        # ── Pose Estimate ──────────────────────────────────────────
        pe_box = QGroupBox(tr("nav_pose_estimate"))
        pel = QVBoxLayout(pe_box)
        hint = _lbl(tr("nav_pose_hint")); hint.setWordWrap(True)
        self._pose_btn = _btn(tr("nav_pose_enable"), h=30)
        self._pose_btn.setCheckable(True)
        self._pose_btn.toggled.connect(self._toggle_pose_mode)
        pel.addWidget(hint); pel.addWidget(self._pose_btn)
        lay.addWidget(pe_box)

        # ── Waypoints ──────────────────────────────────────────────
        wp_box = QGroupBox(tr("route_waypoints"))
        wl = QVBoxLayout(wp_box); wl.setSpacing(6)

        if self._role == "engineer":
            hint2 = _lbl(tr("route_wp_hint")); hint2.setWordWrap(True)
            wl.addWidget(hint2)

        self._wp_list = QListWidget()
        self._wp_list.setFixedHeight(120)
        self._wp_list.currentRowChanged.connect(self._on_wp_select)
        wl.addWidget(self._wp_list)

        if self._role == "engineer":
            wp_btns = QHBoxLayout()
            self._add_end_btn = _btn(tr("route_wp_add_end"), h=28)
            self._add_end_btn.clicked.connect(self._add_end)
            self._del_wp_btn  = _btn("🗑", "BtnDanger", h=28, enabled=False)
            self._del_wp_btn.clicked.connect(self._del_wp)
            self._clr_btn     = _btn(tr("route_wp_clear"), h=28)
            self._clr_btn.clicked.connect(self._clear_wps)
            wp_btns.addWidget(self._add_end_btn)
            wp_btns.addWidget(self._del_wp_btn)
            wp_btns.addWidget(self._clr_btn)
            wl.addLayout(wp_btns)

        lay.addWidget(wp_box)

        # ── Task editor (Engineer only, hiện khi chọn waypoint) ───
        if self._role == "engineer":
            self._task_box = QGroupBox(tr("task_title"))
            tbl = QVBoxLayout(self._task_box)
            self._wp_info_lbl = _lbl("— Chọn waypoint —")
            tbl.addWidget(self._wp_info_lbl)
            self._task_editor = TaskEditor()
            self._task_editor.tasks_changed.connect(self._on_tasks_changed)
            tbl.addWidget(self._task_editor)
            self._task_box.setVisible(False)
            lay.addWidget(self._task_box)

            # Lưu lộ trình
            save_box = QGroupBox("LƯU LỘ TRÌNH")
            sl = QVBoxLayout(save_box); sl.setSpacing(6)
            sl.addWidget(QLabel(tr("route_name")))
            self._route_name = QLineEdit()
            self._route_name.setPlaceholderText(tr("route_name_ph"))
            self._save_route_btn = _btn(tr("route_save"), "BtnSuccess", h=32)
            self._save_route_btn.clicked.connect(self._save_route)
            sl.addWidget(self._route_name)
            sl.addWidget(self._save_route_btn)
            lay.addWidget(save_box)

        # ── Mission control ────────────────────────────────────────
        ctrl_box = QGroupBox(tr("nav_title"))
        cl = QVBoxLayout(ctrl_box); cl.setSpacing(6)

        btn_row1 = QHBoxLayout()
        self._start_btn = _btn(tr("nav_start"), "BtnSuccess", h=36, enabled=False)
        self._stop_btn  = _btn(tr("nav_stop"),  "BtnDanger",  h=36)
        self._start_btn.clicked.connect(self._on_start)
        self._stop_btn.clicked.connect(self.stop_signal)
        btn_row1.addWidget(self._start_btn)
        btn_row1.addWidget(self._stop_btn)

        btn_row2 = QHBoxLayout()
        self._pause_btn  = _btn(tr("nav_pause"),  "BtnPrimary", h=32, enabled=False)
        self._resume_btn = _btn(tr("nav_resume"), "BtnPrimary", h=32, enabled=False)
        self._pause_btn.clicked.connect(self._on_pause)
        self._resume_btn.clicked.connect(self._on_resume)
        btn_row2.addWidget(self._pause_btn)
        btn_row2.addWidget(self._resume_btn)

        self._status_lbl = _lbl(tr("nav_waiting"))
        self._status_lbl.setWordWrap(True)

        cl.addLayout(btn_row1)
        cl.addLayout(btn_row2)
        cl.addWidget(self._status_lbl)
        lay.addWidget(ctrl_box)

        lay.addStretch()
        sa.setWidget(inner)
        return sa

    def _on_nav_toggle(self, checked: bool):
        self.nav_launch_toggle_signal.emit(checked)

    def set_nav_launch_running(self, running: bool, text: str = ""):
        self._nav_toggle_btn.blockSignals(True)
        self._nav_toggle_btn.setChecked(running)
        self._nav_toggle_btn.setText("⬛ Tắt NAV" if running else "▶ Bật NAV")
        self._nav_toggle_btn.setObjectName("BtnDanger" if running else "BtnSuccess")
        self._nav_toggle_btn.style().unpolish(self._nav_toggle_btn)
        self._nav_toggle_btn.style().polish(self._nav_toggle_btn)
        self._nav_toggle_btn.blockSignals(False)
        self._nav_launch_lbl.setText(
            text or ("navigation.launch.py đang chạy" if running
                     else "navigation.launch.py chưa chạy"))

    # ── Joystick overlay ─────────────────────────────────────────────

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._reposition_joystick()

    def showEvent(self, ev):
        super().showEvent(ev)
        self._reposition_joystick()

    def _reposition_joystick(self):
        if hasattr(self, '_joystick') and self._joystick.parent():
            ph = self.map_widget.height()
            self._joystick.move(16, max(16, ph - self._joystick.height() - 16))
            self._joystick.raise_()
            self._joystick.show()

    # ── Public API ───────────────────────────────────────────────────

    def update_pose(self, x: float, y: float, yaw: float):
        self._x_lbl.setText(f"X:    {x:.3f} m")
        self._y_lbl.setText(f"Y:    {y:.3f} m")
        self._yaw_lbl.setText(f"Yaw:  {math.degrees(yaw):.1f} °")

    def update_pose_on_map(self, x: float, y: float, yaw: float):
        self.map_widget.update_pose(x, y, yaw)

    def update_map(self, msg):
        if not self._selected_map_path:
            self.map_widget.update_map(msg)

    def update_scan(self, world_pts):
        self.map_widget.update_scan(world_pts)

    def set_status(self, text: str):
        self._status_lbl.setText(text)

    def set_start_enabled(self, v: bool):
        self._start_btn.setEnabled(v)

    def set_mission_running(self, running: bool):
        self._start_btn.setEnabled(not running and bool(self._waypoints))
        self._pause_btn.setEnabled(running)
        self._resume_btn.setEnabled(False)

    def set_mission_paused(self, paused: bool):
        self._pause_btn.setEnabled(not paused)
        self._resume_btn.setEnabled(paused)

    def _normalize_map_path(self, path: str) -> str:
        path = os.path.expanduser(str(path or "").strip())
        if not path:
            return ""
        if os.path.exists(path):
            return path
        root, ext = os.path.splitext(path)
        if ext.lower() == ".pgm" and os.path.exists(root + ".yaml"):
            return root + ".yaml"
        if not ext and os.path.exists(path + ".yaml"):
            return path + ".yaml"
        return path

    def _has_valid_map(self) -> bool:
        return bool(self._selected_map_path and os.path.exists(self._selected_map_path))

    def load_map_path(self, path: str) -> bool:
        mp = self._normalize_map_path(path)
        if not mp or not os.path.exists(mp):
            self._selected_map_path = ""
            self._map_lbl.setText(tr("route_map_none"))
            self.set_status(f"Map không tồn tại: {path}")
            return False
        try:
            self.map_widget.load_map_file(mp)
        except Exception as e:
            self.set_status(f"Không load được map: {e}")
            return False
        self._selected_map_path = mp
        self._map_lbl.setText(os.path.basename(mp))
        return True

    def load_route_readonly(self, route: dict):
        """Operator: load lộ trình để xem + chạy."""
        self._editing_route_id = ""
        self._waypoints = copy.deepcopy(route.get("waypoints", []))
        map_ok = self.load_map_path(route.get("map_path", ""))
        self._refresh_wp_list()
        self._start_btn.setEnabled(bool(self._waypoints) and map_ok)

    def load_route_for_edit(self, route: dict):
        """Engineer: load route vào Navigation để chỉnh sửa và lưu đè."""
        self._editing_route_id = str(route.get("id", ""))
        self._waypoints = copy.deepcopy(route.get("waypoints", []))
        map_ok = self.load_map_path(route.get("map_path", ""))
        self._refresh_wp_list()
        if hasattr(self, "_route_name"):
            self._route_name.setText(str(route.get("name", "")))
        self._start_btn.setEnabled(bool(self._waypoints) and map_ok)
        self.set_status(f"Đang chỉnh sửa lộ trình: {route.get('name', '-')}")

    # ── Waypoint logic ───────────────────────────────────────────────

    def _on_map_click(self, wx: float, wy: float):
        """Engineer: click trái map → thêm waypoint."""
        if any(w["label"] == "E" for w in self._waypoints):
            return
        idx = len(self._waypoints)
        wp = {
            "label": str(idx + 1),
            "x": wx, "y": wy,
            "tasks": []
        }
        self._waypoints.append(wp)
        self._refresh_wp_list()
        self._wp_list.setCurrentRow(idx)
        self._start_btn.setEnabled(True)

    def _add_end(self):
        if not self._waypoints or any(w["label"] == "E" for w in self._waypoints):
            return
        last = self._waypoints[-1]
        self._waypoints.append({
            "label": "E", "x": last["x"], "y": last["y"], "tasks": []})
        self._refresh_wp_list()

    def _del_wp(self):
        row = self._wp_list.currentRow()
        if 0 <= row < len(self._waypoints):
            self._waypoints.pop(row)
            self._relabel()
            self._refresh_wp_list()
            self._task_box.setVisible(False)
            self._start_btn.setEnabled(bool(self._waypoints))

    def _clear_wps(self):
        self._waypoints.clear()
        self.map_widget.clear_waypoints()
        self._wp_list.clear()
        self._start_btn.setEnabled(False)
        if self._role == "engineer":
            self._task_box.setVisible(False)
            self._task_editor.clear()

    def _relabel(self):
        """Re-index labels 1,2,3,... sau khi xoá."""
        n = 1
        for wp in self._waypoints:
            if wp["label"] != "E":
                wp["label"] = str(n)
                n += 1

    def _on_wp_select(self, row: int):
        self._selected_wp_idx = row
        ok = 0 <= row < len(self._waypoints)
        if hasattr(self, '_del_wp_btn'):
            self._del_wp_btn.setEnabled(ok)
        if self._role == "engineer" and ok:
            wp = self._waypoints[row]
            self._wp_info_lbl.setText(
                f"WP {wp['label']}  ({wp['x']:.2f}, {wp['y']:.2f})")
            self._task_editor.load_tasks(wp.get("tasks", []))
            self._task_box.setVisible(True)

    def _on_tasks_changed(self, tasks: list[dict]):
        """TaskEditor emit → lưu vào waypoint đang chọn."""
        if 0 <= self._selected_wp_idx < len(self._waypoints):
            self._waypoints[self._selected_wp_idx]["tasks"] = tasks

    def _refresh_wp_list(self):
        self._wp_list.clear()
        pts = []
        for wp in self._waypoints:
            n_tasks = len(wp.get("tasks", []))
            extra = f" · {n_tasks} task" if n_tasks else ""
            self._wp_list.addItem(
                f"  {wp['label']}  ({wp['x']:.2f}, {wp['y']:.2f}){extra}")
            pts.append((wp["x"], wp["y"], wp["label"]))
        self.map_widget.set_waypoints(pts)

    # ── Map / Pose ───────────────────────────────────────────────────

    def _load_map(self):
        path, _ = QFileDialog.getOpenFileName(
            self, tr("route_select_map"),
            os.path.expanduser("~/maps"), "Map (*.yaml)")
        if not path:
            return
        self.load_map_path(path)

    def _toggle_pose_mode(self, checked: bool):
        if checked:
            self.map_widget.set_mode(MODE_POSE)
            self._pose_btn.setText(tr("nav_pose_active"))
        else:
            self.map_widget.set_mode(MODE_GOAL)
            self.map_widget.clear_pose_estimate()
            self._pose_btn.setText(tr("nav_pose_enable"))

    # ── Route save ───────────────────────────────────────────────────

    def _save_route(self):
        name = self._route_name.text().strip()
        if not name:
            QMessageBox.warning(self, "Lỗi", "Vui lòng nhập tên lộ trình.")
            return
        if not self._waypoints:
            QMessageBox.warning(self, "Lỗi", "Chưa có waypoint nào.")
            return
        mp = self._normalize_map_path(self._selected_map_path)
        if not mp or not os.path.exists(mp):
            QMessageBox.warning(self, "Lỗi", "Route phải có map .yaml hợp lệ trước khi lưu.")
            return
        self._selected_map_path = mp
        self.save_route_signal.emit(
            name, self._selected_map_path,
            copy.deepcopy(self._waypoints), self._editing_route_id)
        self._route_name.clear()
        self._editing_route_id = ""

    # ── Mission buttons ──────────────────────────────────────────────

    def _on_start(self):
        if not self._waypoints:
            return
        if not self._has_valid_map():
            QMessageBox.warning(self, "Lỗi", "Chưa có map hợp lệ, không thể chạy mission.")
            return
        self.run_mission_signal.emit(copy.deepcopy(self._waypoints))

    def _on_pause(self):
        """FIX: emit signal lên MainWindow để thực sự pause mission."""
        self._pause_btn.setEnabled(False)
        self._resume_btn.setEnabled(True)
        self.pause_signal.emit()

    def _on_resume(self):
        """FIX: emit signal lên MainWindow để thực sự resume mission."""
        self._pause_btn.setEnabled(True)
        self._resume_btn.setEnabled(False)
        self.resume_signal.emit()

    def retranslate(self):
        self.error_header.retranslate()
        self._vel_panel.retranslate()
        self._pose_btn.setText(tr("nav_pose_enable"))
        self._nav_toggle_btn.setText(
            "⬛ Tắt NAV" if self._nav_toggle_btn.isChecked() else "▶ Bật NAV")
        self._start_btn.setText(tr("nav_start"))
        self._stop_btn.setText(tr("nav_stop"))
        self._pause_btn.setText(tr("nav_pause"))
        self._resume_btn.setText(tr("nav_resume"))