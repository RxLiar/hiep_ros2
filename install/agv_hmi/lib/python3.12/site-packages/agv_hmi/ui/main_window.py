"""
main_window.py — AMR Pacific Autonomous Robot v4.3.4

Giữ nguyên hoàn toàn v4.2.6 (threading fix qua _nav_result_signal /
_nav_feedback_signal, wire velocity/pose_estimate từ RoutesPage,
_ensure_nav_running / _toggle_nav_launch / _run_route_from_list),
chỉ thêm/sửa:

  [D] Stuck-handling: khi 1 waypoint (chỉ áp dụng cho mission chạy từ
      Routes screen — mission_source == "routes") thất bại giữa lộ
      trình, KHÔNG còn reset toàn bộ về waypoint đầu như trước. Thay
      vào đó mission chuyển sang trạng thái "stuck", giữ nguyên
      _mission_wps/_mission_idx/_active_record/_cargo_count, và
      RoutesPage hiện banner với 3 lựa chọn:
        - Thử lại/Tiếp tục: gửi lại đúng goal của WP đang dở (Nav2 tự
          replan từ vị trí hiện tại — kể cả sau khi operator lái tay
          bằng joystick để thoát chỗ kẹt).
        - Bỏ qua điểm này: đánh dấu WP hiện tại là DONE (không chạy
          task), nhảy sang WP kế tiếp.
        - Huỷ toàn bộ: y hệt Stop hiện tại — đóng log "failed", cargo
          đã chở tính đến lúc đó vẫn được lưu.
      Mission chạy từ Navigation screen (Engineer test nhanh waypoint)
      KHÔNG bị ảnh hưởng — vẫn giữ hành vi cũ (fail = reset).

  [E] Auto / Manual mode (chỉ áp dụng cho Routes screen):
        - Auto: hành vi cũ + Lặp lại + Stuck-handling ở trên, chạy
          liên tục không cần can thiệp.
        - Manual: mỗi waypoint tách thành 2 bước bấm riêng biệt —
          "Đến WP" rồi "Xác nhận/Thực hiện Task" — nút Start đổi nhãn
          động theo bước sắp tới. Dùng chung cơ chế Stuck-handling khi
          bước "Đến WP" lỗi, và dùng chung cơ chế Lặp lại khi hết 1
          lượt (quay lại WP1, tiếp tục dừng-chờ-bấm).

  [FIX-1] _cleanup_leaving_page(): rời Navigation/Routes KHÔNG còn tự
      _stop_nav_launch() nữa — chỉ dừng mission đang chạy (an toàn: robot
      không nên tự di chuyển khi operator không theo dõi màn hình điều
      khiển). Nav2 (map_server/amcl/bt_navigator...) chỉ tắt khi: bấm nút
      "Tắt NAV", đóng app, hoặc đổi map (xử lý sẵn trong
      _ensure_nav_running). Trước đây chỉ cần lướt sang xem Conveyor/
      Settings là Nav2 bị kill/restart — gây trễ và khó chịu khi vận hành.

  [FIX-2] Bộ đếm retry Nav2 (_start_nav_retry / _on_nav_retry_tick):
      trước đây "elapsed = 60 - attempts" bị hiểu nhầm là giây trong khi
      thực chất là số tick (mỗi tick 500ms) — hiển thị sai kiểu "1/30s"
      rồi nhảy tới "60/30s". Đã sửa tính đúng theo giây thực tế, và khi
      hết thời gian sẽ báo rõ "Không bắt đầu được mission" thay vì lẫn
      với "mission failed".

  [NEW] switch_user_signal + _on_switch_user_clicked(): cho phép quay về
      màn hình Login để đổi người vận hành (operator/engineer) mà không
      cần khởi động lại toàn bộ ROS node — xem thêm agv_hmi/main.py
      (class Session quản lý vòng đời MainWindow qua ROS node dùng chung).
"""
import os
import copy
import json

from agv_hmi.ui.process_manager import ManagedLaunch
from agv_hmi.core.alarm_manager import AlarmManager
from agv_hmi.core.health_model import HealthModel
from agv_hmi.core import audit_log, prefs
from agv_hmi.ui.alarm_center import AlarmCenterPage
from agv_hmi.ui.diagnostics_page import DiagnosticsPage
from agv_hmi.ui.preflight import PreflightDialog
from agv_hmi.ui.toast import ToastManager
from agv_hmi.ui.reports_page import ReportsPage
from agv_hmi.ui.stations_page import StationsPage
from agv_hmi.ui.schedule_page import SchedulePage
from agv_hmi.ui.dashboard import DashboardWidget
from agv_hmi.ui.safety_page import SafetyPage
from agv_hmi.ui.safety_override import OVERRIDE
from agv_hmi.ui.subtabs import SubTabBar
from agv_hmi.core import nav_groups as NG
from agv_hmi.ui.joystick_widget import JoystickWidget
from agv_hmi.ui.forklift_page import ForkliftPage
from agv_hmi.core import robot_profile as RP
import time as _time
import uuid as _uuid
from agv_hmi.ui.map_widget import MapWidget
_json = json
from agv_hmi.version import full_label

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QStackedWidget, QLabel, QPushButton, QMessageBox, QApplication
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer, QVariantAnimation, QEvent
from PyQt6.QtGui import QKeySequence, QShortcut

from agv_hmi.ui.i18n import tr, set_lang, get_lang, LANGUAGES
from agv_hmi.ui import anim
from agv_hmi.ui.anim import AnimatedStack
from agv_hmi.ui.collapsible import CollapsiblePanel, collapse_panels
from agv_hmi.ui.nav_button import NavButton
from agv_hmi.ui.title_bar import TitleBar
from agv_hmi.ui.language_selector import LanguageSelector
from agv_hmi.ui.home_page import HomePage
from agv_hmi.ui.mapping_page import MappingPage
from agv_hmi.ui.navigation_page import NavigationPage
from agv_hmi.ui.routes_page import (
    RoutesPage, WP_PENDING, WP_MOVING, WP_TASK, WP_DONE, WP_STUCK,
)
from agv_hmi.ui.conveyor_panel import ConveyorPage
from agv_hmi.ui.map_library_page import MapLibraryPage
from agv_hmi.ui.settings_page import SettingsPage
from agv_hmi.ui.fleet_page import FleetPage
import agv_hmi.ui.route_manager as RM
from agv_hmi.fleet.map_registry import HmiFleetMapRegistry

NAV_CMD = ['ros2', 'launch', 'mec_mobile_navigation', 'navigation.launch.py']
NAV_KILL_PATTERNS = [
    # map_server phải được dọn khi đổi map, nếu không Nav2/RViz
    # có thể vẫn giữ map cũ.
    'map_server',
    'amcl',
    'bt_navigator',
    'controller_server',
    'planner_server',
    'behavior_server',
    'waypoint_follower',
    'lifecycle_manager',
]


# ── Sidebar ──────────────────────────────────────────────────────────

class Sidebar(QWidget):
    """Main navigation. Animated: expanded (labels) <-> compact icon rail <-> hidden (focus mode)."""
    page_changed          = pyqtSignal(int)
    retranslate_requested = pyqtSignal()
    toggle_requested      = pyqtSignal()      # user clicked the collapse button
    main_clicked          = pyqtSignal(int)   # 4.10.0: a MAIN tab button was pressed (MainWindow picks the sub-page)

    IDX_HOME     = 0
    IDX_MAPPING  = 1
    IDX_NAV      = 2
    IDX_ROUTES   = 3
    IDX_CONVEYOR = 4
    IDX_MAPLIB   = 5
    IDX_SETTINGS = 6
    IDX_FLEET    = 7
    IDX_ALARMS   = 8
    IDX_DIAG     = 9
    IDX_REPORTS  = 10
    IDX_STATIONS = 11
    IDX_SCHEDULE = 12
    IDX_SAFETY   = 13
    IDX_FORKLIFT = 14

    EXPANDED_W = 210
    COMPACT_W  = 64

    def __init__(self, role: str = "operator"):
        super().__init__()
        self.setObjectName("Sidebar")
        self._role = role
        self._btns: list[tuple[NavButton, int, str]] = []
        self._compact = False
        self._hidden = False
        self._touch = False
        self._width_anim = QVariantAnimation(self)
        self._width_anim.setEasingCurve(anim.EASE_INOUT)
        self._width_anim.valueChanged.connect(lambda v: self.setFixedWidth(int(v)))
        self._width_anim.finished.connect(self._after_width)
        self.setFixedWidth(self.EXPANDED_W)
        self._build()

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        logo_w = QWidget(); logo_w.setObjectName("LogoArea")
        logo_w.setFixedHeight(64)
        ll = QHBoxLayout(logo_w); ll.setContentsMargins(14, 0, 14, 0)
        self._logo = QLabel("🤖")
        self._logo.setFixedSize(36, 36)
        self._logo.setStyleSheet(
            "background:#185FA5;border-radius:8px;font-size:20px;"
            "qproperty-alignment:AlignCenter;")
        self._txt = QWidget(); tl = QVBoxLayout(self._txt)
        tl.setContentsMargins(0, 0, 0, 0); tl.setSpacing(1)
        self._app_lbl = QLabel(tr("app_name"))
        self._app_lbl.setObjectName("AppTitle")
        self._app_lbl.setWordWrap(True)
        self._ver_lbl = QLabel(full_label())
        self._ver_lbl.setObjectName("AppVersion")
        tl.addWidget(self._app_lbl); tl.addWidget(self._ver_lbl)
        ll.addWidget(self._logo); ll.addSpacing(8); ll.addWidget(self._txt, 1)
        lay.addWidget(logo_w)

        nav_w = QWidget()
        self._nav_lay = QVBoxLayout(nav_w)
        self._nav_lay.setContentsMargins(0, 10, 0, 0)
        self._nav_lay.setSpacing(2)
        lay.addWidget(nav_w)
        lay.addStretch()

        self._collapse_btn = NavButton("«", tr("sb_collapse"))
        self._collapse_btn.setObjectName("NavBtnAnim")
        self._collapse_btn.clicked.connect(self.toggle_requested)
        cw = QWidget(); cl = QVBoxLayout(cw); cl.setContentsMargins(0, 0, 0, 4)
        cl.addWidget(self._collapse_btn)
        lay.addWidget(cw)

        self._bot = QWidget()
        bl = QVBoxLayout(self._bot); bl.setContentsMargins(12, 4, 12, 10); bl.setSpacing(5)
        self._lang_selector = LanguageSelector(compact=True)
        self._lang_selector.language_changed.connect(
            lambda _: self.retranslate_requested.emit())
        bl.addWidget(self._lang_selector)
        self._mode_lbl = QLabel()
        self._mode_lbl.setStyleSheet("font-size:10px;font-weight:600;")
        bl.addWidget(self._mode_lbl)
        sr = QHBoxLayout()
        self._dot  = QLabel("●")
        self._dot.setStyleSheet("color:#484F58;font-size:10px;")
        self._stxt = QLabel("ROS2"); self._stxt.setObjectName("StatusLabel")
        sr.addWidget(self._dot); sr.addWidget(self._stxt); sr.addStretch()
        bl.addLayout(sr)
        lay.addWidget(self._bot)

        self._rebuild_nav()
        self._update_mode()

    def _rebuild_nav(self):
        while self._nav_lay.count():
            item = self._nav_lay.takeAt(0)
            if item.widget(): item.widget().deleteLater()
        self._btns.clear()

        # 4.10.0: only the MAIN tabs; every other page is a sub-tab (see core/nav_groups.py)
        for idx in NG.main_pages(self._role):
            icon, key = NG.MAIN_ICON[idx], NG.MAIN_KEY[idx]
            b = NavButton(icon, tr(key))
            b.setFixedHeight(54 if self._touch else 38)
            b.set_compact(self._compact)
            b.clicked.connect(lambda _, i=idx: self.main_clicked.emit(i))
            self._nav_lay.addWidget(b)
            self._btns.append((b, idx, key))
        self._nav_lay.addStretch()

    # ── state ────────────────────────────────────────────────────────
    def is_compact(self) -> bool:
        return self._compact

    def is_hidden_rail(self) -> bool:
        return self._hidden

    def set_touch(self, on: bool):
        self._touch = bool(on)
        for b, _, _ in self._btns:
            b.setFixedHeight(54 if on else 38)
        self._collapse_btn.setFixedHeight(54 if on else 38)

    def set_compact(self, compact: bool, animate: bool = True):
        compact = bool(compact)
        if compact == self._compact:
            return
        self._compact = compact
        for b, _, _ in self._btns:
            b.set_compact(compact)
        self._collapse_btn.set_compact(compact)
        self._collapse_btn.set_icon_text("»" if compact else "«")
        self._collapse_btn.set_label(tr("sb_expand") if compact else tr("sb_collapse"))
        if compact:
            self._set_texts_visible(False)
        else:
            d = anim.dur(anim.DUR_MED) if animate else 0
            if d == 0:
                self._set_texts_visible(True)
            else:   # let the rail grow a little before the labels appear
                QTimer.singleShot(int(d * 0.55), self._show_texts_if_expanded)
        self._update_mode()
        self._apply_width(animate)

    def set_hidden(self, hidden: bool, animate: bool = True):
        hidden = bool(hidden)
        if hidden == self._hidden:
            return
        self._hidden = hidden
        if not hidden:
            self.show()
        self._apply_width(animate)

    def _target_width(self) -> int:
        if self._hidden:
            return 0
        return self.COMPACT_W if self._compact else self.EXPANDED_W

    def _apply_width(self, animate: bool):
        target = self._target_width()
        self._width_anim.stop()
        d = anim.dur(anim.DUR_MED) if animate else 0
        if d == 0:
            self.setFixedWidth(target)
            self._after_width()
            return
        self._width_anim.setDuration(d)
        self._width_anim.setStartValue(self.width())
        self._width_anim.setEndValue(target)
        self._width_anim.start()

    def _after_width(self):
        if self._hidden:
            self.hide()

    def _show_texts_if_expanded(self):
        if not self._compact:
            self._set_texts_visible(True)

    def _set_texts_visible(self, vis: bool):
        self._txt.setVisible(vis)
        self._lang_selector.setVisible(vis)
        self._stxt.setVisible(vis)

    def switch(self, idx: int):
        main = NG.group_main(idx)               # a sub-page highlights the main tab it belongs to
        for b, bidx, _ in self._btns:
            b.set_active(bidx == main)
        self.page_changed.emit(idx)

    def set_ros_status(self, txt: str, ok: bool = True):
        self._dot.setStyleSheet(
            f"color:{'#3FB950' if ok else '#E3B341'};font-size:10px;")
        self._stxt.setText(txt)
        self._dot.setToolTip(txt)

    def _update_mode(self):
        mode  = tr("mode_engineer") if self._role == "engineer" else tr("mode_operator")
        color = "#58A6FF" if self._role == "engineer" else "#3FB950"
        self._mode_lbl.setText("●" if self._compact else f"● {mode}")
        self._mode_lbl.setToolTip(mode)
        self._mode_lbl.setAlignment(
            Qt.AlignmentFlag.AlignCenter if self._compact else Qt.AlignmentFlag.AlignLeft)
        self._mode_lbl.setStyleSheet(
            f"color:{color};font-size:10px;font-weight:600;")

    def retranslate(self):
        self._app_lbl.setText(tr("app_name"))
        self._update_mode()
        for b, _, key in self._btns:
            b.set_label(tr(key))
        self._collapse_btn.set_label(tr("sb_expand") if self._compact else tr("sb_collapse"))
        if hasattr(self, "_lang_selector"):
            self._lang_selector.retranslate()


# ── MainWindow ────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    velocity_signal      = pyqtSignal(float, float)
    pose_estimate_signal = pyqtSignal(float, float, float)
    motor_enable_signal  = pyqtSignal(bool)   # -> RosInterface.set_motor_enable
    soft_estop_signal    = pyqtSignal()       # -> RosInterface.soft_estop
    light_mask_signal    = pyqtSignal(int)    # -> RosInterface.publish_light_mask
    safety_config_signal = pyqtSignal(str)    # -> RosInterface.publish_safety_config
    safety_override_signal = pyqtSignal(bool)  # -> RosInterface.publish_safety_override (Engineer teleop, 5 Hz)
    fleet_robot_id_signal = pyqtSignal(str)    # -> RosInterface.set_fleet_robot_id (Fleet control panel)
    forklift_cmd_signal  = pyqtSignal(str)    # -> RosInterface.publish_forklift_command

    # [NEW] Đổi người vận hành: quay về màn hình Login, ROS node vẫn sống.
    switch_user_signal   = pyqtSignal()

    # FIX threading: marshal NavClient callback (chạy trong rclpy
    # spin thread) về Qt main thread an toàn qua QueuedConnection.
    _nav_result_signal   = pyqtSignal(bool, int)
    _nav_feedback_signal = pyqtSignal(int, float, int)

    def __init__(self, role: str = "operator", theme_mgr=None):
        super().__init__()
        self._role      = role
        self._theme_mgr = theme_mgr

        # [FIX] Phân biệt "đóng cửa sổ để đổi người vận hành" (Session sẽ
        # mở lại Login, KHÔNG thoát app) với "đóng cửa sổ thật" (bấm nút X
        # titlebar, Alt+F4...) — trường hợp sau PHẢI gọi app.quit(), nếu
        # không process sẽ chạy ngầm mãi mãi dù cửa sổ đã biến mất (đây
        # chính là nguyên nhân app không exit được / Ctrl+C không có tác
        # dụng, vì QApplication.exec() không bao giờ return).
        self._switching_user = False

        # Cleanup có thể được gọi từ closeEvent() và aboutToQuit.
        # Guard này bảo đảm Mapping/Nav2 chỉ bị stop đúng một lần.
        self._shutdown_done = False

        # 4.5.0: alarms / health / safety state
        self.alarms = AlarmManager()
        self.health = HealthModel()
        audit_log.set_user(role, role)
        self._scan_seen = False
        self._scan_low_ticks = 0
        self._loc_bad_ticks = 0
        self._estop_active = False
        self._joy_active = False
        self._last_tower_mask = None
        self._skip_preflight_once = False
        self._queue_run_active = False
        self._housekeeping_ticks = 0
        self._fork_wait = None        # task waiting for the forklift node: dict
        self._fork_seen_status = 0.0

        self.setWindowTitle(tr("app_name"))
        self.setMinimumSize(1024, 600)
        self.resize(1440, 860)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)

        self.nav_client       = None
        self.map_saver        = None
        self._conveyor_cmd_cb = None
        self._fleet_map_context_cb = None
        self._fleet_map_registry = HmiFleetMapRegistry()

        self._mission_wps:     list[dict] = []
        self._mission_idx:     int  = 0
        self._mission_running: bool = False
        self._mission_paused:  bool = False
        self._cancel_reason:   str | None = None
        self._mission_token:   int = 0
        self._mission_timers:  list[QTimer] = []
        self._confirm_continue = None

        # Pha hiện tại của mission. Pause chỉ hợp lệ khi robot đang
        # di chuyển; không huỷ timer task để tránh chạy task lần thứ hai.
        self._mission_phase = "idle"

        self._nav_retry_timer:    QTimer | None = None
        self._nav_retry_attempts: int = 0
        # [FIX-2] Lưu tổng số attempt để tính đúng số giây đã trôi qua,
        # thay vì lấy trực tiếp số tick làm số giây.
        self._nav_retry_total:    int = 60
        self._nav_retry_wps:      list[dict] = []
        self._nav_retry_token:    int = 0

        # [D][E] Nguồn khởi tạo mission: "routes" (RoutesPage) hoặc
        # "navigation" (NavigationPage - Engineer test nhanh waypoint).
        # Stuck-handling và Auto/Manual CHỈ áp dụng khi mission_source
        # == "routes", để không ảnh hưởng luồng test waypoint hiện có
        # của Engineer trên Navigation screen.
        self._mission_source:  str = "routes"

        # [D] Stuck-handling state
        self._mission_stuck: bool = False
        self._stuck_wp_idx:  int = -1

        # [E] Auto/Manual: False = Auto (mặc định), True = Manual.
        # Đồng bộ với toggle trên RoutesPage qua mode_changed signal.
        self._manual_mode: bool = False
        # Hành động đang chờ operator bấm "bước kế" trong Manual mode:
        # ("task", idx) = đang chờ chạy Task tại WP idx (hoặc xác nhận
        #                  nếu WP không có task).
        # ("move", idx) = task của WP idx đã xong; khi bấm sẽ đánh dấu
        #                  WP idx DONE rồi chuyển sang WP kế tiếp.
        # ("move_after_skip", idx) = WP lỗi đã được bỏ qua và mission_idx
        #                  đã trỏ tới WP kế; khi bấm chỉ gửi goal WP kế.
        self._manual_pending_action: tuple[str, int] | None = None

        self._last_x   = 0.0
        self._last_y   = 0.0
        self._last_yaw = 0.0
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0

        self._current_page_idx = None
        self._nav_launcher     = ManagedLaunch(
            "Navigation", NAV_CMD, kill_patterns=NAV_KILL_PATTERNS)

        # 4.7.0: sidebar / focus mode state
        self._sb_user_compact = bool(prefs.get("ui_sidebar_compact"))
        self._sb_auto_compact = False
        self._focus_mode = False
        self._focus_collapsed: list = []

        self._build()
        self._wire()
        self._init_ui_extras()

        # FIX threading: connect signal nội bộ — Qt tự dùng
        # QueuedConnection khi emit từ thread khác main thread.
        self._nav_result_signal.connect(self._on_wp_result)
        self._nav_feedback_signal.connect(self._on_feedback)

        self._sidebar.switch(Sidebar.IDX_HOME)
        if prefs.get("operator_kiosk"):
            QTimer.singleShot(0, lambda: self.apply_touch_mode(True))

        self._housekeeping_timer = QTimer(self)
        self._housekeeping_timer.timeout.connect(self._housekeeping)
        self._housekeeping_timer.start(1000)
        QTimer.singleShot(2500, self.publish_saved_safety_config)
        audit_log.audit("app_start")

    # ── Build ────────────────────────────────────────────────────────

    def _build(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0); root.setSpacing(0)

        self._titlebar = TitleBar(self)
        root.addWidget(self._titlebar)

        self._estop_banner = QLabel(tr("estop_banner"))
        self._estop_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._estop_banner.setStyleSheet(
            "background:#B62324;color:white;font-size:15px;font-weight:800;padding:8px;")
        self._estop_banner.setVisible(False)
        root.addWidget(self._estop_banner)

        # 4.9.0: progress of a running mission while another page is open
        self._mission_bar = QWidget()
        self._mission_bar.setObjectName("MissionBar")
        self._mission_bar.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        mb = QHBoxLayout(self._mission_bar)
        mb.setContentsMargins(14, 6, 14, 6); mb.setSpacing(8)
        self._mission_bar_lbl = QLabel()
        self._mission_bar_lbl.setStyleSheet("font-weight:700;")
        self._mission_bar_ok = QPushButton(); self._mission_bar_view = QPushButton(); self._mission_bar_stop = QPushButton()
        self._mission_bar_stop.setObjectName("BtnDanger")
        for b in (self._mission_bar_ok, self._mission_bar_view, self._mission_bar_stop):
            b.setFixedHeight(28)
        self._mission_bar_ok.clicked.connect(self._confirm_task_continue)
        self._mission_bar_view.clicked.connect(self._mission_bar_goto)
        self._mission_bar_stop.clicked.connect(self._mission_bar_stop_clicked)
        mb.addWidget(self._mission_bar_lbl, 1)
        for b in (self._mission_bar_ok, self._mission_bar_view, self._mission_bar_stop):
            mb.addWidget(b)
        self._mission_bar.setVisible(False)
        root.addWidget(self._mission_bar)
        self._mission_bar_state = ""

        body = QWidget()
        body_lay = QHBoxLayout(body)
        body_lay.setContentsMargins(0, 0, 0, 0); body_lay.setSpacing(0)

        self._sidebar = Sidebar(role=self._role)
        body_lay.addWidget(self._sidebar)

        self._pages    = AnimatedStack()
        self._home     = HomePage()
        self._mapping  = MappingPage()
        self._nav      = NavigationPage(role=self._role)
        self._routes   = RoutesPage(role=self._role)
        self._conveyor = ConveyorPage()
        self._maplib   = MapLibraryPage()
        self._settings = SettingsPage(role=self._role, theme_mgr=self._theme_mgr)
        self._fleet    = FleetPage()
        self._alarm_page = AlarmCenterPage(self.alarms)
        self._diag     = DiagnosticsPage(self.health)
        self._reports  = ReportsPage(role=self._role)
        self._stations = StationsPage()
        self._schedule = SchedulePage()
        self._safety   = SafetyPage()
        self._forklift = ForkliftPage()

        for p in (self._home, self._mapping, self._nav, self._routes,
                  self._conveyor, self._maplib, self._settings, self._fleet,
                  self._alarm_page, self._diag, self._reports,
                  self._stations, self._schedule, self._safety, self._forklift):
            self._pages.addWidget(p)

        content = QWidget()
        content_lay = QVBoxLayout(content)
        content_lay.setContentsMargins(0, 0, 0, 0); content_lay.setSpacing(0)
        self._subtabs = SubTabBar()
        content_lay.addWidget(self._subtabs)
        content_lay.addWidget(self._pages, 1)
        body_lay.addWidget(content, 1)
        root.addWidget(body)
        self._last_sub: dict[int, int] = {}        # main tab -> sub-page used last

        self._toasts = ToastManager(central)

        self._dashboard = DashboardWidget()
        self._home.add_dashboard(self._dashboard)
        self._apply_robot_profile(self._safety.active_profile())
        self._nav.set_robot_type(self._safety.active_profile().get("robot_type", "agv"))

    # ── Wire ─────────────────────────────────────────────────────────

    def _wire(self):
        self._titlebar.minimize_clicked.connect(self.showMinimized)
        self._titlebar.maximize_clicked.connect(self._toggle_maximize)
        self._titlebar.close_clicked.connect(self.close)
        if hasattr(self._titlebar, "language_changed"):
            self._titlebar.language_changed.connect(
                lambda _: self._retranslate_all())

        self._titlebar.estop_clicked.connect(self._on_soft_estop)
        self._titlebar.safety_chip_clicked.connect(
            lambda: self._sidebar.switch(Sidebar.IDX_SAFETY if self._role == "engineer" else Sidebar.IDX_HOME))
        self._titlebar.nav_chip_clicked.connect(lambda: self._sidebar.switch(Sidebar.IDX_NAV))
        self._titlebar.fleet_chip_clicked.connect(lambda: self._sidebar.switch(Sidebar.IDX_FLEET))
        self._titlebar.mission_chip_clicked.connect(
            lambda: self._mission_bar_goto() if self._mission_running else self._sidebar.switch(Sidebar.IDX_ROUTES))
        self._titlebar.alarms_clicked.connect(
            lambda: self._sidebar.switch(Sidebar.IDX_ALARMS))
        self._safety.profile_applied.connect(self._on_profile_applied)

        # 4.8.0: Engineer can switch the safety zones off while teleoperating (joystick / gamepad).
        # The switch is only a request that is re-sent every 200 ms; the ROS node drops it after <= 2 s.
        OVERRIDE.set(False)
        for pg in (self._mapping, self._nav, self._routes):
            js = getattr(pg, "_joystick", None)
            if js is not None:
                js.set_engineer(self._role == "engineer")
        # 4.11.0: Fleet control (start fleet / connect this robot from the HMI)
        self._fleet.control.set_engineer(self._role == "engineer")
        self._fleet.control.robot_id_changed.connect(self.fleet_robot_id_signal.emit)
        self._fleet.control.status_changed.connect(self._titlebar.set_fleet_chip)
        QTimer.singleShot(3000, self._fleet.control.auto_start)
        self._override_timer = QTimer(self)
        self._override_timer.setInterval(200)
        self._override_timer.timeout.connect(self._refresh_override)
        OVERRIDE.changed.connect(self._on_override_changed)
        self._forklift.command_requested.connect(self.forklift_cmd_signal)
        self._forklift.params_save_requested.connect(self._safety.apply_external)
        self._stations.set_pose_requested.connect(self.pose_estimate_signal)
        self._stations.goto_requested.connect(self._goto_station)
        self._schedule.run_queue_requested.connect(lambda: self._try_run_queue(manual=True))
        self._settings._sys_box.touch_mode_changed.connect(self.apply_touch_mode)
        self.alarms.changed.connect(self._on_alarms_changed)
        self.alarms.raised.connect(self._on_alarm_raised)
        self.alarms.raised.connect(self._diag.on_alarm)

        self._sidebar.page_changed.connect(self._switch_page)
        self._sidebar.main_clicked.connect(self._on_main_clicked)
        self._subtabs.page_requested.connect(self._sidebar.switch)
        self._sidebar.retranslate_requested.connect(self._retranslate_all)
        self._sidebar.toggle_requested.connect(self.toggle_sidebar)
        self._titlebar.menu_clicked.connect(self.toggle_sidebar)
        self._titlebar.focus_clicked.connect(self.toggle_focus_mode)

        self._home.connection_toggle.connect(self._on_connection_toggle)
        self._home.motor_enable_requested.connect(self.motor_enable_signal)
        # [NEW] Nút đổi người vận hành trên Home
        self._home.switch_user_requested.connect(self._on_switch_user_clicked)

        self._mapping.velocity_signal.connect(self.velocity_signal)
        self._mapping.save_map_requested.connect(self._do_save_map)
        self._mapping.mapping_started.connect(
            lambda: self._sidebar.set_ros_status("SLAM đang chạy...", ok=True))
        self._mapping.mapping_stopped.connect(
            lambda: self._sidebar.set_ros_status("SLAM đã dừng", ok=False))

        self._nav.velocity_signal.connect(self.velocity_signal)
        self._nav.pose_estimate_signal.connect(self.pose_estimate_signal)
        self._nav.run_mission_signal.connect(self._run_navigation_mission)
        self._nav.stop_signal.connect(self._stop_mission)
        self._nav.pause_signal.connect(self._pause_mission)
        self._nav.resume_signal.connect(self._resume_mission)
        self._nav.save_route_signal.connect(self._save_route)
        self._nav.nav_launch_toggle_signal.connect(self._toggle_nav_launch)

        self._routes.run_route_signal.connect(self._run_route_from_list)
        self._routes.pause_signal.connect(self._pause_mission)
        self._routes.resume_signal.connect(self._resume_mission)
        self._routes.stop_signal.connect(self._stop_mission)

        # [+] Joystick và Pose Estimate từ RoutesPage v2.3
        self._routes.velocity_signal.connect(self.velocity_signal)
        self._routes.pose_estimate_signal.connect(self.pose_estimate_signal)

        if hasattr(self._routes, "confirm_signal"):
            self._routes.confirm_signal.connect(self._confirm_task_continue)
        if hasattr(self._routes, "edit_route_signal"):
            self._routes.edit_route_signal.connect(self._edit_route)

        # [E] Auto/Manual mode + bước tiếp theo trong Manual
        self._routes.mode_changed.connect(self._on_mode_changed)
        self._routes.manual_continue_signal.connect(self._on_manual_continue)

        # [D] Stuck-handling: Thử lại / Bỏ qua / Huỷ
        self._routes.retry_wp_signal.connect(self._on_retry_wp)
        self._routes.skip_wp_signal.connect(self._on_skip_wp)
        self._routes.cancel_stuck_signal.connect(self._on_cancel_stuck)

        self._conveyor.conveyor_cmd.connect(self._on_conveyor_cmd)
        self._maplib.map_selected.connect(self._load_map_to_nav)

        # Ngôn ngữ đổi trong Settings: apply ngay cho toàn bộ MainWindow.
        self._settings.language_changed.connect(
            lambda _code: self._retranslate_all())

    # ── Page switch ───────────────────────────────────────────────────

    def _on_main_clicked(self, main_idx: int):
        """A main tab was pressed: another group -> its last sub-page; the same group -> its overview page."""
        self._sidebar.switch(NG.target_for_main_click(
            main_idx, self._current_page_idx, self._last_sub, self._role))

    def _switch_page(self, idx: int):
        old_idx = self._current_page_idx
        if (idx == Sidebar.IDX_MAPPING and old_idx is not None and old_idx != idx
                and self._mission_running):
            ans = QMessageBox.question(
                self, tr("mb_mapping_title"), tr("mb_mapping_warn"),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No)
            if ans != QMessageBox.StandardButton.Yes:
                self._sidebar.switch(old_idx)          # stay where we were (the mission keeps running)
                return
            self._stop_mission()
        if old_idx is not None and old_idx != idx:
            self._cleanup_leaving_page(old_idx, idx)

        self._current_page_idx = idx
        self._pages.setCurrentIndex(idx)
        main = NG.group_main(idx)
        self._last_sub[main] = idx
        self._subtabs.set_group(NG.subpages(main, self._role), idx)
        titles = {
            Sidebar.IDX_HOME:     tr("page_home"),
            Sidebar.IDX_MAPPING:  tr("page_mapping"),
            Sidebar.IDX_NAV:      tr("page_navigation"),
            Sidebar.IDX_ROUTES:   tr("page_routes"),
            Sidebar.IDX_CONVEYOR: tr("conv_title"),
            Sidebar.IDX_MAPLIB:   tr("maplib_title"),
            Sidebar.IDX_SETTINGS: tr("settings_title"),
            Sidebar.IDX_FLEET:    tr("page_fleet"),
            Sidebar.IDX_ALARMS:   tr("page_alarms"),
            Sidebar.IDX_DIAG:     tr("page_diag"),
            Sidebar.IDX_REPORTS:  tr("page_reports"),
            Sidebar.IDX_STATIONS: tr("page_stations"),
            Sidebar.IDX_SCHEDULE: tr("page_schedule"),
            Sidebar.IDX_SAFETY:   tr("page_safety"),
            Sidebar.IDX_FORKLIFT: tr("page_forklift"),
        }
        self._titlebar.set_page_title(titles.get(idx, ""))
        if self._focus_mode:
            self._focus_collapsed += collapse_panels(
                self._pages.currentWidget(), True, animate=False)
        self._update_mission_bar()
        if idx == Sidebar.IDX_ROUTES: self._routes.refresh()
        if idx == Sidebar.IDX_MAPLIB: self._maplib.refresh()
        if idx == Sidebar.IDX_FLEET:
            try:
                self._fleet.refresh_registered_maps()
            except Exception:
                pass

    def _cleanup_leaving_page(self, old_idx: int, new_idx: int):
        """Cleanup resources when changing pages.

        4.9.0: a running mission is a BACKGROUND task. Leaving Navigation / Routes / Fleet for any other
        page no longer stops it (the state machine lives in MainWindow, not in the page). A banner under
        the title bar shows its progress on the other pages. The only page that would break a mission is
        Mapping (SLAM and Nav2 fight over /map and the map->odom transform): _switch_page asks first.
        """
        if old_idx == Sidebar.IDX_MAPPING and NG.group_main(new_idx) != Sidebar.IDX_MAPPING:
            # leaving the Mapping tab stops SLAM; the Map library sub-tab belongs to it and keeps it running
            try:
                self._mapping.cleanup()
            except Exception:
                pass

    # ── Nav2 launch control ────────────────────────────────────────────

    def _toggle_nav_launch(self, start: bool):
        """
        Bật/tắt NAV2 từ nút trên Navigation screen.

        Nếu Navigation đã chọn map hợp lệ:
            start navigation.launch.py với map:=<map đang chọn>

        Nếu chưa chọn map:
            start navigation.launch.py KHÔNG truyền map:=...
            => navigation.launch.py tự dùng default my_map.yaml.
        """
        if start:
            map_path = getattr(self._nav, "_selected_map_path", "") or ""
            ok, msg = self._ensure_nav_running(map_path)
            try:
                self._nav.set_nav_launch_running(ok, msg)
            except Exception:
                pass
            self._sidebar.set_ros_status(msg, ok=ok)
        else:
            self._stop_nav_launch()

    def _ensure_nav_running(self, map_path: str = "") -> tuple[bool, str]:
        """
        Đảm bảo navigation.launch.py đang chạy với đúng map.

        map_path rỗng:
            chạy navigation.launch.py không truyền map:=...
            => launch file dùng default my_map.yaml.

        map_path có giá trị:
            chạy navigation.launch.py map:=<map_path>.

        Nếu Nav2 đang chạy với cùng extra_args thì giữ nguyên.
        Nếu Nav2 đang chạy với extra_args khác thì stop() rồi start() lại.
        """
        raw_map_path = str(map_path or "").strip()
        extra_args: list[str] = []
        display_map = "map mặc định"

        if raw_map_path:
            mp = self._normalize_map_path(raw_map_path)
            if not mp or not os.path.exists(mp):
                return False, f"Map không tồn tại: {mp}"
            extra_args = [f"map:={mp}"]
            display_map = os.path.basename(mp)

        if self._nav_launcher.is_running():
            current_args = getattr(self._nav_launcher, "current_extra_args", [])
            if current_args == extra_args:
                return True, f"Nav2 đang chạy đúng {display_map}"

            self._sidebar.set_ros_status(
                "Map thay đổi — đang khởi động lại Nav2...", ok=True)
            self._nav_launcher.stop()

        try:
            ok, msg = self._nav_launcher.start(extra_args=extra_args)
        except TypeError as e:
            # Báo lỗi rõ khi runtime vẫn đang import process_manager.py cũ.
            return False, (
                "process_manager.py đang là bản cũ, ManagedLaunch.start() "
                f"chưa hỗ trợ extra_args: {e}"
            )

        if ok:
            msg = f"Nav2 đã chạy với {display_map}"
        return ok, msg

    def _run_navigation_mission(self, wps: list[dict]):
        """
        Chạy waypoint từ Navigation screen.
        Trước khi gửi goal, đảm bảo Nav2/RViz dùng đúng map đang chọn
        trong Navigation. Nếu chưa chọn map thì dùng map mặc định.

        [D][E] mission_source = "navigation" — Stuck-handling và
        Auto/Manual KHÔNG áp dụng cho luồng này, giữ nguyên hành vi cũ.
        """
        self._mission_source = "navigation"
        if not self._robot_type_ok(wps):
            return
        if not self._preflight():
            return
        map_path = getattr(self._nav, "_selected_map_path", "") or ""
        ok, msg = self._ensure_nav_running(map_path)
        try:
            self._nav.set_nav_launch_running(ok, msg)
        except Exception:
            pass
        self._sidebar.set_ros_status(msg, ok=ok)

        if not ok:
            try:
                self._nav.set_status(msg)
            except Exception:
                pass
            return

        token = self._new_mission_token()
        self._cancel_mission_timers()
        self._clear_route_confirm()
        self._cancel_reason = None
        self._stop_nav_retry()

        try:
            self._nav.set_status("Đang chờ Nav2 action server sẵn sàng...")
        except Exception:
            pass

        self._start_nav_retry(wps, token, max_attempts=60)

    def _stop_nav_launch(self):
        ok, msg = self._nav_launcher.stop()
        try:
            self._nav.set_nav_launch_running(False, msg)
        except Exception:
            pass
        self._sidebar.set_ros_status(msg, ok=False)
        self.velocity_signal.emit(0.0, 0.0)

    def shutdown_processes(self):
        """Dừng mission, Mapping và Nav2 đúng một lần."""
        if self._shutdown_done:
            return
        self._shutdown_done = True

        try:
            self._stop_mission()
        except Exception as e:
            print(f"[Shutdown] Stop mission failed: {e}")

        try:
            self._mapping.cleanup()
        except Exception as e:
            print(f"[Shutdown] Mapping cleanup failed: {e}")

        try:
            self._diag.shutdown()
        except Exception as e:
            print(f"[Shutdown] Diagnostics cleanup failed: {e}")

        try:
            self._stop_nav_launch()
        except Exception as e:
            print(f"[Shutdown] Nav2 cleanup failed: {e}")

    # ── ROS → UI ─────────────────────────────────────────────────────

    def on_mapping_pose_update(self, x: float, y: float, yaw: float):
        self._mapping.update_pose(x, y, yaw)
        self._mapping.update_pose_on_map(x, y, yaw)

    def on_pose_update(self, x: float, y: float, yaw: float):
        self._last_x = x; self._last_y = y; self._last_yaw = yaw
        self._stations.set_current_pose(x, y, yaw)
        self._nav.update_pose_on_map(x, y, yaw)
        self._routes.update_pose(x, y, yaw)
        self._sidebar.set_ros_status(tr("status_amcl"), ok=True)

    def on_odom_update(self, x: float, y: float, yaw: float):
        self._odom_x = x; self._odom_y = y; self._odom_yaw = yaw
        self._nav.update_pose(x, y, yaw)

    def on_odom_twist_update(self, linear: float, angular: float):
        self._routes.update_velocity(linear, angular)
        self._diag.feed_odom_twist(linear, angular)
        self._dashboard.set_speed(linear, angular)

    def on_map_update(self, msg):
        self._mapping.update_map(msg)
        if not self._nav._selected_map_path:
            self._nav.update_map(msg)
        # Fleet phải dùng cùng metadata /map với AMCL và Nav2.
        if hasattr(self, "_fleet"):
            try:
                self._fleet.update_map_info(msg)
            except Exception as exc:
                print(
                    f"[MainWindow] Không chuyển được /map info sang Fleet: {exc}"
                )

    def on_scan_update(self, world_pts):
        self._mapping.update_scan(world_pts)
        self._nav.update_scan(world_pts)
        self._routes.update_scan(world_pts)

    def on_battery_update(self, pct: int):
        self._titlebar.set_battery(pct)
        self.health.battery_pct = int(pct)
        self._dashboard.set_battery(int(pct))
        self.alarms.set_condition("battery.low", pct < 15, "warn", tr("al_battery", pct), "Battery")

    def on_connection_update(self, state: str):
        self._titlebar.set_connection(state)
        self.health.ros_online = (state != "offline")
        self.alarms.set_condition("ros.offline", state == "offline", "error", tr("al_ros"), "ROS")
        self._home.update_connection(state)
        self._sidebar.set_ros_status(
            tr(f"status_{state}"), ok=(state != "offline"))

    def on_agv_status_update(self, moving: bool):
        self._home.update_agv_status(moving)

    def on_conveyor_cargo_update(self, belt_id: int, has_cargo: bool):
        self._home.update_conveyor_cargo(belt_id, has_cargo)
        self._dashboard.diagram.set_cargo(belt_id + 1, has_cargo)   # signal is 0-based

    def on_sensor_update(self, sensor_id: int, on: bool):
        self._conveyor.update_sensor(sensor_id, on)
        self._dashboard.diagram.set_sensor(sensor_id, on)

    def on_bumper_update(self, side: str, triggered: bool):
        self._routes.update_bumper(side, triggered)
        self._dashboard.diagram.set_bumper(side, triggered)
        self.alarms.set_condition(
            f"bumper.{side}", triggered, "error",
            tr("al_bumper", tr("side_left") if side == "left" else tr("side_right")), "Bumper")

    def on_plc_connection_update(self, state: str):
        self._home.update_plc_connection(state)
        online = (state == "online")
        self.health.set_plc(online)
        self.alarms.set_condition("plc.offline", not online, "error", tr("al_plc_offline"), "PLC")

    def on_estop_update(self, active: bool):
        self._home.update_estop(active)
        self.health.estop = bool(active)
        self.alarms.set_condition("estop", active, "critical", tr("al_estop"), "E-stop")
        self._estop_banner.setVisible(bool(active))
        if active and not self._estop_active:
            audit_log.audit("estop_pressed")
            self._abort_fork_wait(send_stop=True)
            if self._mission_running:
                self._stop_mission("cancelled")
                self._toasts.show_toast(tr("estop_mission_cancelled"), "error", 6000)
        self._estop_active = bool(active)

    def on_driver_status_update(self, payload: str):
        self._home.update_driver_status(payload)
        self.health.set_driver_payload(payload)
        d = self.health.driver
        online = bool(d.get("online"))
        try:
            faults = [int(x) for x in (d.get("fault") or [0, 0])]
        except (TypeError, ValueError):
            faults = [0, 0]
        self.alarms.set_condition("driver.offline", not online, "error", tr("al_driver_offline"), "KEYA")
        self.alarms.set_condition(
            "driver.fault", online and any(faults), "critical",
            tr("al_driver_fault", "/".join(f"{c:04X}" for c in faults)), "KEYA")

    def on_motor_enable_result(self, requested: bool, success: bool, message: str):
        self._home.motor_request_done()
        audit_log.audit("motor_enable" if requested else "motor_disable", "ok" if success else message)
        if success:
            if requested:
                self.alarms.clear_alarm("estop.soft")
            return
        detail = tr("home_motor_no_service") if message == "__no_service__" else message
        QMessageBox.warning(self, tr("home_motor"), tr("home_motor_failed", detail))

    def on_robot_status_update(self, msg: str):
        self._mapping.error_header.update_from_ros(msg)
        self._nav.error_header.update_from_ros(msg)
        self._routes.update_error(msg)

    def on_nav2_status_update(self, status_msg: str):
        level = "ok"; message = ""
        try:
            data = json.loads(status_msg or "{}")
            if isinstance(data, dict):
                level   = str(data.get("level",   "ok")).lower().strip()
                message = str(data.get("message", "")).strip()
        except Exception:
            message = str(status_msg or "").strip()
            level   = "error" if message else "ok"
        _ok = level in ("ok", "normal", "none", "0", "false")
        self.alarms.set_condition(
            "nav2", not _ok, "warn" if level in ("warn", "warning", "1") else "error",
            tr("al_nav2", message or level), "Nav2")
        for header in (self._nav.error_header, self._routes.error_header):
            if level in ("ok", "normal", "none", "0", "false"):
                header.clear_nav2()
            elif level in ("warn", "warning", "1"):
                header.set_nav2_warning(message or "Nav2 cảnh báo")
            else:
                header.set_nav2_error(message or "Nav2 lỗi")

    # ── 4.5.0: diagnostics, safety, alarms ───────────────────────────

    def on_topic_stats(self, rates: dict):
        self._diag.update_rates(rates)
        scan = float(rates.get("/scan", 0.0))
        if scan > 0.0:
            self._scan_seen = True
            self._scan_low_ticks = 0
        elif self._scan_seen:
            self._scan_low_ticks += 1
        self.alarms.set_condition("lidar.lost", self._scan_seen and self._scan_low_ticks >= 3,
                                  "error", tr("al_lidar"), "Lidar")

    def on_amcl_cov(self, sigma: float):
        self.health.set_amcl_cov(sigma)
        if sigma > 0.5:
            self.alarms.raise_alarm("loc.poor", "warn", tr("al_loc", sigma), "AMCL")
        elif sigma <= 0.35:
            self.alarms.clear_alarm("loc.poor")

    def on_rosout(self, level: int, node: str, text: str, stamp: float):
        self._diag.feed_rosout(level, node, text, stamp)

    def on_cmd_vel(self, lin: float, ang: float):
        self._diag.feed_cmd_vel(lin, ang)

    def on_joy(self, axes: list, buttons: list):
        """Gamepad teleop with a dead-man button (release = stop)."""
        if not prefs.get("joy_enabled"):
            return
        db = int(prefs.get("joy_deadman_button"))
        if 0 <= db < len(buttons) and buttons[db]:
            la, aa = int(prefs.get("joy_lin_axis")), int(prefs.get("joy_ang_axis"))
            lin = axes[la] * float(prefs.get("joy_max_lin")) if 0 <= la < len(axes) else 0.0
            ang = axes[aa] * float(prefs.get("joy_max_ang")) if 0 <= aa < len(axes) else 0.0
            self._joy_active = True
            self.velocity_signal.emit(lin, ang)
        elif self._joy_active:
            self._joy_active = False
            self.velocity_signal.emit(0.0, 0.0)

    def _on_soft_estop(self):
        """Soft stop: zero velocity, motors off, running mission cancelled."""
        audit_log.audit("soft_estop")
        self.soft_estop_signal.emit()
        self._abort_fork_wait(send_stop=True)
        if self._mission_running:
            self._stop_mission("cancelled")
        else:
            self.velocity_signal.emit(0.0, 0.0)
        self.alarms.raise_alarm("estop.soft", "error", tr("estop_soft_done"), "Soft stop")
        self._toasts.show_toast(tr("estop_soft_done"), "warn", 5000)

    def _preflight(self) -> bool:
        """True when the mission may start. Shows the checklist only if something fails."""
        if self._skip_preflight_once:
            self._skip_preflight_once = False
            return True
        if not prefs.get("preflight_enabled"):
            return True
        checks = self.health.checks()
        if all(c.ok or not c.blocking for c in checks):
            return True
        dlg = PreflightDialog(self.health, allow_override=(self._role == "engineer"), parent=self)
        ok = dlg.exec() == PreflightDialog.DialogCode.Accepted
        audit_log.audit("preflight", "accepted" if ok else "cancelled")
        return ok

    def _on_alarms_changed(self):
        self._titlebar.set_alarm_badge(self.alarms.unacked_count(), self.alarms.worst_active_level())
        self._dashboard.set_alarm_count(len(self.alarms.active()))
        self._subtabs.set_badge(Sidebar.IDX_ALARMS, len(self.alarms.active()))
        self._update_tower()

    def _on_alarm_raised(self, alarm: dict):
        if alarm.get("level") in ("error", "critical"):
            self._toasts.show_toast(f"{alarm['source']}: {alarm['message']}", alarm["level"], 6000)
            if prefs.get("sound_enabled"):
                QApplication.beep()

    def _update_tower(self):
        if not prefs.get("tower_enabled"):
            return
        worst = self.alarms.worst_active_level()
        if worst in ("error", "critical"):
            state = "error"
        elif self._mission_running and self._mission_paused:
            state = "paused"
        elif self._mission_running:
            state = "running"
        else:
            state = "idle"
        mask = int(prefs.get(f"tower_mask_{state}"))
        if mask != self._last_tower_mask:
            self._last_tower_mask = mask
            self.light_mask_signal.emit(mask)

    # ── Robot profile + safety monitor ──────────────────────────────

    def _apply_robot_profile(self, p: dict):
        """Use the configured size on every map and show the zones around the robot."""
        for mw in self.findChildren(MapWidget):
            mw.set_robot_size(p["length_m"], p["width_m"], p["center_x_m"])
            mw.set_safety_overlay(p if p.get("safety_enabled") else None)

    def _on_profile_applied(self, p: dict):
        audit_log.audit("robot_profile", f"{p['name']} {p['length_m']}x{p['width_m']}")
        self._apply_robot_profile(p)
        self._forklift.set_profile(p)
        self.safety_config_signal.emit(_json.dumps(p))
        self._toasts.show_toast("🛡 " + p["name"], "ok")

    def publish_saved_safety_config(self):
        """Send the saved profile to ROS (the topic is latched, so a late node still gets it)."""
        self.safety_config_signal.emit(_json.dumps(self._safety.active_profile()))

    def _on_override_changed(self, on: bool):
        if on and self._role != "engineer":          # defensive: only Engineer may do this
            OVERRIDE.set(False)
            return
        audit_log.audit("safety_override", "ON (teleop, zones off)" if on else "OFF")
        if on:
            self._override_timer.start()
            self.safety_override_signal.emit(True)
            self._toasts.show_toast(tr("joy_safety_toast_on"), "warn", 4000)
        else:
            self._override_timer.stop()
            self.safety_override_signal.emit(False)
            self._toasts.show_toast(tr("joy_safety_toast_off"), "ok", 2500)

    def _refresh_override(self):
        """Keep the override alive while it is on. A running mission always puts the zones back."""
        if not OVERRIDE.is_on():
            self._override_timer.stop()
            return
        if self._mission_running:
            OVERRIDE.set(False)
            return
        self.safety_override_signal.emit(True)

    def on_safety_points(self, payload: str):
        """Points as the safety_monitor sees them (base frame): the Safety page draws exactly these."""
        if self._pages.currentIndex() == Sidebar.IDX_SAFETY:
            self._safety.feed_points(payload)

    def on_safety_state(self, payload: str):
        try:
            d = _json.loads(payload)
        except ValueError:
            return
        state = str(d.get("state", ""))
        self.health.set_safety(state)
        self._safety.feed_state(payload)
        self._dashboard.set_safety(state)
        self._titlebar.set_safety_chip(state)
        self._forklift.feed_bypass(d.get("bypass", []))
        self.alarms.set_condition("safety.stop", state == "stop", "warn", tr("al_safety_stop"), "Safety")
        self.alarms.set_condition("safety.noscan", state == "no_scan", "error", tr("al_safety_noscan"), "Safety")
        self.alarms.set_condition("safety.override", state == "override", "warn", tr("al_safety_override"), "Safety")

    def on_scan_raw(self, angle_min: float, inc: float, rmin: float, rmax: float, ranges: list):
        if self._pages.currentIndex() == Sidebar.IDX_SAFETY:        # only while the page is open
            self._safety.feed_scan(angle_min, inc, rmin, rmax, ranges)

    def _mission_summary(self):
        """(kind, text) of the mission for the title-bar chip: idle / run / wait / pause / stuck."""
        if not self._mission_running:
            return "idle", ""
        total = len(self._mission_wps)
        pos = self._mission_idx
        n = f"{min(pos + 1, total)}/{total}" if total else ""
        if self._mission_stuck:
            return "stuck", tr("chip_mission_stuck", n)
        if self._confirm_continue is not None:
            return "wait", tr("chip_mission_wait", n)
        if self._mission_paused:
            return "pause", tr("chip_mission_pause", n)
        return "run", tr("chip_mission_run", n)

    def _update_status_chips(self):
        self._titlebar.set_mission_chip(*self._mission_summary())
        ready = None
        if self.nav_client is not None:
            try:
                ready = bool(self.nav_client.is_ready(0.0))
            except Exception:
                ready = None
        self._titlebar.set_nav_chip(ready)
        if self.health_safety_stale():
            self._titlebar.set_safety_chip(None)

    def health_safety_stale(self) -> bool:
        """True when /safety_state has not been received for a while (the chip must not keep showing 'ok')."""
        try:
            return not self.health.safety_fresh()
        except Exception:
            return False

    def _update_mission_bar(self):
        """Show the progress of a running mission when Navigation / Routes is not the open page."""
        self._update_status_chips()
        idx = self._current_page_idx
        away = bool(self._mission_running) and idx not in (Sidebar.IDX_NAV, Sidebar.IDX_ROUTES)
        if not away:
            if self._mission_bar.isVisible():
                self._mission_bar.setVisible(False)
            self._mission_bar_state = ""
            return
        total = len(self._mission_wps)
        pos = self._mission_idx
        label = self._mission_wps[pos].get("label", "?") if 0 <= pos < total else "?"
        waiting = self._confirm_continue is not None
        if self._mission_stuck:
            key, color = "mb_stuck", "#F85149"
        elif waiting:
            key, color = "mb_confirm", "#E3B341"
        elif self._mission_paused:
            key, color = "mb_paused", "#E3B341"
        else:
            key, color = "mb_running", "#3FB950"
        self._mission_bar_lbl.setText(tr(key, min(pos + 1, total) if total else 0, total, label))
        self._mission_bar_ok.setText(tr("mb_ok")); self._mission_bar_ok.setVisible(waiting and not self._mission_stuck)
        self._mission_bar_view.setText(tr("mb_view")); self._mission_bar_stop.setText(tr("mb_stop"))
        sig = f"{key}|{color}"
        if sig != self._mission_bar_state:
            self._mission_bar_state = sig
            self._mission_bar.setStyleSheet(
                f"QWidget#MissionBar{{background:{color}26;border-bottom:2px solid {color};}}"
                f"QLabel{{color:{color};}}")
        self._mission_bar.setVisible(True)

    def _mission_bar_goto(self):
        self._sidebar.switch(Sidebar.IDX_ROUTES if self._mission_source == "routes" else Sidebar.IDX_NAV)

    def _mission_bar_stop_clicked(self):
        ans = QMessageBox.question(self, tr("mb_stop"), tr("mb_stop_confirm"),
                                   QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                   QMessageBox.StandardButton.No)
        if ans == QMessageBox.StandardButton.Yes:
            self._stop_mission()
            self._update_mission_bar()

    def _notify_away(self, text: str, kind: str = "warn", ms: int = 8000):
        """Toast for a mission event the operator cannot see because another page is open."""
        if self._current_page_idx not in (Sidebar.IDX_NAV, Sidebar.IDX_ROUTES):
            self._toasts.show_toast(text, kind, ms)

    def _housekeeping(self):
        self._update_mission_bar()
        self._update_tower()
        if self.health.safety_ever and not self.health.safety_fresh():
            self._safety.mark_state_stale()
            self._dashboard.set_safety(None)
        self._housekeeping_ticks += 1
        self._check_fork_watchdog()
        self._forklift.set_node_alive(_time.monotonic() - self._fork_seen_status < 3.0 if self._fork_seen_status else False)
        if self._housekeeping_ticks % 15 == 0:
            self._schedule_tick()

    # ── Fork lift tasks ─────────────────────────────────────────────

    def _robot_type_ok(self, wps: list) -> bool:
        """Fork tasks need a fork-lift robot profile (and the other way round is a warning only)."""
        has_fork = any(str(t.get("type", "")).startswith("fork_")
                       for wp in wps for t in wp.get("tasks", []))
        prof_type = self._safety.active_profile().get("robot_type", "agv")
        if has_fork and prof_type != "forklift":
            self._toasts.show_toast(tr("rt_mismatch", tr("rt_forklift"), tr("rt_" + prof_type)), "error", 7000)
            return False
        return True

    def _start_fork_task(self, task: dict, tasks: list, next_idx: int, on_done, token):
        cmd = {k: v for k, v in task.items() if k not in ("type", "order")}
        cmd["cmd"] = str(task["type"])[5:]                       # fork_pick -> pick
        cmd["id"] = _uuid.uuid4().hex[:10]
        self._fork_wait = {"id": cmd["id"], "tasks": tasks, "next": next_idx, "on_done": on_done,
                           "token": token, "t0": _time.monotonic(), "seen": False,
                           "label": tr("task_" + str(task["type"]))}
        audit_log.audit("fork_task", f"{cmd['cmd']} {json.dumps({k: v for k, v in cmd.items() if k not in ('id', 'cmd')})}")
        self._routes.set_task_status(tr("fk_waiting", self._fork_wait["label"]))
        self.forklift_cmd_signal.emit(_json.dumps(cmd))

    def _abort_fork_wait(self, send_stop: bool = False):
        had = self._fork_wait is not None
        self._fork_wait = None
        if send_stop and (had or self._forklift.is_running()):
            self.forklift_cmd_signal.emit(_json.dumps({"cmd": "stop"}))

    def _fail_fork_task(self, message: str):
        self._abort_fork_wait(send_stop=True)
        self._toasts.show_toast(tr("fk_failed", message), "error", 8000)
        self.alarms.raise_alarm("forklift.error", "error", tr("al_forklift", message), "Fork lift")
        audit_log.audit("fork_task_failed", message)
        if self._mission_running:
            self._stop_mission("failed")

    def on_forklift_status(self, payload: str):
        try:
            st = _json.loads(payload)
        except ValueError:
            return
        self._fork_seen_status = _time.monotonic()
        self._forklift.feed_status(st)
        state = st.get("state")
        if state == "error":
            self.alarms.raise_alarm("forklift.error", "error", tr("al_forklift", st.get("message", "")), "Fork lift")
        else:
            self.alarms.clear_alarm("forklift.error")
        w = self._fork_wait
        if not w or st.get("id") != w["id"]:
            return
        w["seen"] = True
        if state == "done":
            self._fork_wait = None
            if self._mission_active(w["token"]):
                self._run_sequential(w["tasks"], w["next"], w["on_done"], token=w["token"])
        elif state == "error":
            self._fail_fork_task(str(st.get("message", "")))
        else:
            self._routes.set_task_status(tr("fk_waiting", f"{w['label']} - {st.get('step', '')} {int(100 * float(st.get('progress', 0)))}%"))

    def _check_fork_watchdog(self):
        w = self._fork_wait
        if not w:
            return
        age = _time.monotonic() - w["t0"]
        if not w["seen"] and age > 5.0:
            self._fail_fork_task(tr("fk_no_node"))
        elif age > 300.0:
            self._fail_fork_task("timeout")

    # ── Stations, schedule, queue, touch mode ───────────────────────

    def _goto_station(self, st: dict):
        if self._mission_running:
            self._toasts.show_toast(tr("st_busy"), "warn")
            return
        audit_log.audit("goto_station", str(st.get("name", "")))
        self._toasts.show_toast("📍 " + str(st.get("name", "")), "info")
        self._run_navigation_mission([{"label": st["name"], "x": st["x"], "y": st["y"], "tasks": []}])

    def _schedule_tick(self):
        if not self._schedule.auto_enabled():
            return
        for job in self._schedule.scheduler.due():
            route = RM.get_route(job["route_id"])
            if route:
                audit_log.audit("schedule_due", job.get("route_name", ""))
                self._schedule.enqueue(route)
        self._try_run_queue()

    def _try_run_queue(self, manual: bool = False):
        """Run the next queued route. Automatic runs never bypass the safety checks."""
        if self._mission_running or not self._schedule.queue:
            if not self._schedule.queue:
                self._queue_run_active = False
            return
        if manual:
            self._queue_run_active = True
        elif not (self._queue_run_active or self._schedule.auto_enabled()):
            return
        if not manual:
            blocked = self._estop_active or any(
                (not c.ok) and c.blocking for c in self.health.checks())
            if blocked:
                self.alarms.raise_alarm("schedule.blocked", "warn", tr("sc_blocked"), "Schedule")
                return
            self.alarms.clear_alarm("schedule.blocked")
            self._skip_preflight_once = True      # checks were just verified above
        route = self._schedule.pop_next()
        if route is None:
            return
        audit_log.audit("queue_run", str(route.get("name", "")))
        self._toasts.show_toast(tr("sc_started", route.get("name", "")), "info", 5000)
        self._run_route_from_list(route)

    def apply_touch_mode(self, on: bool):
        qss = ("QPushButton{min-height:44px;font-size:15px;} QComboBox{min-height:40px;font-size:15px;}"
               "QLineEdit,QSpinBox,QDoubleSpinBox,QTimeEdit{min-height:38px;font-size:15px;}"
               "QCheckBox{font-size:15px;spacing:10px;} QTableWidget{font-size:14px;}") if on else ""
        self.setStyleSheet(qss)
        self._sidebar.set_touch(on)
        if on:
            self.showFullScreen()
        elif self.isFullScreen():
            self.showNormal()

    # ── Conveyor ─────────────────────────────────────────────────────

    def set_fleet_map_context_callback(self, cb):
        """Set ROS publisher callback for /fleet/map_context."""
        self._fleet_map_context_cb = cb

    def _ensure_route_map_record(self, route: dict) -> dict:
        """Register legacy/new route map and return its canonical Fleet record."""
        mp = self._normalize_map_path(route.get("map_path", ""))
        if not mp or not os.path.exists(mp):
            raise FileNotFoundError(
                f"Route thiếu map .yaml hợp lệ: {route.get('map_path', '(trống)')}"
            )

        record = self._fleet_map_registry.ensure_registered(
            mp,
            name=str(route.get("name") or os.path.splitext(os.path.basename(mp))[0]),
            preferred_map_id=str(route.get("map_id", "") or ""),
        )
        route["map_id"] = record["map_id"]
        route["map_version"] = int(record.get("map_version", 1))
        route["map_checksum"] = str(record.get("checksum", ""))
        # Canonical Fleet YAML is also used by Nav2, guaranteeing that RViz,
        # Routes and Fleet Monitor read the same origin/resolution/image pair.
        route["map_path"] = str(record["active_yaml"])

        route_id = str(route.get("id", "") or "")
        if route_id:
            try:
                RM.attach_map_context(
                    route_id,
                    map_id=route["map_id"],
                    map_version=route["map_version"],
                    map_path=route["map_path"],
                    map_checksum=route["map_checksum"],
                )
            except Exception as exc:
                print(f"[Fleet auto map] Không migrate được route {route_id}: {exc}")
        return record

    def _publish_fleet_route_context(self, route: dict, record: dict) -> bool:
        cb = self._fleet_map_context_cb
        if cb is None:
            print("[Fleet auto map] Chưa gắn callback /fleet/map_context")
            return False
        payload = {
            "map_id": str(record["map_id"]),
            "map_version": int(record.get("map_version", 1)),
            "map_checksum": str(record.get("checksum", "")),
            "map_path": str(record["active_yaml"]),
            "route_id": str(route.get("id", "")),
            "route_name": str(route.get("name", "")),
            "waypoint_current": 0,
            "waypoint_total": len(route.get("waypoints", []) or []),
            "frame_id": "map",
        }
        try:
            return bool(cb(payload))
        except Exception as exc:
            print(f"[Fleet auto map] Publish map context lỗi: {exc}")
            return False

    def set_conveyor_cmd_callback(self, cb):
        self._conveyor_cmd_cb = cb

    def _on_conveyor_cmd(self, payload: dict):
        if self._conveyor_cmd_cb:
            self._conveyor_cmd_cb(payload)

    def _on_connection_toggle(self):
        self._sidebar.switch(Sidebar.IDX_NAV)

    # ── [NEW] Đổi người vận hành ─────────────────────────────────────

    def _on_switch_user_clicked(self):
        audit_log.audit("switch_user")
        """
        Nút "Đổi người vận hành" trên Home. Nếu mission đang chạy, hỏi
        xác nhận trước vì đổi user sẽ dừng mission (tránh robot chạy vô
        chủ khi HMI đã đóng để đăng nhập lại). ROS node vẫn giữ nguyên,
        chỉ MainWindow bị đóng và LoginDialog mở lại — xem agv_hmi/main.py.
        """
        if self._mission_running:
            reply = QMessageBox.question(
                self, tr("home_switch_user"),
                "Đang chạy lộ trình. Dừng lộ trình và đổi người vận hành?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if reply != QMessageBox.StandardButton.Yes:
                return
            self._stop_mission()
        else:
            reply = QMessageBox.question(
                self, tr("home_switch_user"),
                "Quay về màn hình đăng nhập để đổi người vận hành?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if reply != QMessageBox.StandardButton.Yes:
                return
        # [FIX] Đánh dấu đây là đóng cửa sổ để đổi người vận hành —
        # closeEvent() sẽ dựa vào cờ này để KHÔNG gọi app.quit().
        self._switching_user = True
        self.switch_user_signal.emit()

    # ── Map save ──────────────────────────────────────────────────────

    def _do_save_map(self, path: str):
        audit_log.audit("map_save", path)
        if not self.map_saver:
            return
        ok = self.map_saver.save(path)
        record = None
        if ok:
            yaml_path = path if str(path).lower().endswith(".yaml") else path + ".yaml"
            MapLibraryPage.save_thumbnail(yaml_path, self._mapping.map_widget)
            try:
                # Every newly created Mapping result receives a new random Mxxxxx ID.
                record = self._fleet_map_registry.register_new_map(
                    yaml_path,
                    name=os.path.splitext(os.path.basename(yaml_path))[0],
                )
            except Exception as exc:
                print(f"[Fleet auto map] Đăng ký map mới thất bại: {exc}")

        if ok and record:
            self._sidebar.set_ros_status(
                f"{tr('map_saved')}: {record['map_id']} v{record.get('map_version', 1)}",
                True,
            )
        else:
            self._sidebar.set_ros_status(
                tr("map_saved") if ok else tr("map_save_fail"),
                ok,
            )
        self._maplib.refresh()

    def _load_map_to_nav(self, path: str):
        ok = self._nav.load_map_path(path) if hasattr(self._nav, "load_map_path") else False
        if ok:
            self._sidebar.switch(Sidebar.IDX_NAV)
        else:
            self._sidebar.set_ros_status("Không load được map đã chọn", ok=False)

    # ── Normalize map path ────────────────────────────────────────────

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

    # ── Route save / edit ─────────────────────────────────────────────

    def _save_route(self, name: str, map_path: str,
                    wps: list, route_id: str = ""):
        mp = self._normalize_map_path(map_path)
        if not mp or not os.path.exists(mp):
            self._sidebar.set_ros_status(
                "Không lưu: route thiếu map hợp lệ", ok=False)
            return

        try:
            record = self._fleet_map_registry.ensure_registered(
                mp,
                name=os.path.splitext(os.path.basename(mp))[0],
            )
        except Exception as exc:
            self._sidebar.set_ros_status(
                f"Không đăng ký được map Fleet: {exc}", ok=False)
            return

        canonical_mp = str(record["active_yaml"])
        map_id = str(record["map_id"])
        map_version = int(record.get("map_version", 1))
        map_checksum = str(record.get("checksum", ""))

        if route_id:
            ok = RM.update_route(
                route_id,
                name,
                canonical_mp,
                wps,
                map_id=map_id,
                map_version=map_version,
                map_checksum=map_checksum,
            )
            route = RM.get_route(route_id) or {
                "id": route_id,
                "name": name,
                "map_path": canonical_mp,
                "map_id": map_id,
                "map_version": map_version,
                "map_checksum": map_checksum,
                "waypoints": wps,
            }
            if not ok:
                self._sidebar.set_ros_status(
                    f"Không cập nhật được: {name}", ok=False)
                return
        else:
            route = RM.create_route(
                name,
                canonical_mp,
                wps,
                map_id=map_id,
                map_version=map_version,
                map_checksum=map_checksum,
            )

        try:
            RM.set_route_field(route["id"], "robot_type", self._nav.robot_type())
        except Exception as exc:
            print(f"[Route] robot_type not saved: {exc}")

        try:
            thumb_dir = os.path.expanduser("~/.agv_hmi/route_thumbnails")
            os.makedirs(thumb_dir, exist_ok=True)
            thumb_path = os.path.join(thumb_dir, f"{route['id']}.png")
            pix = self._nav.map_widget.grab()
            if not pix.isNull():
                pix = pix.scaled(
                    240,
                    160,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
                pix.save(thumb_path, "PNG")
                routes = RM.load_all()
                for saved_route in routes:
                    if saved_route.get("id") == route.get("id"):
                        saved_route["thumbnail_path"] = thumb_path
                        break
                RM.save_all(routes)
        except Exception as exc:
            print(f"[Route thumbnail] save failed: {exc}")

        action = "Đã cập nhật" if route_id else "Đã lưu"
        self._sidebar.set_ros_status(
            f"{action}: {name} | Map {map_id} v{map_version}", ok=True)
        self._routes.refresh()

    def _edit_route(self, route: dict):
        if self._role != "engineer":
            self._sidebar.set_ros_status(
                "Chỉ Engineer được chỉnh sửa route", ok=False)
            return
        try:
            self._nav.load_route_for_edit(route)
            self._sidebar.switch(Sidebar.IDX_NAV)
        except Exception as e:
            self._sidebar.set_ros_status(
                f"Không mở được route để sửa: {e}", ok=False)

    # ── Route run from list ───────────────────────────────────────────

    def _run_route_from_list(self, route: dict):
        """Run a saved route and automatically activate its Fleet map."""
        if self._mission_running:                  # a mission already runs (maybe in the background)
            self._toasts.show_toast(tr("st_busy"), "warn")
            return
        self._mission_source = "routes"
        if not self._preflight():
            try:
                self._routes.set_mission_done()
            except Exception:
                pass
            return
        route = copy.deepcopy(route or {})

        try:
            record = self._ensure_route_map_record(route)
        except Exception as exc:
            self._routes.set_status(str(exc))
            self._sidebar.set_ros_status("Route thiếu map hợp lệ", ok=False)
            try:
                self._routes.set_mission_done()
            except Exception:
                pass
            return

        mp = self._normalize_map_path(route.get("map_path", ""))
        wps = copy.deepcopy(route.get("waypoints", []))
        audit_log.audit("mission_start", str(route.get("name", "")))
        if not self._robot_type_ok(wps):
            try:
                self._routes.set_mission_done()
            except Exception:
                pass
            return
        if not wps:
            self._routes.set_status("Route không có waypoint.")
            try:
                self._routes.set_mission_done()
            except Exception:
                pass
            return

        token = self._new_mission_token()
        self._cancel_mission_timers()
        self._clear_route_confirm()
        self._cancel_reason = None
        self._stop_nav_retry()

        # Nav2 receives the canonical active.yaml also used by Fleet Monitor.
        ok, msg = self._ensure_nav_running(mp)
        try:
            self._nav.set_nav_launch_running(ok, msg)
        except Exception:
            pass
        self._sidebar.set_ros_status(msg, ok=ok)
        if not ok:
            self._routes.set_status(msg)
            try:
                self._routes.set_mission_done()
            except Exception:
                pass
            return

        published = self._publish_fleet_route_context(route, record)
        if published:
            self._routes.set_status(
                f"Đã kích hoạt Fleet map {record['map_id']} "
                f"v{record.get('map_version', 1)}. Đang chờ Nav2..."
            )
        else:
            self._routes.set_status(
                "Map đã đăng ký nhưng chưa publish được Fleet context. "
                "Đang chờ Nav2..."
            )
        self._start_nav_retry(wps, token, max_attempts=60)

    # ── Single-timer retry ────────────────────────────────────────────

    def _start_nav_retry(self, wps: list[dict],
                         token: int, max_attempts: int = 60):
        self._nav_retry_wps      = wps
        self._nav_retry_token    = token
        # [FIX-2] Lưu lại tổng attempt để tính đúng số giây đã trôi qua.
        self._nav_retry_total    = max_attempts
        self._nav_retry_attempts = max_attempts
        if self._nav_retry_timer is None:
            self._nav_retry_timer = QTimer(self)
            self._nav_retry_timer.setSingleShot(False)
            self._nav_retry_timer.timeout.connect(self._on_nav_retry_tick)
        self._nav_retry_timer.start(500)

    def _stop_nav_retry(self):
        if self._nav_retry_timer is not None:
            self._nav_retry_timer.stop()
        self._nav_retry_attempts = 0

    def _on_nav_retry_tick(self):
        token = self._nav_retry_token
        if token != self._mission_token:
            self._stop_nav_retry(); return
        if not self.nav_client:
            self._stop_nav_retry()
            self._routes.set_status("NavClient chưa được khởi tạo.")
            try: self._routes.set_mission_done()
            except Exception: pass
            return
        if self.nav_client.is_ready(timeout_sec=0.05):
            self._stop_nav_retry()
            self._start_mission(self._nav_retry_wps, token=token)
            return
        self._nav_retry_attempts -= 1

        # [FIX-2] Tính đúng số giây theo interval timer thực tế (500ms/tick)
        # thay vì lấy trực tiếp số tick làm số giây (bug cũ: "1/30s"...
        # "60/30s" — sai đơn vị hoàn toàn).
        total_sec   = int(self._nav_retry_total * 0.5)
        elapsed_sec = int((self._nav_retry_total - self._nav_retry_attempts) * 0.5)
        self._routes.set_status(
            f"Đang chờ Nav2 sẵn sàng... ({elapsed_sec}/{total_sec}s)")

        if self._nav_retry_attempts <= 0:
            self._stop_nav_retry()
            # [FIX-2] Phân biệt rõ "không bắt đầu được mission" (Nav2
            # chưa ready, mission chưa hề start) với "mission failed"
            # (mission đã chạy nhưng thất bại giữa chừng) — 2 trạng thái
            # khác nhau, tránh gây hiểu nhầm khi xem log/status.
            self._routes.set_status(
                f"Không bắt đầu được mission: Nav2 action server "
                f"chưa sẵn sàng sau {total_sec}s.\n"
                "Kiểm tra navigation.launch.py.")
            self._sidebar.set_ros_status("Nav2 chưa sẵn sàng", ok=False)
            try: self._routes.set_mission_done()
            except Exception: pass

    # ── Mission control ───────────────────────────────────────────────

    def _new_mission_token(self) -> int:
        self._mission_token += 1
        return self._mission_token

    def _cancel_mission_timers(self):
        for timer in list(self._mission_timers):
            try: timer.stop(); timer.deleteLater()
            except Exception: pass
        self._mission_timers.clear()

    def _mission_active(self, token: int | None = None) -> bool:
        if token is not None and token != self._mission_token:
            return False
        return self._mission_running and not self._mission_paused

    def _set_pause_controls(self, can_pause: bool, paused: bool = False):
        """Đồng bộ Pause/Resume cho đúng page đang sở hữu mission."""
        nav_owner = self._mission_source == "navigation"
        route_owner = self._mission_source == "routes"

        try:
            self._nav._pause_btn.setEnabled(
                bool(nav_owner and can_pause and not paused))
            self._nav._resume_btn.setEnabled(bool(nav_owner and paused))
        except Exception:
            pass

        try:
            manual = route_owner and self._manual_mode
            self._routes._pause_btn.setEnabled(
                bool(route_owner and can_pause and not paused and not manual))
            self._routes._resume_btn.setEnabled(
                bool(route_owner and paused and not manual))
        except Exception:
            pass

    def _schedule_mission_timer(self, delay_ms: int,
                                callback, token: int | None = None):
        timer = QTimer(self); timer.setSingleShot(True)
        def _wrapped():
            try:
                if timer in self._mission_timers:
                    self._mission_timers.remove(timer)
                timer.deleteLater()
            except Exception: pass
            if not self._mission_active(token): return
            callback()
        timer.timeout.connect(_wrapped)
        self._mission_timers.append(timer)
        timer.start(max(0, int(delay_ms)))

    def _clear_route_confirm(self):
        try:
            if hasattr(self._routes, "clear_confirm"):
                self._routes.clear_confirm()
        except Exception: pass

    def _request_route_confirm(self):
        try:
            if hasattr(self._routes, "request_confirm"):
                self._routes.request_confirm(tr("nav_confirm"))
        except Exception: pass

    def _start_mission(self, waypoints: list[dict],
                       token: int | None = None):
        if not waypoints or not self.nav_client:
            return
        OVERRIDE.set(False)          # an autonomous mission never runs with the safety zones off
        if token is None:
            token = self._new_mission_token()
        else:
            self._mission_token = token

        self._cancel_mission_timers()
        self._clear_route_confirm()
        self._confirm_continue = None
        self._mission_phase = "idle"

        # Reset trạng thái stuck/manual của lần chạy trước.
        self._mission_stuck = False
        self._stuck_wp_idx = -1
        self._manual_pending_action = None
        try:
            self._routes.clear_stuck()
        except Exception:
            pass

        if self._mission_running:
            self._cancel_reason = "restart"
            self.nav_client.cancel_goal()

        self._mission_wps = copy.deepcopy(waypoints)
        self._mission_idx = 0
        self._mission_running = True
        self._mission_paused = False
        self._cancel_reason = None

        self._nav.set_mission_running(True)
        if self._mission_source == "routes":
            for i in range(len(waypoints)):
                self._routes.set_wp_status(i, WP_PENDING)

            # Chỉ Routes mission được phép đóng/mở route log.
            try:
                if getattr(self._routes, "_active_record", None) is not None:
                    self._routes._stop_logging("cancelled")
            except Exception:
                pass

            if getattr(self._routes, "_current_route", None) is not None:
                try:
                    self._routes._start_logging()
                except Exception as e:
                    print(f"[Mission] _start_logging failed: {e}")

        self._run_next(token=token)

    def _run_next(self, token: int | None = None):
        if not self._mission_active(token):
            return
        if self._mission_idx >= len(self._mission_wps):
            self._on_mission_done()
            return

        self._mission_phase = "moving"
        self._set_pause_controls(can_pause=True, paused=False)

        wp = self._mission_wps[self._mission_idx]
        idx = self._mission_idx
        msg = tr("nav_going", wp["label"], wp["x"], wp["y"])
        self._nav.set_status(msg)
        if self._mission_source == "routes":
            self._routes.set_status(msg)
            self._routes.set_wp_status(idx, WP_MOVING)

        t = self._mission_token
        self.nav_client.send_goal(
            wp["x"], wp["y"],
            on_result=lambda ok, t=t: self._nav_result_signal.emit(ok, t),
            on_feedback=lambda d, t=t: self._nav_feedback_signal.emit(
                idx + 1, d, t),
        )

    def _on_feedback(self, n: int, dist: float, token: int = None):
        if token is not None and token != self._mission_token:
            return
        if not self._mission_running:
            return
        msg = tr("nav_remaining", n, dist)
        self._nav.set_status(msg)
        if self._mission_source == "routes":
            self._routes.set_status(msg)

    def _on_wp_result(self, success: bool, token: int = None):
        if token is not None and token != self._mission_token:
            return
        if not self._mission_running:
            return

        if not success:
            if self._mission_paused or self._cancel_reason in (
                    "pause", "stop", "restart"):
                return

            self._set_pause_controls(can_pause=False, paused=False)

            # Navigation test mission không được ghi log hoặc điều khiển
            # repeat state của RoutesPage.
            if self._mission_source != "routes":
                self._mission_running = False
                self._mission_phase = "idle"
                self._nav.set_status(tr("nav_failed"))
                self._nav.set_mission_running(False)
                self._cancel_mission_timers()
                self._clear_route_confirm()
                return

            # Routes mission: giữ tiến độ và chuyển sang STUCK.
            idx = self._mission_idx
            self._mission_phase = "stuck"
            self._mission_stuck = True
            self._stuck_wp_idx = idx
            self._routes.set_wp_status(idx, WP_STUCK)
            label = (self._mission_wps[idx].get("label", "?")
                     if 0 <= idx < len(self._mission_wps) else "?")
            msg = f"⚠ Gặp sự cố tại WP {label} — xem lựa chọn bên dưới"
            self._nav.set_status(msg)
            self._routes.on_mission_stuck(label)
            self._notify_away(tr("mb_stuck_toast", label), "error", 10000)
            return

        idx = self._mission_idx
        self._mission_phase = "task"
        self._set_pause_controls(can_pause=False, paused=False)
        if self._mission_source == "routes":
            self._routes.set_wp_status(idx, WP_TASK)
        tasks = self._mission_wps[idx].get("tasks", [])

        if self._mission_source == "routes" and self._manual_mode:
            self._mission_phase = "manual_wait"
            label = self._mission_wps[idx].get("label", "?")
            self._manual_pending_action = ("task", idx)
            if tasks:
                self._routes.manual_prompt(tr("manual_step_task", label))
            else:
                self._routes.manual_prompt(tr("manual_step_confirm", label))
            return

        if tasks:
            self._execute_tasks(
                tasks, on_done=self._advance, token=self._mission_token)
        else:
            self._advance(token=self._mission_token)

    def _execute_tasks(self, tasks: list[dict], on_done,
                       token: int | None = None):
        if not self._mission_active(token):
            return
        self._mission_phase = "task"
        self._set_pause_controls(can_pause=False, paused=False)
        if not tasks:
            on_done(token=token)
            return
        sequential = [
            t for t in tasks
            if t.get("order") != "parallel" or t.get("type") == "confirm"
            or str(t.get("type", "")).startswith("fork_")
        ]
        parallel = [
            t for t in tasks
            if t.get("order") == "parallel" and t.get("type") != "confirm"
            and not str(t.get("type", "")).startswith("fork_")
        ]
        delays = []
        for task in parallel:
            d = self._execute_single_task(task, token=token)
            if isinstance(d, int):
                delays.append(d)
        wait_ms = max(delays, default=0)

        def _after():
            self._run_sequential(sequential, 0, on_done, token=token)

        if wait_ms > 0:
            self._schedule_mission_timer(wait_ms, _after, token=token)
        else:
            _after()

    def _run_sequential(self, tasks: list[dict], idx: int,
                        on_done, token: int | None = None):
        if not self._mission_active(token): return
        if idx >= len(tasks): on_done(token=token); return
        task = tasks[idx]
        if task.get("type", "") == "confirm":
            self._confirm_continue = (tasks, idx + 1, on_done, token)
            self._routes.set_task_status("⏸ Chờ xác nhận")
            self._nav.set_status(tr("nav_confirm"))
            self._routes.set_status(
                "⏸ Chờ người vận hành xác nhận để tiếp tục.")
            self._notify_away(tr("mb_confirm_toast"), "warn", 10000)
            self._request_route_confirm(); return
        if str(task.get("type", "")).startswith("fork_"):
            self._start_fork_task(task, tasks, idx + 1, on_done, token)
            return
        delay = self._execute_single_task(task, token=token)
        if delay is None: return
        self._schedule_mission_timer(
            max(0, delay),
            lambda: self._run_sequential(tasks, idx + 1, on_done, token=token),
            token=token)

    def _confirm_task_continue(self):
        cont = self._confirm_continue
        self._confirm_continue = None; self._clear_route_confirm()
        if not cont: return
        tasks, idx, on_done, token = cont
        if not self._mission_active(token): return
        self._run_sequential(tasks, idx, on_done, token=token)

    def _execute_single_task(self, task: dict,
                             token: int | None = None) -> int | None:
        if not self._mission_active(token): return 0
        t = task.get("type", "")
        self._routes.set_task_status(f"▶ {t}")
        if t == "conveyor":
            ids  = task.get("conveyor_ids", [1])
            mode = task.get("conveyor_mode", "receive")
            for cid in ids:
                payload = {
                    "conveyor_id": cid, "mode": mode,
                    "speed": task.get("speed", 75),
                    "duration": task.get("duration", 5.0),
                }
                if self._conveyor_cmd_cb:
                    self._conveyor_cmd_cb(payload)
            try: self._routes.notify_cargo_delivered(len(ids))
            except AttributeError: pass
            return int(float(task.get("duration", 5.0)) * 1000)
        if t == "wait":
            delay = int(task.get("delay", 3))
            self._routes.set_status(tr("nav_stopping", delay))
            return delay * 1000
        if t == "buzzer":
            if self._conveyor_cmd_cb:
                self._conveyor_cmd_cb({"type": "buzzer"})
            return 500
        if t == "io":
            if self._conveyor_cmd_cb:
                self._conveyor_cmd_cb({"type": "io"})
            return 200
        return 0

    def _advance(self, token: int | None = None):
        if not self._mission_active(token):
            return
        idx = self._mission_idx
        if not (0 <= idx < len(self._mission_wps)):
            return
        if self._mission_source == "routes":
            self._routes.set_wp_status(idx, WP_DONE)
            self._routes.set_task_status("—")
        self._clear_route_confirm()
        self._mission_idx += 1
        self._run_next(token=token)

    # ── [E] Manual mode: xử lý task xong → chờ bước "di chuyển kế" ──────

    def _manual_after_task_done(self, token: int | None = None):
        """Dừng chờ operator sau khi task của waypoint hiện tại hoàn tất."""
        if token is not None and token != self._mission_token:
            return
        if not self._mission_running:
            return

        self._mission_phase = "manual_wait"
        self._set_pause_controls(can_pause=False, paused=False)
        idx = self._mission_idx
        self._routes.set_task_status("—")
        self._clear_route_confirm()

        self._manual_pending_action = ("move", idx)
        next_idx = idx + 1
        if next_idx < len(self._mission_wps):
            next_label = self._mission_wps[next_idx].get("label", "?")
            self._routes.manual_prompt(tr("manual_step_move", next_label))
        else:
            self._routes.manual_prompt(tr("manual_step_finish"))

    def _on_manual_continue(self):
        """Thực hiện đúng một bước Manual đang chờ."""
        if self._mission_source != "routes" or not self._manual_mode:
            return
        if self._manual_pending_action is None:
            return

        kind, idx = self._manual_pending_action
        self._manual_pending_action = None

        if kind == "task":
            tasks = (self._mission_wps[idx].get("tasks", [])
                     if 0 <= idx < len(self._mission_wps) else [])
            if tasks:
                self._execute_tasks(
                    tasks,
                    on_done=self._manual_after_task_done,
                    token=self._mission_token,
                )
            else:
                self._manual_after_task_done(token=self._mission_token)

        elif kind == "move":
            # Task tại WP hiện tại đã xong: đánh dấu WP hiện tại DONE,
            # tăng mission_idx rồi mới gửi goal WP kế tiếp.
            self._advance(token=self._mission_token)

        elif kind == "move_after_skip":
            # _on_skip_wp() đã tăng mission_idx. Không được gọi _advance()
            # vì sẽ đánh dấu nhầm WP kế tiếp DONE và bỏ qua thêm một điểm.
            self._run_next(token=self._mission_token)

    def _on_mode_changed(self, manual: bool):
        """RoutesPage đổi Auto/Manual — chỉ có tác dụng khi mission rảnh
        (RoutesPage đã tự khoá 2 nút này trong lúc đang chạy)."""
        self._manual_mode = manual

    # ── [D] Stuck-handling: Thử lại / Bỏ qua / Huỷ ──────────────────────

    def _on_retry_wp(self):
        """Gửi lại đúng goal của waypoint đang dở — Nav2 tự replan từ
        vị trí hiện tại của robot (kể cả sau khi operator lái tay)."""
        if not self._mission_stuck: return
        idx = self._stuck_wp_idx
        self._mission_stuck = False
        self._stuck_wp_idx = -1
        self._routes.clear_stuck()
        if not (0 <= idx < len(self._mission_wps)):
            return
        self._routes.set_wp_status(idx, WP_MOVING)
        wp = self._mission_wps[idx]
        msg = tr("nav_going", wp.get("label", "?"), wp.get("x", 0.0), wp.get("y", 0.0))
        self._nav.set_status(msg); self._routes.set_status(msg)
        self._run_next(token=self._mission_token)

    def _on_skip_wp(self):
        """Bỏ qua waypoint đang kẹt mà không bỏ nhầm waypoint kế tiếp."""
        if not self._mission_stuck:
            return
        idx = self._stuck_wp_idx
        self._mission_stuck = False
        self._stuck_wp_idx = -1
        self._routes.clear_stuck()
        if not (0 <= idx < len(self._mission_wps)):
            return

        self._routes.set_wp_status(idx, WP_DONE)
        self._mission_idx = idx + 1

        if self._mission_idx >= len(self._mission_wps):
            self._on_mission_done()
            return

        if self._manual_mode:
            self._mission_phase = "manual_wait"
            self._set_pause_controls(can_pause=False, paused=False)
            next_label = self._mission_wps[self._mission_idx].get("label", "?")
            self._manual_pending_action = (
                "move_after_skip", self._mission_idx)
            self._routes.manual_prompt(tr("manual_step_move", next_label))
        else:
            self._run_next(token=self._mission_token)

    def _on_cancel_stuck(self):
        """Huỷ mission do lỗi và ghi log failed thay vì cancelled."""
        if not self._mission_stuck:
            return
        self._mission_stuck = False
        self._stuck_wp_idx = -1
        self._routes.clear_stuck()
        self._stop_mission(log_status="failed")

    def _on_mission_done(self):
        source = self._mission_source
        self._mission_running = False
        self._mission_paused = False
        self._mission_phase = "idle"
        self._cancel_reason = None
        self._confirm_continue = None
        self._cancel_mission_timers()
        self._clear_route_confirm()
        self._set_pause_controls(can_pause=False, paused=False)

        self._nav.set_status(tr("nav_done"))
        self._nav.set_mission_running(False)
        self._nav.map_widget.clear_nav_path()
        self._notify_away(tr("mb_done_toast"), "ok", 6000)
        self._update_mission_bar()

        if source == "routes" and self._schedule.queue:
            QTimer.singleShot(2500, self._try_run_queue)
        if source == "routes":
            self._routes.set_status(tr("nav_done"))
            self._routes.map_widget.clear_nav_path()
            try:
                self._routes.on_mission_success()
            except AttributeError:
                self._routes.set_mission_done()

    def _pause_mission(self):
        # Pause task có thể khiến timer bị huỷ rồi task chạy lại từ đầu.
        # Vì vậy chỉ cho Pause khi Nav2 đang di chuyển tới waypoint.
        if not self._mission_running or self._mission_paused:
            return

        if self._mission_phase != "moving":
            msg = "Chỉ có thể Pause khi robot đang di chuyển."
            self._nav.set_status(msg)
            if self._mission_source == "routes":
                self._routes.set_status(msg)
            self._set_pause_controls(can_pause=False, paused=False)
            return

        self._mission_paused = True
        self._mission_phase = "paused_moving"
        self._cancel_reason = "pause"
        self._cancel_mission_timers()
        self._clear_route_confirm()
        self._confirm_continue = None
        if self.nav_client:
            self.nav_client.cancel_goal()
        self._nav.set_mission_paused(True)
        self._set_pause_controls(can_pause=False, paused=True)
        self._nav.set_status("⏸ Đã tạm dừng.")
        if self._mission_source == "routes":
            self._routes.set_status("⏸ Đã tạm dừng.")

    def _resume_mission(self):
        if self._mission_running and self._mission_paused:
            self._mission_paused = False
            self._mission_phase = "moving"
            self._cancel_reason = None
            self._nav.set_mission_paused(False)
            self._set_pause_controls(can_pause=True, paused=False)
            self._run_next(token=self._mission_token)

    def _stop_mission(self, log_status: str = "cancelled"):
        source = self._mission_source
        was_running = self._mission_running
        if was_running:
            audit_log.audit("mission_stop", log_status)
        self._queue_run_active = False
        self._abort_fork_wait(send_stop=True)
        self._cancel_reason = "stop"
        self._new_mission_token()
        self._stop_nav_retry()
        self._cancel_mission_timers()
        self._clear_route_confirm()
        self._confirm_continue = None

        self._mission_stuck = False
        self._stuck_wp_idx = -1
        self._manual_pending_action = None
        try:
            self._routes.clear_stuck()
        except Exception:
            pass

        self._mission_running = False
        self._mission_paused = False
        self._mission_phase = "idle"
        self._mission_wps = []
        self._mission_idx = 0
        self._set_pause_controls(can_pause=False, paused=False)

        if self.nav_client:
            self.nav_client.cancel_goal()

        self._nav.set_status(tr("nav_stopped"))
        self._nav.set_mission_running(False)
        self.velocity_signal.emit(0.0, 0.0)
        self._nav.map_widget.clear_nav_path()

        if source == "routes":
            self._routes.set_status(tr("nav_stopped"))
            self._routes.map_widget.clear_nav_path()

            if was_running:
                try:
                    if getattr(self._routes, "_active_record", None) is not None:
                        self._routes._stop_logging(log_status)
                except Exception:
                    pass

            try:
                self._routes.set_mission_done()
            except Exception:
                pass

    # ── Window controls ───────────────────────────────────────────────

    def _toggle_maximize(self):
        if self.isMaximized(): self.showNormal()
        else: self.showMaximized()

    def closeEvent(self, ev):
        """Dọn process và thoát Qt, trừ trường hợp đổi người vận hành."""
        try:
            OVERRIDE.set(False)
        except Exception:
            pass
        try:
            self.shutdown_processes()
        except Exception as e:
            print(f"[MainWindow] Shutdown failed: {e}")

        super().closeEvent(ev)

        if ev.isAccepted() and not self._switching_user:
            app = QApplication.instance()
            if app is not None:
                # Tránh gọi quit lồng ngay bên trong closeEvent stack.
                QTimer.singleShot(0, app.quit)

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        if hasattr(self, "_toasts"):
            self._toasts.reposition()
        auto = ev.size().width() < 900
        if auto != self._sb_auto_compact:
            self._sb_auto_compact = auto
            self._apply_sidebar(animate=self.isVisible())

    def changeEvent(self, ev):
        super().changeEvent(ev)
        if ev.type() == QEvent.Type.WindowStateChange and hasattr(self, "_titlebar"):
            self._titlebar.set_maximized(self.isMaximized())

    # ── 4.7.0: sidebar / focus mode ──────────────────────────────────

    def _init_ui_extras(self):
        self._apply_sidebar(animate=False)
        QShortcut(QKeySequence("Ctrl+B"), self, activated=self.toggle_sidebar)
        QShortcut(QKeySequence("F11"), self, activated=self.toggle_focus_mode)
        QShortcut(QKeySequence("Esc"), self, activated=self._exit_focus_mode)

    def _apply_sidebar(self, animate: bool = True):
        self._sidebar.set_compact(self._sb_user_compact or self._sb_auto_compact, animate)

    def toggle_sidebar(self):
        """Collapse the sidebar to an icon rail / expand it again (remembered)."""
        if self._focus_mode:
            self.toggle_focus_mode()
            return
        self._sb_user_compact = not (self._sidebar.is_compact())
        prefs.set("ui_sidebar_compact", self._sb_user_compact)
        self._sb_auto_compact = False if not self._sb_user_compact else self._sb_auto_compact
        self._apply_sidebar()

    def _exit_focus_mode(self):
        if self._focus_mode:
            self.toggle_focus_mode()

    def toggle_focus_mode(self):
        """Hide the sidebar and every side panel of the current page (maximum map); toggle back."""
        self._focus_mode = not self._focus_mode
        if self._focus_mode:
            self._focus_collapsed = collapse_panels(self._pages.currentWidget(), True)
            self._sidebar.set_hidden(True)
        else:
            self._sidebar.set_hidden(False)
            collapse_panels(self._pages, False,
                            only=[p for p in self._focus_collapsed if p is not None])
            self._focus_collapsed = []
        self._titlebar.set_focus_mode(self._focus_mode)

    # draw path on map widget 
       
    def on_nav_path_update(self, pts: list):
        self._nav.map_widget.update_nav_path(pts)
        self._routes.map_widget.update_nav_path(pts)

    def on_fleet_snapshot_update(self, payload: str):
        """Receive aggregated multi-AGV state from /fleet/snapshot."""
        self._fleet.update_snapshot_json(payload)


    # ── i18n ─────────────────────────────────────────────────────────

    def _retranslate_all(self):
        self._titlebar.retranslate(); self._sidebar.retranslate()
        self._home.retranslate(); self._mapping.retranslate()
        self._nav.retranslate(); self._routes.retranslate()
        self._conveyor.retranslate(); self._maplib.retranslate()
        self._settings.retranslate(); self._fleet.retranslate()
        self._alarm_page.retranslate(); self._diag.retranslate()
        self._reports.retranslate(); self._stations.retranslate(); self._schedule.retranslate()
        self._safety.retranslate(); self._forklift.retranslate()
        self._dashboard.retranslate()
        self._subtabs.retranslate()
        for js in self.findChildren(JoystickWidget):
            js.retranslate()
        for panel in self.findChildren(CollapsiblePanel):
            panel.retranslate()
        self._estop_banner.setText(tr("estop_banner"))
        self.setWindowTitle(tr("app_name"))
        self._switch_page(self._pages.currentIndex())
