"""
main_window.py — AMR Pacific Autonomous Robot v4.3.0

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
from agv_hmi.version import full_label

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QStackedWidget, QLabel, QPushButton, QMessageBox, QApplication
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer

from agv_hmi.ui.i18n import tr, set_lang, get_lang, LANGUAGES
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
    page_changed          = pyqtSignal(int)
    retranslate_requested = pyqtSignal()

    IDX_HOME     = 0
    IDX_MAPPING  = 1
    IDX_NAV      = 2
    IDX_ROUTES   = 3
    IDX_CONVEYOR = 4
    IDX_MAPLIB   = 5
    IDX_SETTINGS = 6
    IDX_FLEET    = 7

    def __init__(self, role: str = "operator"):
        super().__init__()
        self.setObjectName("Sidebar")
        self.setFixedWidth(210)
        self._role = role
        self._btns: list[tuple[QPushButton, int, str]] = []
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
        txt = QWidget(); tl = QVBoxLayout(txt)
        tl.setContentsMargins(0, 0, 0, 0); tl.setSpacing(1)
        self._app_lbl = QLabel(tr("app_name"))
        self._app_lbl.setObjectName("AppTitle")
        self._app_lbl.setWordWrap(True)
        self._ver_lbl = QLabel(full_label())
        self._ver_lbl.setObjectName("AppVersion")
        tl.addWidget(self._app_lbl); tl.addWidget(self._ver_lbl)
        ll.addWidget(self._logo); ll.addSpacing(8); ll.addWidget(txt)
        lay.addWidget(logo_w)

        nav_w = QWidget(); nav_w.setContentsMargins(8, 10, 8, 0)
        self._nav_lay = QVBoxLayout(nav_w)
        self._nav_lay.setContentsMargins(0, 0, 0, 0)
        self._nav_lay.setSpacing(2)
        lay.addWidget(nav_w)
        lay.addStretch()

        bot = QWidget(); bot.setFixedHeight(96)
        bl = QVBoxLayout(bot); bl.setContentsMargins(12, 4, 12, 8); bl.setSpacing(5)
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
        lay.addWidget(bot)

        self._rebuild_nav()
        self._update_mode()

    def _rebuild_nav(self):
        while self._nav_lay.count():
            item = self._nav_lay.takeAt(0)
            if item.widget(): item.widget().deleteLater()
        self._btns.clear()

        engineer_only = {self.IDX_MAPPING, self.IDX_MAPLIB}
        pages = [
            ("⌂",  "page_home",       self.IDX_HOME),
            ("◉",  "page_fleet",      self.IDX_FLEET),
            ("◎",  "page_mapping",    self.IDX_MAPPING),
            ("⊳",  "page_navigation", self.IDX_NAV),
            ("≡",  "page_routes",     self.IDX_ROUTES),
            ("⟳",  "page_conveyors",  self.IDX_CONVEYOR),
            ("🗺", "page_maplib",     self.IDX_MAPLIB),
            ("⚙",  "page_settings",   self.IDX_SETTINGS),
        ]
        for icon, key, idx in pages:
            if idx in engineer_only and self._role != "engineer":
                continue
            b = QPushButton(f"  {icon}  {tr(key)}")
            b.setObjectName("NavBtn"); b.setFixedHeight(38)
            b.clicked.connect(lambda _, i=idx: self.switch(i))
            self._nav_lay.addWidget(b)
            self._btns.append((b, idx, key))
        self._nav_lay.addStretch()

    def switch(self, idx: int):
        for b, bidx, _ in self._btns:
            b.setProperty("active", str(bidx == idx).lower())
            b.style().unpolish(b); b.style().polish(b)
        self.page_changed.emit(idx)

    def set_ros_status(self, txt: str, ok: bool = True):
        self._dot.setStyleSheet(
            f"color:{'#3FB950' if ok else '#E3B341'};font-size:10px;")
        self._stxt.setText(txt)

    def _update_mode(self):
        mode  = tr("mode_engineer") if self._role == "engineer" else tr("mode_operator")
        color = "#58A6FF" if self._role == "engineer" else "#3FB950"
        self._mode_lbl.setText(f"● {mode}")
        self._mode_lbl.setStyleSheet(
            f"color:{color};font-size:10px;font-weight:600;")

    def retranslate(self):
        self._app_lbl.setText(tr("app_name"))
        self._update_mode()
        for b, _, key in self._btns:
            icon = b.text().strip().split("  ")[0]
            b.setText(f"  {icon}  {tr(key)}")
        if hasattr(self, "_lang_selector"):
            self._lang_selector.retranslate()


# ── MainWindow ────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    velocity_signal      = pyqtSignal(float, float)
    pose_estimate_signal = pyqtSignal(float, float, float)

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

        self.setWindowTitle(tr("app_name"))
        self.setMinimumSize(1024, 600)
        self.resize(1440, 860)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)

        self.nav_client       = None
        self.map_saver        = None
        self._conveyor_cmd_cb = None

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

        self._build()
        self._wire()

        # FIX threading: connect signal nội bộ — Qt tự dùng
        # QueuedConnection khi emit từ thread khác main thread.
        self._nav_result_signal.connect(self._on_wp_result)
        self._nav_feedback_signal.connect(self._on_feedback)

        self._sidebar.switch(Sidebar.IDX_HOME)

    # ── Build ────────────────────────────────────────────────────────

    def _build(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0); root.setSpacing(0)

        self._titlebar = TitleBar(self)
        root.addWidget(self._titlebar)

        body = QWidget()
        body_lay = QHBoxLayout(body)
        body_lay.setContentsMargins(0, 0, 0, 0); body_lay.setSpacing(0)

        self._sidebar = Sidebar(role=self._role)
        body_lay.addWidget(self._sidebar)

        self._pages    = QStackedWidget()
        self._home     = HomePage()
        self._mapping  = MappingPage()
        self._nav      = NavigationPage(role=self._role)
        self._routes   = RoutesPage(role=self._role)
        self._conveyor = ConveyorPage()
        self._maplib   = MapLibraryPage()
        self._settings = SettingsPage(role=self._role, theme_mgr=self._theme_mgr)
        self._fleet    = FleetPage()

        for p in (self._home, self._mapping, self._nav, self._routes,
                  self._conveyor, self._maplib, self._settings, self._fleet):
            self._pages.addWidget(p)

        body_lay.addWidget(self._pages)
        root.addWidget(body)

    # ── Wire ─────────────────────────────────────────────────────────

    def _wire(self):
        self._titlebar.minimize_clicked.connect(self.showMinimized)
        self._titlebar.maximize_clicked.connect(self._toggle_maximize)
        self._titlebar.close_clicked.connect(self.close)
        if hasattr(self._titlebar, "language_changed"):
            self._titlebar.language_changed.connect(
                lambda _: self._retranslate_all())

        self._sidebar.page_changed.connect(self._switch_page)
        self._sidebar.retranslate_requested.connect(self._retranslate_all)

        self._home.connection_toggle.connect(self._on_connection_toggle)
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

    def _switch_page(self, idx: int):
        old_idx = self._current_page_idx
        if old_idx is not None and old_idx != idx:
            self._cleanup_leaving_page(old_idx, idx)

        self._current_page_idx = idx
        self._pages.setCurrentIndex(idx)
        titles = {
            Sidebar.IDX_HOME:     tr("page_home"),
            Sidebar.IDX_MAPPING:  tr("page_mapping"),
            Sidebar.IDX_NAV:      tr("page_navigation"),
            Sidebar.IDX_ROUTES:   tr("page_routes"),
            Sidebar.IDX_CONVEYOR: tr("conv_title"),
            Sidebar.IDX_MAPLIB:   tr("maplib_title"),
            Sidebar.IDX_SETTINGS: tr("settings_title"),
            Sidebar.IDX_FLEET:    tr("page_fleet"),
        }
        self._titlebar.set_page_title(titles.get(idx, ""))
        if idx == Sidebar.IDX_ROUTES: self._routes.refresh()
        if idx == Sidebar.IDX_MAPLIB: self._maplib.refresh()

    def _cleanup_leaving_page(self, old_idx: int, new_idx: int):
        """Cleanup resources when changing pages.

        Fleet Monitor is an observation screen. Navigation/Routes -> Fleet and
        Fleet -> Navigation/Routes must preserve the active Nav2 goal and mission
        state. Moving from the observation screen to an unrelated page still
        stops an active mission, preserving the previous safety policy.
        """
        if old_idx == Sidebar.IDX_MAPPING:
            try:
                self._mapping.cleanup()
            except Exception:
                pass
            return

        control_pages = {
            Sidebar.IDX_NAV,
            Sidebar.IDX_ROUTES,
            Sidebar.IDX_FLEET,
        }

        if old_idx in (Sidebar.IDX_NAV, Sidebar.IDX_ROUTES):
            if new_idx == Sidebar.IDX_FLEET:
                # Observation only: mission continues in the background.
                return
            self._stop_mission()
            return

        if old_idx == Sidebar.IDX_FLEET and self._mission_running:
            if new_idx in control_pages:
                # Return to a mission/control screen without cancelling.
                return
            self._stop_mission()

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
            self._stop_nav_launch()
        except Exception as e:
            print(f"[Shutdown] Nav2 cleanup failed: {e}")

    # ── ROS → UI ─────────────────────────────────────────────────────

    def on_mapping_pose_update(self, x: float, y: float, yaw: float):
        self._mapping.update_pose(x, y, yaw)
        self._mapping.update_pose_on_map(x, y, yaw)

    def on_pose_update(self, x: float, y: float, yaw: float):
        self._last_x = x; self._last_y = y; self._last_yaw = yaw
        self._nav.update_pose_on_map(x, y, yaw)
        self._routes.update_pose(x, y, yaw)
        self._sidebar.set_ros_status(tr("status_amcl"), ok=True)

    def on_odom_update(self, x: float, y: float, yaw: float):
        self._odom_x = x; self._odom_y = y; self._odom_yaw = yaw
        self._nav.update_pose(x, y, yaw)

    def on_odom_twist_update(self, linear: float, angular: float):
        self._routes.update_velocity(linear, angular)

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

    def on_connection_update(self, state: str):
        self._titlebar.set_connection(state)
        self._home.update_connection(state)
        self._sidebar.set_ros_status(
            tr(f"status_{state}"), ok=(state != "offline"))

    def on_agv_status_update(self, moving: bool):
        self._home.update_agv_status(moving)

    def on_conveyor_cargo_update(self, belt_id: int, has_cargo: bool):
        self._home.update_conveyor_cargo(belt_id, has_cargo)

    def on_sensor_update(self, sensor_id: int, on: bool):
        self._conveyor.update_sensor(sensor_id, on)

    def on_bumper_update(self, side: str, triggered: bool):
        self._routes.update_bumper(side, triggered)

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
        for header in (self._nav.error_header, self._routes.error_header):
            if level in ("ok", "normal", "none", "0", "false"):
                header.clear_nav2()
            elif level in ("warn", "warning", "1"):
                header.set_nav2_warning(message or "Nav2 cảnh báo")
            else:
                header.set_nav2_error(message or "Nav2 lỗi")

    # ── Conveyor ─────────────────────────────────────────────────────

    def set_conveyor_cmd_callback(self, cb):
        self._conveyor_cmd_cb = cb

    def _on_conveyor_cmd(self, payload: dict):
        if self._conveyor_cmd_cb:
            self._conveyor_cmd_cb(payload)

    def _on_connection_toggle(self):
        self._sidebar.switch(Sidebar.IDX_NAV)

    # ── [NEW] Đổi người vận hành ─────────────────────────────────────

    def _on_switch_user_clicked(self):
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
        if not self.map_saver: return
        ok = self.map_saver.save(path)
        if ok:
            MapLibraryPage.save_thumbnail(path + ".yaml", self._mapping.map_widget)
        self._sidebar.set_ros_status(
            tr("map_saved") if ok else tr("map_save_fail"), ok)
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
        if route_id:
            ok = RM.update_route(route_id, name, mp, wps)
            route = RM.get_route(route_id) or {
                "id": route_id, "name": name,
                "map_path": mp, "waypoints": wps}
            if not ok:
                self._sidebar.set_ros_status(
                    f"Không cập nhật được: {name}", ok=False)
                return
        else:
            route = RM.create_route(name, mp, wps)
        try:
            thumb_dir = os.path.expanduser("~/.agv_hmi/route_thumbnails")
            os.makedirs(thumb_dir, exist_ok=True)
            thumb_path = os.path.join(thumb_dir, f"{route['id']}.png")
            pix = self._nav.map_widget.grab()
            if not pix.isNull():
                pix = pix.scaled(240, 160,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation)
                pix.save(thumb_path, "PNG")
                routes = RM.load_all()
                for r in routes:
                    if r.get("id") == route.get("id"):
                        r["thumbnail_path"] = thumb_path; break
                RM.save_all(routes)
        except Exception as e:
            print(f"[Route thumbnail] save failed: {e}")
        action = "Đã cập nhật" if route_id else "Đã lưu"
        self._sidebar.set_ros_status(f"{action}: {name}", ok=True)
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
        """
        Nhận signal từ RoutesPage._on_start().

        [+] Đảm bảo Nav2 chạy với ĐÚNG map của route này thông qua
        _ensure_nav_running(mp) — nếu Nav2 đang chạy với map khác,
        sẽ tự stop() + start() lại với map đúng, tránh lệch vị trí
        robot giữa RViz và app khi chạy Routes.

        [D][E] mission_source = "routes" — bật Stuck-handling và
        Auto/Manual cho mission này.
        """
        self._mission_source = "routes"
        route = copy.deepcopy(route or {})
        mp = self._normalize_map_path(route.get("map_path", ""))
        if not mp or not os.path.exists(mp):
            self._routes.set_status(
                f"Route thiếu map .yaml hợp lệ.\n"
                f"path: {route.get('map_path', '(trống)')}")
            self._sidebar.set_ros_status("Route thiếu map hợp lệ", ok=False)
            try: self._routes.set_mission_done()
            except Exception: pass
            return

        wps = copy.deepcopy(route.get("waypoints", []))
        if not wps:
            self._routes.set_status("Route không có waypoint.")
            try: self._routes.set_mission_done()
            except Exception: pass
            return

        token = self._new_mission_token()
        self._cancel_mission_timers()
        self._clear_route_confirm()
        self._cancel_reason = None
        self._stop_nav_retry()

        # [+] Đảm bảo Nav2 chạy đúng map của route (restart nếu cần)
        ok, msg = self._ensure_nav_running(mp)
        try: self._nav.set_nav_launch_running(ok, msg)
        except Exception: pass
        self._sidebar.set_ros_status(msg, ok=ok)
        if not ok:
            self._routes.set_status(msg)
            try: self._routes.set_mission_done()
            except Exception: pass
            return

        self._routes.set_status("Đang chờ Nav2 action server sẵn sàng...")
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
        ]
        parallel = [
            t for t in tasks
            if t.get("order") == "parallel" and t.get("type") != "confirm"
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
            self._request_route_confirm(); return
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
        w = ev.size().width()
        collapsed = w < 900
        if collapsed != getattr(self, '_sb_collapsed', False):
            self._sb_collapsed = collapsed
            if collapsed:
                self._sidebar.setFixedWidth(52)
                if hasattr(self._sidebar, "_lang_selector"):
                    self._sidebar._lang_selector.setVisible(False)
                for b, _, _ in self._sidebar._btns:
                    icon = b.text().strip().split("  ")[0]
                    b.setText(f" {icon} ")
            else:
                self._sidebar.setFixedWidth(210)
                if hasattr(self._sidebar, "_lang_selector"):
                    self._sidebar._lang_selector.setVisible(True)
                for b, _, key in self._sidebar._btns:
                    icon = b.text().strip()[0]
                    b.setText(f"  {icon}  {tr(key)}")


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
        self.setWindowTitle(tr("app_name"))
        self._switch_page(self._pages.currentIndex())
