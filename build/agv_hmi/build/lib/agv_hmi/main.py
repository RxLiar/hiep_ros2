"""
main.py — AGV Busan Autonomous Robot HMI

Entry point (Session pattern):
  ThemeManager -> Session.start_login() -> LoginDialog -> MainWindow
  -> Qt event loop

[NEW] Session giữ RosInterface/NavClient/MapSaver SỐNG xuyên suốt nhiều
  lần đổi người vận hành (operator/engineer). Khi MainWindow emit
  switch_user_signal, Session đóng MainWindow cũ và mở lại LoginDialog
  mà KHÔNG re-init rclpy — tránh phải khởi động lại toàn bộ ROS node.

[FIX] Bỏ hẳn QTimer gọi ros.spin_once() mỗi 10ms — RosInterface đã tự
  chạy SingleThreadedExecutor trong một spin thread riêng (xem
  RosInterface._spin_loop trong ros_node.py). spin_once() hiện tại chỉ
  là hàm rỗng (pass), nên QTimer cũ không có tác dụng gì, chỉ gây nhiễu
  logic khi đọc code.
"""
import sys

from PyQt6.QtWidgets import QApplication

from agv_hmi.ui.login_dialog import LoginDialog
from agv_hmi.ui.main_window import MainWindow
from agv_hmi.ros.ros_node import RosInterface
from agv_hmi.ros.nav_client import NavClient
from agv_hmi.ros.map_saver import MapSaver
from agv_hmi.ui.theme_manager import ThemeManager


def _safe_connect(signal_owner, signal_name: str, slot):
    sig = getattr(signal_owner, signal_name, None)
    if sig is None:
        print(f"[HMI] Warning: RosInterface thiếu signal '{signal_name}'")
        return False
    try:
        sig.connect(slot)
        return True
    except Exception as e:
        print(f"[HMI] Không connect được signal '{signal_name}': {e}")
        return False


class Session:
    """
    Quản lý vòng đời MainWindow qua nhiều lần đổi người vận hành, trong
    khi ROS node (RosInterface/NavClient/MapSaver) chỉ khởi tạo MỘT LẦN
    và sống xuyên suốt cho tới khi thoát app.
    """

    def __init__(self, app: QApplication, theme_mgr: ThemeManager):
        self.app = app
        self.theme_mgr = theme_mgr
        self.ros: RosInterface | None = None
        self.nav_client: NavClient | None = None
        self.map_saver: MapSaver | None = None
        self.window: MainWindow | None = None

    def _ensure_ros(self):
        if self.ros is not None:
            return
        self.ros = RosInterface(args=sys.argv)
        self.nav_client = NavClient(self.ros.node)
        self.map_saver = MapSaver(self.ros.node)
        # FIX: KHÔNG cần QTimer gọi ros.spin_once() nữa — RosInterface
        # đã tự chạy SingleThreadedExecutor trong spin thread riêng.
        # spin_once() hiện là no-op; timer cũ chỉ gọi hàm rỗng mỗi 10ms.

    def _build_window(self, role: str) -> MainWindow:
        window = MainWindow(role=role, theme_mgr=self.theme_mgr)
        window.nav_client = self.nav_client
        window.map_saver = self.map_saver
        window.set_conveyor_cmd_callback(self.ros.publish_conveyor_cmd)

        ros = self.ros
        _safe_connect(ros, "pose_signal",           window.on_pose_update)
        _safe_connect(ros, "mapping_pose_signal",   window.on_mapping_pose_update)
        _safe_connect(ros, "odom_signal",           window.on_odom_update)
        _safe_connect(ros, "odom_twist_signal",     window.on_odom_twist_update)
        _safe_connect(ros, "map_signal",            window.on_map_update)
        _safe_connect(ros, "scan_signal",           window.on_scan_update)
        _safe_connect(ros, "robot_status_signal",   window.on_robot_status_update)
        _safe_connect(ros, "battery_signal",        window.on_battery_update)
        _safe_connect(ros, "connection_signal",     window.on_connection_update)
        _safe_connect(ros, "agv_moving_signal",     window.on_agv_status_update)
        _safe_connect(ros, "conveyor_cargo_signal", window.on_conveyor_cargo_update)
        _safe_connect(ros, "sensor_signal",         window.on_sensor_update)
        _safe_connect(ros, "bumper_signal",         window.on_bumper_update)
        _safe_connect(ros, "path_signal",           window.on_nav_path_update)
        _safe_connect(ros, "nav2_status_signal",    window.on_nav2_status_update)

        window.velocity_signal.connect(ros.publish_velocity)
        window.pose_estimate_signal.connect(ros.publish_initial_pose)

        # [NEW] Đổi người vận hành → đóng MainWindow, mở lại Login,
        # ROS node vẫn sống nguyên.
        window.switch_user_signal.connect(self._on_switch_user)
        return window

    def _on_switch_user(self):
        old_window, self.window = self.window, None
        if old_window is not None:
            try:
                old_window.shutdown_processes()
            except Exception:
                pass
            old_window.close()
            old_window.deleteLater()
        self.start_login()

    def start_login(self):
        self._ensure_ros()
        login = LoginDialog()
        if login.exec() != LoginDialog.DialogCode.Accepted:
            self.app.quit()
            return
        role = login.result_role()
        self.window = self._build_window(role)
        self.window.show()

    def shutdown(self):
        if self.window is not None:
            try:
                self.window.shutdown_processes()
            except Exception:
                pass
        if self.ros is not None:
            try:
                self.ros.shutdown()
            except Exception:
                pass


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("AGV Busan Autonomous Robot")
    # Quan trọng: tránh app tự thoát khi MainWindow đóng để quay lại Login
    # (mặc định Qt sẽ quit khi window cuối cùng đóng).
    app.setQuitOnLastWindowClosed(False)

    theme_mgr = ThemeManager(app)
    theme_mgr.apply_initial()

    session = Session(app, theme_mgr)
    app.aboutToQuit.connect(session.shutdown)

    session.start_login()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
