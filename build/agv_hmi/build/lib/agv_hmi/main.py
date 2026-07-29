"""
main.py — AMR Pacific Autonomous Robot HMI v4.3.3

Adds Fleet Monitor wiring while keeping ROS alive across user switching.
"""
import signal
import sys

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from agv_hmi.ros.map_saver import MapSaver
from agv_hmi.ros.nav_client import NavClient
from agv_hmi.ros.ros_node import RosInterface
from agv_hmi.ui.login_dialog import LoginDialog
from agv_hmi.ui.main_window import MainWindow
from agv_hmi.ui.theme_manager import ThemeManager


def _safe_connect(signal_owner, signal_name: str, slot):
    sig = getattr(signal_owner, signal_name, None)
    if sig is None:
        print(f"[HMI] Warning: RosInterface thiếu signal '{signal_name}'")
        return False
    try:
        sig.connect(slot)
        return True
    except Exception as exc:
        print(f"[HMI] Không connect được signal '{signal_name}': {exc}")
        return False


class Session:
    """Giữ ROS sống xuyên suốt các lần đổi operator/engineer."""

    def __init__(self, app: QApplication, theme_mgr: ThemeManager):
        self.app = app
        self.theme_mgr = theme_mgr
        self.ros: RosInterface | None = None
        self.nav_client: NavClient | None = None
        self.map_saver: MapSaver | None = None
        self.window: MainWindow | None = None
        self._shutdown_done = False

    def _ensure_ros(self):
        if self._shutdown_done or self.ros is not None:
            return
        self.ros = RosInterface(args=sys.argv)
        self.nav_client = NavClient(self.ros.node)
        self.map_saver = MapSaver(self.ros.node)

    def _build_window(self, role: str) -> MainWindow:
        if self.ros is None or self.nav_client is None or self.map_saver is None:
            raise RuntimeError("ROS interface chưa được khởi tạo")

        window = MainWindow(role=role, theme_mgr=self.theme_mgr)
        window.nav_client = self.nav_client
        window.map_saver = self.map_saver
        window.set_conveyor_cmd_callback(self.ros.publish_conveyor_cmd)

        ros = self.ros
        _safe_connect(ros, "pose_signal", window.on_pose_update)
        _safe_connect(ros, "mapping_pose_signal", window.on_mapping_pose_update)
        _safe_connect(ros, "odom_signal", window.on_odom_update)
        _safe_connect(ros, "odom_twist_signal", window.on_odom_twist_update)
        _safe_connect(ros, "map_signal", window.on_map_update)
        _safe_connect(ros, "scan_signal", window.on_scan_update)
        _safe_connect(ros, "robot_status_signal", window.on_robot_status_update)
        _safe_connect(ros, "battery_signal", window.on_battery_update)
        _safe_connect(ros, "connection_signal", window.on_connection_update)
        _safe_connect(ros, "agv_moving_signal", window.on_agv_status_update)
        _safe_connect(ros, "conveyor_cargo_signal", window.on_conveyor_cargo_update)
        _safe_connect(ros, "sensor_signal", window.on_sensor_update)
        _safe_connect(ros, "bumper_signal", window.on_bumper_update)
        _safe_connect(ros, "path_signal", window.on_nav_path_update)
        _safe_connect(ros, "nav2_status_signal", window.on_nav2_status_update)
        _safe_connect(ros, "fleet_snapshot_signal", window.on_fleet_snapshot_update)

        window.velocity_signal.connect(ros.publish_velocity)
        window.pose_estimate_signal.connect(ros.publish_initial_pose)
        window.switch_user_signal.connect(self._on_switch_user)
        return window

    def _on_switch_user(self):
        if self._shutdown_done:
            return
        old_window, self.window = self.window, None
        if old_window is not None:
            old_window.close()
            old_window.deleteLater()
        QTimer.singleShot(0, self.start_login)

    def start_login(self):
        if self._shutdown_done:
            return
        self._ensure_ros()
        if self.ros is None:
            self.app.quit()
            return
        login = LoginDialog()
        result = login.exec()
        if result != LoginDialog.DialogCode.Accepted:
            self.app.quit()
            return
        self.window = self._build_window(login.result_role())
        self.window.show()

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        window, self.window = self.window, None
        ros, self.ros = self.ros, None
        self.nav_client = None
        self.map_saver = None
        if window is not None:
            try:
                window.shutdown_processes()
            except Exception as exc:
                print(f"[HMI] Window shutdown failed: {exc}")
        if ros is not None:
            try:
                ros.shutdown()
            except Exception as exc:
                print(f"[HMI] ROS shutdown failed: {exc}")


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("AGV Busan Autonomous Robot")
    app.setQuitOnLastWindowClosed(False)

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    signal_tick = QTimer(app)
    signal_tick.setInterval(200)
    signal_tick.timeout.connect(lambda: None)
    signal_tick.start()

    theme_mgr = ThemeManager(app)
    theme_mgr.apply_initial()
    session = Session(app, theme_mgr)
    app.aboutToQuit.connect(session.shutdown)
    QTimer.singleShot(0, session.start_login)

    exit_code = app.exec()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
