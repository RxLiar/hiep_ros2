"""
main.py — AGV Busan Autonomous Robot HMI

Entry point:
  LoginDialog -> RosInterface -> MainWindow -> Qt event loop

Fix:
  - ThemeManager khởi tạo TRƯỚC LoginDialog để theme áp dụng ngay.
  - theme_mgr được truyền vào MainWindow → SettingsPage.
"""

import sys

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

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


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("AGV Busan Autonomous Robot")

    # ── Theme — khởi tạo và apply TRƯỚC mọi widget ─────────────────
    theme_mgr = ThemeManager(app)
    theme_mgr.apply_initial()

    # ── Login ───────────────────────────────────────────────────────
    login = LoginDialog()
    if login.exec() != LoginDialog.DialogCode.Accepted:
        import sys as _sys; _sys.exit(0)

    role = login.result_role()

    # ── ROS + MainWindow ────────────────────────────────────────────
    ros = RosInterface(args=sys.argv)
    window = MainWindow(role=role, theme_mgr=theme_mgr)

    window.nav_client = NavClient(ros.node)
    window.map_saver  = MapSaver(ros.node)
    window.set_conveyor_cmd_callback(ros.publish_conveyor_cmd)

    # ── ROS → UI ────────────────────────────────────────────────────
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
    _safe_connect(ros, "path_signal", window.on_nav_path_update)

    # Nav2 planner/controller status → error header
    _safe_connect(ros, "nav2_status_signal",    window.on_nav2_status_update)

    # ── UI → ROS ────────────────────────────────────────────────────
    window.velocity_signal.connect(ros.publish_velocity)
    window.pose_estimate_signal.connect(ros.publish_initial_pose)

    # ── ROS spin timer ──────────────────────────────────────────────
    timer = QTimer()
    timer.timeout.connect(ros.spin_once)
    timer.start(10)

    # ── Shutdown ────────────────────────────────────────────────────
    def _shutdown():
        try: timer.stop()
        except Exception: pass
        try: window.shutdown_processes()
        except Exception: pass
        try: ros.shutdown()
        except Exception: pass

    app.aboutToQuit.connect(_shutdown)

    window.show()
    import sys as _sys; _sys.exit(app.exec())


if __name__ == "__main__":
    main()