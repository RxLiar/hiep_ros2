"""version.py — Nguồn version duy nhất cho toàn app.

Trước đây version bị lệch nhau ở 3 nơi:
  - agv_hmi/__init__.py        -> "4.2.0"
  - agv_hmi/ui/i18n.py          -> "v4.2.0 · ROS2 Jazzy"
  - agv_hmi/ui/main_window.py   -> "v4.2.6 · ROS2 Jazzy"
Giờ tất cả import từ đây.
"""

APP_VERSION = "4.2.6"
ROS_DISTRO = "ROS2 Jazzy"


def full_label() -> str:
    return f"v{APP_VERSION} · {ROS_DISTRO}"
