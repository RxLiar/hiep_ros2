"""version.py — Nguồn version duy nhất cho toàn app.

v4.3.0: thêm Stuck-handling (Thử lại/Bỏ qua/Huỷ khi mission gặp sự cố)
        + chế độ Auto/Manual trên Routes screen.
"""

APP_VERSION = "4.3.0"
ROS_DISTRO = "ROS2 Jazzy"


def full_label() -> str:
    return f"v{APP_VERSION} · {ROS_DISTRO}"
