"""version.py — Nguồn version duy nhất cho toàn app.

v4.5.0: bản đồ riêng của HMI (làm sạch + tường thẳng), cảnh báo, chẩn đoán, kiểm tra trước khi chạy, ...
v4.4.0: lớp phần cứng — nút Bật/Tắt motor (/motor_enable), thẻ PLC + E-stop,
        trạng thái driver KEYA, trang băng tải còn 8 cảm biến (4 belt x 2).

v4.3.0: thêm Stuck-handling (Thử lại/Bỏ qua/Huỷ khi mission gặp sự cố)
        + chế độ Auto/Manual trên Routes screen.
"""

APP_VERSION = "4.5.0"
ROS_DISTRO = "ROS2 Jazzy"


def full_label() -> str:
    return f"v{APP_VERSION} · {ROS_DISTRO}"
