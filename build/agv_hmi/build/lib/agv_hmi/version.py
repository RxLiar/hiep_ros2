"""version.py — Nguồn version duy nhất cho toàn app.

v4.11.0: Fleet — bật Fleet Server và kết nối robot ngay trên giao diện (không gõ lệnh), có log, kiểm tra gói ROS, chip Fleet.
v4.10.0: Sidebar còn 6 thẻ chính (Home, Navigation, Mapping, Routes, Fleet, Settings), các trang khác thành tab con;
        chip trạng thái Safety / Nav2 / Mission trên thanh tiêu đề; Home hiện cả vận tốc góc.
v4.9.1: Mapping — mở map đã lưu (.yaml) để sửa tiếp, lưu ra tên mới.
v4.9.0: Mapping — sửa map bằng tay (bút, tẩy, đường thẳng, chữ nhật, làm dày/mỏng tường, hoàn tác/làm lại),
        đo nhiều điểm; mission tiếp tục chạy khi sang thẻ khác (thanh trạng thái + cảnh báo).
v4.8.0: Safety page có 2 lidar (mỗi cái 1 topic + pose riêng); Engineer tắt vùng safety khi teleop
        (công tắc trên joystick, tự bật lại khi chạy mission / mất HMI); sửa safety_monitor crash khi khởi động.
v4.7.0: thanh ẩn/hiện (panel, sidebar, nhóm, joystick), chế độ tập trung, animation.
v4.6.0: fork lift (loại robot, task pallet, bypass vùng đầu càng, trang Fork lift).
v4.5.0: bản đồ riêng của HMI (làm sạch + tường thẳng), cảnh báo, chẩn đoán, kiểm tra trước khi chạy, ...
v4.4.0: lớp phần cứng — nút Bật/Tắt motor (/motor_enable), thẻ PLC + E-stop,
        trạng thái driver KEYA, trang băng tải còn 8 cảm biến (4 belt x 2).

v4.3.0: thêm Stuck-handling (Thử lại/Bỏ qua/Huỷ khi mission gặp sự cố)
        + chế độ Auto/Manual trên Routes screen.
"""

APP_VERSION = "4.11.0"
ROS_DISTRO = "ROS2 Jazzy"


def full_label() -> str:
    return f"v{APP_VERSION} · {ROS_DISTRO}"
