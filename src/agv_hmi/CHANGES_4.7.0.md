# agv_hmi 4.7.0 - UI: collapsible bars + animation

## Ẩn / hiện thanh
- Panel bên (Mapping, Navigation, Routes: danh sách + trạng thái, Fleet: bảng + chi tiết): tay nắm ở mép, trượt có animation, nhớ trạng thái (`ui_panels` trong `~/.agv_hmi/hmi_prefs.json`).
- Sidebar: thu gọn thành thanh icon (nút ☰, nút "Thu gọn", **Ctrl+B**); tự thu gọn khi cửa sổ < 900 px.
- Chế độ tập trung: nút ⛶ / **F11** (thoát: F11 hoặc **Esc**) - ẩn sidebar và mọi panel của trang.
- Nhóm (GroupBox) trong panel: bấm tiêu đề để gập / mở.
- Joystick: nút tròn nhỏ góc trái dưới để thu / mở.

## Animation / giao diện
- Chuyển trang có fade; toast hiện/tắt mượt; nút sidebar hover/active mượt; nút cửa sổ vẽ tay.
- Settings: checkbox tắt hiệu ứng (`ui_animations`) cho máy HMI chậm.
- Sửa selector `QFrame{}` làm viền rò xuống label con (Home, Routes, Login); sửa độ rộng panel Navigation/Mapping; khối QSS polish.

## File mới
`ui/anim.py`, `ui/collapsible.py`, `ui/nav_button.py`, `test/test_ui_collapsible.py`.

## Biết còn tồn tại
- Theme light: chữ title bar và thẻ Home còn màu cứng của dark.
- Safety / Diagnostics / Forklift / Reports chưa có thanh ẩn/hiện.
