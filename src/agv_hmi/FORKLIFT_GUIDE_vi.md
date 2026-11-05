# Hướng dẫn fork lift (agv_hmi 4.6.2 + hiep_robot2 0.7)

## 1. Cài đặt (làm một lần)
1. **Hồ sơ robot** — trang *Robot & An toàn* → `➕ Fork lift` → đặt tên.
   Nhập *Dài* (tính cả càng), *Rộng*, tâm thân, vùng STOP/SAFETY → `Áp dụng & lưu`.
2. **Thông số fork lift** — trang *Fork lift* → tab **Thông số** → chỉnh → `💾 Lưu thông số fork lift`.
   (Gõ số rồi nhấn Enter/Tab. Dòng "forklift_node đang dùng đúng các thông số đã lưu" báo node đã nhận.)
3. **Chạy phần cứng** — *Chẩn đoán → Phần cứng*: profile `real`, safety `true`, forklift `true` (càng ảo) hoặc `plc` (càng thật).
   Cần bật `FORK_ENABLED` trong firmware Mega và kiểm tra chân/cảm biến chiều cao trước khi dùng `plc`.

## 2. Thử từng bước (tab Điều khiển)
`Đi tới chiều cao` → `Vào pallet` → `Nhận hàng` / `Thả hàng` (giữ nút 0.8 giây). `Dừng fork lift` huỷ tác vụ.

## 3. Lập chương trình
*Điều hướng* → Loại robot **Fork lift** → waypoint đứng **trước pallet, thẳng hàng** → task `Nhận hàng`;
waypoint trước kệ → task `Thả hàng` → lưu lộ trình → chạy ở trang *Lộ trình*.
Mỗi task có thể ghi đè thông số (độ sâu vào, chiều cao...) cho riêng waypoint đó.

## 4. Dữ liệu lưu ở đâu
| Dữ liệu | File |
|---|---|
| Kích thước robot, vùng SAFETY/STOP, **thông số fork lift** (mọi hồ sơ) | `~/.agv_hmi/robot_profile.json` |
| Thông số riêng từng waypoint (task fork lift) | `~/agv_routes/routes.json` (trong từng route) |
| Cờ "đang mang hàng" (để nhớ sau khi khởi động lại) | `~/.agv_hmi/forklift_state.json` |
| Cảnh báo / nhật ký thao tác | `~/.agv_hmi/alarms.jsonl`, `~/.agv_hmi/audit.jsonl` |
| Tuỳ chọn HMI (kiểm tra trước khi chạy, tay cầm, đèn tháp...) | `~/.agv_hmi/hmi_prefs.json` |

`forklift_node` đọc `robot_profile.json` khi khởi động và nhận thay đổi trực tiếp qua `/safety_config`.
Sao lưu: *Cài đặt → Sao lưu* (gồm các file trên).

## 5. Khi có sự cố
- Không thấy trạng thái / "forklift_node chưa báo thông số": chưa chạy `forklift:=true|plc`.
- Task báo "blocked": robot không di chuyển được (vùng safety, vật cản, driver chưa bật).
- Task báo "fork stalled": càng không đổi chiều cao (cảm biến chiều cao, relay, nguồn thuỷ lực).
- Vùng safety không tác dụng: `ros2 run hiep_robot2 safety_check`.
