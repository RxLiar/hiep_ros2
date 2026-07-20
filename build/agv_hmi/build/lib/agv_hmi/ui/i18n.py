"""
i18n.py — 3 ngôn ngữ: Tiếng Việt / English / 한국어

[NEW] Thêm key cho:
  - home_switch_user: nút đổi người vận hành trên Home
  - route_repeat_*: setup lặp lại lộ trình (N lần / vô hạn) trên Routes
  - route_busy_warning: cảnh báo khi đang chạy/lặp mà chọn route khác
"""

_lang = "vi"

_S = {

    # Route delete confirm (takes 1 positional arg)
    "route_del_confirm": {"vi": "Xoá lộ trình '{}'?",
                          "en": "Delete route '{}'?",
                          "ko": "경로 '{}'을(를) 삭제할까요?"},

    # ── App ───────────────────────────────────────────────────────
    "app_name":          {"vi": "AGV Busan Autonomous Robot",
                          "en": "AGV Busan Autonomous Robot",
                          "ko": "AGV 부산 자율 로봇"},
    "app_version":       {"vi": "v4.2.7 · ROS2 Jazzy",
                          "en": "v4.2.7 · ROS2 Jazzy",
                          "ko": "v4.2.7 · ROS2 재지"},
    # ── Login ──────────────────────────────────────────────────────
    "login_select_mode": {"vi": "Chọn chế độ làm việc",
                          "en": "Select Operating Mode",
                          "ko": "운영 모드 선택"},
    "login_operator":    {"vi": "Người vận hành",
                          "en": "Operator",
                          "ko": "운영자"},
    "login_engineer":    {"vi": "Kỹ sư",
                          "en": "Engineer",
                          "ko": "엔지니어"},
    "login_op_desc":     {"vi": "Vận hành lộ trình, điều khiển băng tải",
                          "en": "Run routes, control conveyors",
                          "ko": "경로 운영, 컨베이어 제어"},
    "login_eng_desc":    {"vi": "Toàn quyền: Mapping, cấu hình lộ trình",
                          "en": "Full access: Mapping, route config",
                          "ko": "전체 접근: 매핑, 경로 설정"},
    "login_password":    {"vi": "Mật khẩu Kỹ sư",
                          "en": "Engineer Password",
                          "ko": "엔지니어 비밀번호"},
    "login_enter":       {"vi": "Vào →",
                          "en": "Enter →",
                          "ko": "입력 →"},
    "login_wrong_pw":    {"vi": "Sai mật khẩu!",
                          "en": "Wrong password!",
                          "ko": "비밀번호가 틀렸습니다!"},

    # ── Sidebar pages ──────────────────────────────────────────────
    "page_home":         {"vi": "Home",        "en": "Home",         "ko": "홈"},
    "page_mapping":      {"vi": "Dựng Maps",     "en": "Mapping",      "ko": "매핑"},
    "page_navigation":   {"vi": "Điều hướng",  "en": "Navigation",   "ko": "내비게이션"},
    "page_routes":       {"vi": "Lộ trình",    "en": "Routes",       "ko": "경로"},
    "page_conveyors":    {"vi": "Băng tải",    "en": "Conveyors",    "ko": "컨베이어"},
    "page_maplib":       {"vi": "Thư viện Map","en": "Map Library",  "ko": "지도 라이브러리"},
    "page_settings":     {"vi": "Cài đặt",     "en": "Settings",     "ko": "설정"},

    # ── Status / TopBar ────────────────────────────────────────────
    "status_offline":    {"vi": "Offline",     "en": "Offline",      "ko": "오프라인"},
    "status_idle":       {"vi": "Idle",        "en": "Idle",         "ko": "대기"},
    "status_online":     {"vi": "Online",      "en": "Online",       "ko": "온라인"},
    "status_amcl":       {"vi": "AMCL hoạt động","en": "AMCL active","ko": "AMCL 활성"},
    "mode_engineer":     {"vi": "Kỹ sư",       "en": "Engineer",     "ko": "엔지니어"},
    "mode_operator":     {"vi": "Vận hành",    "en": "Operator",     "ko": "운영자"},
    "battery":           {"vi": "Pin",         "en": "Battery",      "ko": "배터리"},

    # ── Home ───────────────────────────────────────────────────────
    "home_connection":   {"vi": "Kết nối",     "en": "Connection",   "ko": "연결"},
    "home_agv_status":   {"vi": "Trạng thái AGV","en": "AGV Status", "ko": "AGV 상태"},
    "home_moving":       {"vi": "Đang di chuyển","en": "Moving",     "ko": "이동 중"},
    "home_stopped":      {"vi": "Dừng",        "en": "Stopped",      "ko": "정지"},
    "home_conv":         {"vi": "Băng tải {}",  "en": "Conveyor {}",  "ko": "컨베이어 {}"},
    "home_has_cargo":    {"vi": "Có hàng",     "en": "Has cargo",    "ko": "화물 있음"},
    "home_no_cargo":     {"vi": "Không hàng",  "en": "No cargo",     "ko": "화물 없음"},
    # [NEW] Nút đổi người vận hành trên Home
    "home_switch_user":  {"vi": "Đổi người vận hành",
                          "en": "Switch operator",
                          "ko": "운영자 변경"},

    # ── Errors / Robot status ──────────────────────────────────────
    "no_errors":         {"vi": "Không có lỗi","en": "No errors",    "ko": "오류 없음"},
    "error_prefix":      {"vi": "Lỗi: ",       "en": "Error: ",      "ko": "오류: "},

    # ── Pose ───────────────────────────────────────────────────────
    "pose_robot":        {"vi": "VỊ TRÍ ROBOT","en": "ROBOT POSE",  "ko": "로봇 위치"},

    # ── Velocity ───────────────────────────────────────────────────
    "vel_title":         {"vi": "VẬN TỐC TỐI ĐA","en": "MAX VELOCITY","ko": "최대 속도"},
    "vel_linear":        {"vi": "Tuyến tính (m/s)","en": "Linear (m/s)","ko": "선형 (m/s)"},
    "vel_angular":       {"vi": "Góc quay (rad/s)","en": "Angular (rad/s)","ko": "각속도 (rad/s)"},
    "vel_apply":         {"vi": "Áp dụng",     "en": "Apply",        "ko": "적용"},
    "vel_info":          {"vi": "Giới hạn tốc độ tối đa khi AGV di chuyển.",
                          "en": "Limit maximum speed when AGV moves.",
                          "ko": "AGV 이동 시 최대 속도 제한."},

    # ── Mapping ────────────────────────────────────────────────────
    "mapping_start":     {"vi": "▶ Bắt đầu Mapping","en": "▶ Start Mapping","ko": "▶ 매핑 시작"},
    "mapping_stop":      {"vi": "⬛ Dừng Mapping","en": "⬛ Stop Mapping","ko": "⬛ 매핑 중지"},
    "mapping_save":      {"vi": "💾 Lưu Map",   "en": "💾 Save Map",  "ko": "💾 지도 저장"},
    "mapping_reset":     {"vi": "⊙ Reset view", "en": "⊙ Reset View","ko": "⊙ 뷰 리셋"},
    "mapping_actions":   {"vi": "HÀNH ĐỘNG",    "en": "ACTIONS",      "ko": "작업"},
    "map_saved":         {"vi": "Map đã lưu",   "en": "Map saved",    "ko": "지도 저장됨"},
    "map_save_fail":     {"vi": "Lưu thất bại!","en": "Save failed!", "ko": "저장 실패!"},

    # ── Navigation ─────────────────────────────────────────────────
    "nav_title":         {"vi": "ĐIỀU HƯỚNG",  "en": "NAVIGATION",   "ko": "내비게이션"},
    "nav_run":           {"vi": "▶ Chạy",      "en": "▶ Run",        "ko": "▶ 실행"},
    "nav_stop":          {"vi": "⬛ Stop",      "en": "⬛ Stop",      "ko": "⬛ 정지"},
    "nav_pause":         {"vi": "⏸ Pause",     "en": "⏸ Pause",     "ko": "⏸ 일시정지"},
    "nav_resume":        {"vi": "▶ Tiếp tục",  "en": "▶ Resume",     "ko": "▶ 재개"},
    "nav_start":         {"vi": "▶ Start",     "en": "▶ Start",      "ko": "▶ 시작"},
    "nav_waiting":       {"vi": "Chờ...",       "en": "Waiting...",   "ko": "대기 중..."},
    "nav_done":          {"vi": "✅ Hoàn thành!","en": "✅ Done!",    "ko": "✅ 완료!"},
    "nav_failed":        {"vi": "❌ Thất bại.", "en": "❌ Failed.",   "ko": "❌ 실패."},
    "nav_stopped":       {"vi": "⬛ Đã dừng.", "en": "⬛ Stopped.",  "ko": "⬛ 정지됨."},
    "nav_paused":        {"vi": "⏸ Tạm dừng tại {}.",
                          "en": "⏸ Paused at {}.",
                          "ko": "⏸ {}에서 일시정지."},
    "nav_going":         {"vi": "→ Đến {} ({:.2f}, {:.2f})",
                          "en": "→ Going to {} ({:.2f}, {:.2f})",
                          "ko": "→ {} ({:.2f}, {:.2f}) 이동 중"},
    "nav_remaining":     {"vi": "Goal {} — còn {:.2f} m",
                          "en": "Goal {} — {:.2f} m left",
                          "ko": "목표 {} — {:.2f} m 남음"},
    "nav_stopping":      {"vi": "⏳ Dừng {}s...",
                          "en": "⏳ Waiting {}s...",
                          "ko": "⏳ {}초 대기..."},
    "nav_confirm":       {"vi": "✔ Xác nhận & tiếp",
                          "en": "✔ Confirm & Continue",
                          "ko": "✔ 확인 및 계속"},
    "nav_pose_estimate": {"vi": "POSE ESTIMATE (AMCL)",
                          "en": "POSE ESTIMATE (AMCL)",
                          "ko": "위치 추정 (AMCL)"},
    "nav_pose_hint":     {"vi": "Click & kéo trên map để đặt vị trí robot.",
                          "en": "Click & drag on map to set robot pose.",
                          "ko": "지도에서 클릭 & 드래그하여 로봇 위치 설정."},
    "nav_pose_enable":   {"vi": "✛ Bật Pose Estimate",
                          "en": "✛ Enable Pose Estimate",
                          "ko": "✛ 위치 추정 활성화"},
    "nav_pose_active":   {"vi": "✛ Đang đặt — click map",
                          "en": "✛ Setting — click map",
                          "ko": "✛ 설정 중 — 지도 클릭"},

    # ── Waypoint / Route ───────────────────────────────────────────
    "route_name":        {"vi": "Tên lộ trình", "en": "Route Name",  "ko": "경로 이름"},
    "route_name_ph":     {"vi": "VD: Khu A → Kho B",
                          "en": "e.g. Zone A → Warehouse B",
                          "ko": "예: 구역 A → 창고 B"},
    "route_save":        {"vi": "💾 Lưu lộ trình","en": "💾 Save Route","ko": "💾 경로 저장"},
    "route_waypoints":   {"vi": "WAYPOINTS",    "en": "WAYPOINTS",    "ko": "경유지"},
    "route_wp_hint":     {"vi": "Click trái lên map để thêm điểm.",
                          "en": "Left-click map to add waypoint.",
                          "ko": "지도를 좌클릭하여 경유지 추가."},
    "route_wp_add_end":  {"vi": "⊕ Thêm điểm kết thúc",
                          "en": "⊕ Add End Point",
                          "ko": "⊕ 종료 지점 추가"},
    "route_wp_clear":    {"vi": "🗑 Xoá tất cả","en": "🗑 Clear All","ko": "🗑 모두 삭제"},
    "route_map_none":    {"vi": "Chưa chọn map","en": "No map selected","ko": "지도 미선택"},
    "route_select_map":  {"vi": "📂 Chọn map...","en": "📂 Select map...","ko": "📂 지도 선택..."},
    "route_no_routes":   {"vi": "Chưa có lộ trình.\nTạo trong Navigation.",
                          "en": "No routes yet.\nCreate in Navigation.",
                          "ko": "경로 없음.\n내비게이션에서 생성하세요."},
    "route_title":       {"vi": "LỘ TRÌNH",     "en": "ROUTES",       "ko": "경로"},
    "delete_route":      {"vi": "Xoá lộ trình",  "en": "Delete Route", "ko": "경로 삭제"},
    "delete_map":        {"vi": "Xoá map",        "en": "Delete Map",   "ko": "지도 삭제"},
    "route_run":         {"vi": "▶ Chạy",       "en": "▶ Run",        "ko": "▶ 실행"},
    "route_edit":        {"vi": "✏ Chỉnh sửa",  "en": "✏ Edit",       "ko": "✏ 편집"},

    # [NEW] Lặp lại lộ trình (repeat)
    "route_repeat_title":    {"vi": "LẶP LẠI LỘ TRÌNH", "en": "REPEAT ROUTE", "ko": "경로 반복"},
    "route_repeat_once":     {"vi": "Chạy 1 lần", "en": "Run once", "ko": "1회 실행"},
    "route_repeat_count":    {"vi": "Lặp lại N lần", "en": "Repeat N times", "ko": "N회 반복"},
    "route_repeat_infinite": {"vi": "Lặp vô hạn (đến khi Stop)",
                              "en": "Loop until stopped",
                              "ko": "정지할 때까지 반복"},
    "route_repeat_times":    {"vi": "Số lần lặp", "en": "Repeat count", "ko": "반복 횟수"},
    "route_repeat_progress": {"vi": "Chuyến {}/{}", "en": "Run {}/{}", "ko": "{}/{} 회차"},
    "route_repeat_progress_inf": {"vi": "Chuyến {} (vô hạn)",
                                  "en": "Run {} (infinite)",
                                  "ko": "{}회차 (무한)"},
    "route_repeat_waiting":  {"vi": "⏳ Nghỉ giữa các chuyến ({}s)...",
                              "en": "⏳ Pausing between runs ({}s)...",
                              "ko": "⏳ 회차 사이 대기 ({}초)..."},
    "route_busy_warning":    {"vi": "Đang chạy/lặp lộ trình. Vui lòng Stop trước khi chọn lộ trình khác.",
                              "en": "A route is running/repeating. Please Stop before selecting another route.",
                              "ko": "경로가 실행/반복 중입니다. 다른 경로를 선택하기 전에 정지하세요."},

    # ── Tasks ──────────────────────────────────────────────────────
    "task_title":        {"vi": "TASKS",        "en": "TASKS",        "ko": "작업"},
    "task_add":          {"vi": "+ Thêm task",  "en": "+ Add Task",   "ko": "+ 작업 추가"},
    "task_del":          {"vi": "Xoá task",     "en": "Delete Task",  "ko": "작업 삭제"},
    "task_conveyor":     {"vi": "Băng tải",     "en": "Conveyor",     "ko": "컨베이어"},
    "task_wait":         {"vi": "Dừng X giây",  "en": "Wait X sec",   "ko": "X초 대기"},
    "task_confirm":      {"vi": "Chờ xác nhận", "en": "Wait Confirm", "ko": "확인 대기"},
    "task_io":           {"vi": "Tín hiệu IO",  "en": "IO Signal",    "ko": "IO 신호"},
    "task_buzzer":       {"vi": "Phát còi",     "en": "Buzzer",       "ko": "버저"},
    "task_none":         {"vi": "Không có",     "en": "None",         "ko": "없음"},
    "task_order":        {"vi": "Thứ tự:",      "en": "Order:",       "ko": "순서:"},
    "task_parallel":     {"vi": "Song song",    "en": "Parallel",     "ko": "병렬"},
    "task_sequential":   {"vi": "Tuần tự",      "en": "Sequential",   "ko": "순차"},

    # ── Conveyor ───────────────────────────────────────────────────
    "conv_title":        {"vi": "BĂNG TẢI",     "en": "CONVEYORS",    "ko": "컨베이어"},
    "conv_belt":         {"vi": "Băng tải {}",  "en": "Conveyor {}",  "ko": "컨베이어 {}"},
    "conv_receive":      {"vi": "Nhận hàng",    "en": "Receive",      "ko": "수신"},
    "conv_send":         {"vi": "Chuyển hàng",  "en": "Transfer",     "ko": "전송"},
    "conv_speed":        {"vi": "Tốc độ (%)",   "en": "Speed (%)",    "ko": "속도 (%)"},
    "conv_duration":     {"vi": "Thời gian (s)","en": "Duration (s)", "ko": "시간 (초)"},
    "conv_start":        {"vi": "▶ Bắt đầu",   "en": "▶ Start",      "ko": "▶ 시작"},
    "conv_stop":         {"vi": "⬛ Dừng",      "en": "⬛ Stop",      "ko": "⬛ 정지"},
    "conv_all_stop":     {"vi": "⬛ Dừng tất cả băng tải",
                          "en": "⬛ Stop All Conveyors",
                          "ko": "⬛ 모든 컨베이어 정지"},
    "conv_idle":         {"vi": "Chờ",          "en": "Idle",         "ko": "대기"},
    "conv_running":      {"vi": "Đang chạy",    "en": "Running",      "ko": "실행 중"},
    "conv_status_idle":  {"vi": "Chờ",            "en": "Idle",         "ko": "대기"},
    "conv_status_run":   {"vi": "Đang chạy",       "en": "Running",      "ko": "실행 중"},
    "sensor_title":      {"vi": "CẢM BIẾN",     "en": "SENSORS",      "ko": "센서"},
    "sensor_n":          {"vi": "Sensor {}",    "en": "Sensor {}",    "ko": "센서 {}"},

    # ── Routes page ────────────────────────────────────────────────
    "routes_select":     {"vi": "Chọn lộ trình để chạy:",
                          "en": "Select route to run:",
                          "ko": "실행할 경로 선택:"},
    "bumper_title":      {"vi": "BUMPER SAFETY", "en": "BUMPER SAFETY","ko": "범퍼 안전"},
    "bumper_front":      {"vi": "Trước",        "en": "Front",        "ko": "전방"},
    "bumper_rear":       {"vi": "Sau",          "en": "Rear",         "ko": "후방"},
    "bumper_left":       {"vi": "Trái",         "en": "Left",         "ko": "좌측"},
    "bumper_right":      {"vi": "Phải",         "en": "Right",        "ko": "우측"},
    "bumper_ok":         {"vi": "OK",           "en": "OK",           "ko": "정상"},
    "bumper_triggered":  {"vi": "Kích hoạt!",   "en": "Triggered!",   "ko": "감지됨!"},
    "wp_status_pending": {"vi": "Chưa đến",     "en": "Pending",      "ko": "대기"},
    "wp_status_moving":  {"vi": "Đang đi",      "en": "Moving",       "ko": "이동 중"},
    "wp_status_task":    {"vi": "Đang làm task","en": "Running task", "ko": "작업 실행"},
    "wp_status_done":    {"vi": "Xong",         "en": "Done",         "ko": "완료"},

    # ── Map Library ────────────────────────────────────────────────
    "maplib_title":      {"vi": "THƯ VIỆN MAP", "en": "MAP LIBRARY",  "ko": "지도 라이브러리"},
    "maplib_refresh":    {"vi": "↺ Làm mới",    "en": "↺ Refresh",    "ko": "↺ 새로고침"},
    "maplib_use_nav":    {"vi": "⊳ Dùng trong Nav","en": "⊳ Use in Nav","ko": "⊳ 내비에서 사용"},
    "maplib_delete":     {"vi": "🗑 Xoá",       "en": "🗑 Delete",    "ko": "🗑 삭제"},
    "maplib_created":    {"vi": "Tạo:",         "en": "Created:",     "ko": "생성:"},

    # ── Settings ───────────────────────────────────────────────────
    "settings_title":    {"vi": "CÀI ĐẶT",      "en": "SETTINGS",     "ko": "설정"},
    "settings_pw_title": {"vi": "Đổi mật khẩu Kỹ sư",
                          "en": "Change Engineer Password",
                          "ko": "엔지니어 비밀번호 변경"},
    "settings_pw_old":   {"vi": "Mật khẩu hiện tại",
                          "en": "Current password",
                          "ko": "현재 비밀번호"},
    "settings_pw_new":   {"vi": "Mật khẩu mới", "en": "New password", "ko": "새 비밀번호"},
    "settings_pw_conf":  {"vi": "Xác nhận mật khẩu mới",
                          "en": "Confirm new password",
                          "ko": "새 비밀번호 확인"},
    "settings_pw_save":  {"vi": "💾 Lưu mật khẩu","en": "💾 Save Password","ko": "💾 비밀번호 저장"},
    "settings_pw_ok":    {"vi": "✅ Đã cập nhật mật khẩu!",
                          "en": "✅ Password updated!",
                          "ko": "✅ 비밀번호 업데이트됨!"},
    "settings_pw_wrong": {"vi": "Mật khẩu hiện tại sai!",
                          "en": "Current password is wrong!",
                          "ko": "현재 비밀번호가 틀렸습니다!"},
    "settings_pw_nomatch":{"vi": "Mật khẩu mới không khớp!",
                           "en": "New passwords don't match!",
                           "ko": "새 비밀번호가 일치하지 않습니다!"},
    "settings_future":   {"vi": "Tính năng sắp có...",
                          "en": "Coming soon...",
                          "ko": "곧 제공 예정..."},
    "settings_theme_title": {"vi": "GIAO DIỆN", "en": "APPEARANCE", "ko": "테마"},
    "settings_theme_hint":  {"vi": "Chọn chế độ sáng/tối cho ứng dụng. Áp dụng ngay không cần khởi động lại.",
                              "en": "Choose light/dark mode. Applies immediately, no restart needed.",
                              "ko": "밝게/어둡게 모드를 선택하세요. 재시작 없이 즉시 적용됩니다."},
    "theme_system": {"vi": "Hệ thống", "en": "System", "ko": "시스템"},
    "theme_light":  {"vi": "Sáng",     "en": "Light",  "ko": "밝게"},
    "theme_dark":   {"vi": "Tối",      "en": "Dark",   "ko": "어둡게"},

    # ── General ────────────────────────────────────────────────────
    "general_stop":      {"vi": "⬛ STOP",      "en": "⬛ STOP",      "ko": "⬛ 정지"},
    "confirm_yes":       {"vi": "Xác nhận",     "en": "Confirm",      "ko": "확인"},
    "confirm_no":        {"vi": "Huỷ",          "en": "Cancel",       "ko": "취소"},
    "lang_vi":           {"vi": "Tiếng Việt",   "en": "Tiếng Việt",   "ko": "Tiếng Việt"},
    "lang_en":           {"vi": "English",      "en": "English",      "ko": "English"},
    "lang_ko":           {"vi": "한국어",        "en": "한국어",        "ko": "한국어"},
}


def set_lang(lang: str):
    global _lang
    if lang in ("vi", "en", "ko"):
        _lang = lang


def get_lang() -> str:
    return _lang


def tr(key: str, *args) -> str:
    row = _S.get(key)
    if row is None:
        return key
    text = row.get(_lang, row.get("vi", key))
    if args:
        try:
            text = text.format(*args)
        except Exception:
            pass
    return text


# Language metadata
LANGUAGES = [
    {"code": "vi", "flag": "🇻🇳", "label": "Tiếng Việt"},
    {"code": "en", "flag": "🇬🇧", "label": "English"},
    {"code": "ko", "flag": "🇰🇷", "label": "한국어"},
]
