"""
i18n.py — 5 ngôn ngữ: Tiếng Việt / English / 한국어 / 日本語 / 简体中文

- Lưu ngôn ngữ vào ~/.agv_hmi/config.json.
- Tự khôi phục ngôn ngữ ở lần mở ứng dụng tiếp theo.
- Nếu một bản dịch bị thiếu, fallback sang English rồi Tiếng Việt.
"""
import json
import os

CONFIG_PATH = os.path.expanduser("~/.agv_hmi/config.json")

LANGUAGES = [
    {"code": "vi", "short": "VN", "flag": "🇻🇳", "label": "Tiếng Việt"},
    {"code": "en", "short": "EN", "flag": "🇬🇧", "label": "English"},
    {"code": "ko", "short": "KR", "flag": "🇰🇷", "label": "한국어"},
    {"code": "ja", "short": "JP", "flag": "🇯🇵", "label": "日本語"},
    {"code": "zh", "short": "CN", "flag": "🇨🇳", "label": "简体中文"},
]
SUPPORTED_LANGS = frozenset(item["code"] for item in LANGUAGES)


def _read_config() -> dict:
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _write_config(data: dict) -> None:
    try:
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        tmp_path = CONFIG_PATH + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp_path, CONFIG_PATH)
    except Exception as exc:
        print(f"[i18n] Không lưu được ngôn ngữ: {exc}")


def _load_saved_language() -> str:
    language = str(_read_config().get("language", "vi")).strip().lower()
    return language if language in SUPPORTED_LANGS else "vi"


_lang = _load_saved_language()

_S = {
    "route_del_confirm": {
        "vi": "Xoá lộ trình '{}'?",
        "en": "Delete route '{}'?",
        "ko": "경로 '{}'을(를) 삭제할까요?",
        "ja": "ルート「{}」を削除しますか？",
        "zh": "删除路线“{}”？"
    },
    "app_name": {
        "vi": "AMR Pacific Autonomous Robot",
        "en": "AMR Pacific Autonomous Robot",
        "ko": "AMR 부산 자율 로봇",
        "ja": "AMR Pacific 自律走行ロボット",
        "zh": "AMR Pacific 自主移动机器人"
    },
    "app_version": {
        "vi": "v4.3.0 · ROS2 Jazzy",
        "en": "v4.3.0 · ROS2 Jazzy",
        "ko": "v4.3.0 · ROS2 재지",
        "ja": "v4.3.0 · ROS2 Jazzy",
        "zh": "v4.3.0 · ROS2 Jazzy"
    },
    "login_select_mode": {
        "vi": "Chọn chế độ làm việc",
        "en": "Select Operating Mode",
        "ko": "운영 모드 선택",
        "ja": "作業モードを選択",
        "zh": "选择工作模式"
    },
    "login_operator": {
        "vi": "Người vận hành",
        "en": "Operator",
        "ko": "운영자",
        "ja": "オペレーター",
        "zh": "操作员"
    },
    "login_engineer": {
        "vi": "Kỹ sư",
        "en": "Engineer",
        "ko": "엔지니어",
        "ja": "エンジニア",
        "zh": "工程师"
    },
    "login_op_desc": {
        "vi": "Vận hành lộ trình, điều khiển băng tải",
        "en": "Run routes, control conveyors",
        "ko": "경로 운영, 컨베이어 제어",
        "ja": "ルート運転、コンベヤー制御",
        "zh": "运行路线、控制输送机"
    },
    "login_eng_desc": {
        "vi": "Toàn quyền: Mapping, cấu hình lộ trình",
        "en": "Full access: Mapping, route config",
        "ko": "전체 접근: 매핑, 경로 설정",
        "ja": "全機能：マッピング、ルート設定",
        "zh": "完整权限：建图、路线配置"
    },
    "login_password": {
        "vi": "Mật khẩu Kỹ sư",
        "en": "Engineer Password",
        "ko": "엔지니어 비밀번호",
        "ja": "エンジニアパスワード",
        "zh": "工程师密码"
    },
    "login_enter": {
        "vi": "Vào →",
        "en": "Enter →",
        "ko": "입력 →",
        "ja": "開始 →",
        "zh": "进入 →"
    },
    "login_wrong_pw": {
        "vi": "Sai mật khẩu!",
        "en": "Wrong password!",
        "ko": "비밀번호가 틀렸습니다!",
        "ja": "パスワードが違います！",
        "zh": "密码错误！"
    },
    "page_home": {
        "vi": "Home",
        "en": "Home",
        "ko": "홈",
        "ja": "ホーム",
        "zh": "主页"
    },
    "page_mapping": {
        "vi": "Dựng Maps",
        "en": "Mapping",
        "ko": "매핑",
        "ja": "マッピング",
        "zh": "建图"
    },
    "page_navigation": {
        "vi": "Điều hướng",
        "en": "Navigation",
        "ko": "내비게이션",
        "ja": "ナビゲーション",
        "zh": "导航"
    },
    "page_routes": {
        "vi": "Lộ trình",
        "en": "Routes",
        "ko": "경로",
        "ja": "ルート",
        "zh": "路线"
    },
    "page_conveyors": {
        "vi": "Băng tải",
        "en": "Conveyors",
        "ko": "컨베이어",
        "ja": "コンベヤー",
        "zh": "输送机"
    },
    "page_maplib": {
        "vi": "Thư viện Map",
        "en": "Map Library",
        "ko": "지도 라이브러리",
        "ja": "マップライブラリ",
        "zh": "地图库"
    },
    "page_settings": {
        "vi": "Cài đặt",
        "en": "Settings",
        "ko": "설정",
        "ja": "設定",
        "zh": "设置"
    },
    "status_offline": {
        "vi": "Offline",
        "en": "Offline",
        "ko": "오프라인",
        "ja": "オフライン",
        "zh": "离线"
    },
    "status_idle": {
        "vi": "Idle",
        "en": "Idle",
        "ko": "대기",
        "ja": "待機",
        "zh": "待机"
    },
    "status_online": {
        "vi": "Online",
        "en": "Online",
        "ko": "온라인",
        "ja": "オンライン",
        "zh": "在线"
    },
    "status_amcl": {
        "vi": "AMCL hoạt động",
        "en": "AMCL active",
        "ko": "AMCL 활성",
        "ja": "AMCL動作中",
        "zh": "AMCL 运行中"
    },
    "mode_engineer": {
        "vi": "Kỹ sư",
        "en": "Engineer",
        "ko": "엔지니어",
        "ja": "エンジニア",
        "zh": "工程师"
    },
    "mode_operator": {
        "vi": "Vận hành",
        "en": "Operator",
        "ko": "운영자",
        "ja": "オペレーター",
        "zh": "操作员"
    },
    "battery": {
        "vi": "Pin",
        "en": "Battery",
        "ko": "배터리",
        "ja": "バッテリー",
        "zh": "电池"
    },
    "home_connection": {
        "vi": "Kết nối",
        "en": "Connection",
        "ko": "연결",
        "ja": "接続",
        "zh": "连接"
    },
    "home_agv_status": {
        "vi": "Trạng thái AGV",
        "en": "AGV Status",
        "ko": "AGV 상태",
        "ja": "AGV状態",
        "zh": "AGV 状态"
    },
    "home_moving": {
        "vi": "Đang di chuyển",
        "en": "Moving",
        "ko": "이동 중",
        "ja": "移動中",
        "zh": "移动中"
    },
    "home_stopped": {
        "vi": "Dừng",
        "en": "Stopped",
        "ko": "정지",
        "ja": "停止",
        "zh": "已停止"
    },
    "home_conv": {
        "vi": "Băng tải {}",
        "en": "Conveyor {}",
        "ko": "컨베이어 {}",
        "ja": "コンベヤー{}",
        "zh": "输送机 {}"
    },
    "home_has_cargo": {
        "vi": "Có hàng",
        "en": "Has cargo",
        "ko": "화물 있음",
        "ja": "荷物あり",
        "zh": "有货物"
    },
    "home_no_cargo": {
        "vi": "Không hàng",
        "en": "No cargo",
        "ko": "화물 없음",
        "ja": "荷物なし",
        "zh": "无货物"
    },
    "home_switch_user": {
        "vi": "Đổi người vận hành",
        "en": "Switch operator",
        "ko": "운영자 변경",
        "ja": "オペレーター切替",
        "zh": "切换操作员"
    },
    "no_errors": {
        "vi": "Không có lỗi",
        "en": "No errors",
        "ko": "오류 없음",
        "ja": "エラーなし",
        "zh": "无错误"
    },
    "error_prefix": {
        "vi": "Lỗi: ",
        "en": "Error: ",
        "ko": "오류: ",
        "ja": "エラー：",
        "zh": "错误："
    },
    "pose_robot": {
        "vi": "VỊ TRÍ ROBOT",
        "en": "ROBOT POSE",
        "ko": "로봇 위치",
        "ja": "ロボット位置",
        "zh": "机器人位置"
    },
    "vel_title": {
        "vi": "VẬN TỐC TỐI ĐA",
        "en": "MAX VELOCITY",
        "ko": "최대 속도",
        "ja": "最大速度",
        "zh": "最大速度"
    },
    "vel_linear": {
        "vi": "Tuyến tính (m/s)",
        "en": "Linear (m/s)",
        "ko": "선형 (m/s)",
        "ja": "直進速度 (m/s)",
        "zh": "线速度 (m/s)"
    },
    "vel_angular": {
        "vi": "Góc quay (rad/s)",
        "en": "Angular (rad/s)",
        "ko": "각속도 (rad/s)",
        "ja": "角速度 (rad/s)",
        "zh": "角速度 (rad/s)"
    },
    "vel_apply": {
        "vi": "Áp dụng",
        "en": "Apply",
        "ko": "적용",
        "ja": "適用",
        "zh": "应用"
    },
    "vel_info": {
        "vi": "Giới hạn tốc độ tối đa khi AGV di chuyển.",
        "en": "Limit maximum speed when AGV moves.",
        "ko": "AGV 이동 시 최대 속도 제한.",
        "ja": "AGV移動時の最大速度を制限します。",
        "zh": "限制 AGV 移动时的最大速度。"
    },
    "mapping_start": {
        "vi": "▶ Bắt đầu Mapping",
        "en": "▶ Start Mapping",
        "ko": "▶ 매핑 시작",
        "ja": "▶ マッピング開始",
        "zh": "▶ 开始建图"
    },
    "mapping_stop": {
        "vi": "⬛ Dừng Mapping",
        "en": "⬛ Stop Mapping",
        "ko": "⬛ 매핑 중지",
        "ja": "⬛ マッピング停止",
        "zh": "⬛ 停止建图"
    },
    "mapping_save": {
        "vi": "💾 Lưu Map",
        "en": "💾 Save Map",
        "ko": "💾 지도 저장",
        "ja": "💾 マップ保存",
        "zh": "💾 保存地图"
    },
    "mapping_reset": {
        "vi": "⊙ Reset view",
        "en": "⊙ Reset View",
        "ko": "⊙ 뷰 리셋",
        "ja": "⊙ 表示をリセット",
        "zh": "⊙ 重置视图"
    },
    "mapping_actions": {
        "vi": "HÀNH ĐỘNG",
        "en": "ACTIONS",
        "ko": "작업",
        "ja": "操作",
        "zh": "操作"
    },
    "map_saved": {
        "vi": "Map đã lưu",
        "en": "Map saved",
        "ko": "지도 저장됨",
        "ja": "マップを保存しました",
        "zh": "地图已保存"
    },
    "map_save_fail": {
        "vi": "Lưu thất bại!",
        "en": "Save failed!",
        "ko": "저장 실패!",
        "ja": "保存に失敗しました！",
        "zh": "保存失败！"
    },
    "nav_title": {
        "vi": "ĐIỀU HƯỚNG",
        "en": "NAVIGATION",
        "ko": "내비게이션",
        "ja": "ナビゲーション",
        "zh": "导航"
    },
    "nav_run": {
        "vi": "▶ Chạy",
        "en": "▶ Run",
        "ko": "▶ 실행",
        "ja": "▶ 実行",
        "zh": "▶ 运行"
    },
    "nav_stop": {
        "vi": "⬛ Stop",
        "en": "⬛ Stop",
        "ko": "⬛ 정지",
        "ja": "⬛ 停止",
        "zh": "⬛ 停止"
    },
    "nav_pause": {
        "vi": "⏸ Pause",
        "en": "⏸ Pause",
        "ko": "⏸ 일시정지",
        "ja": "⏸ 一時停止",
        "zh": "⏸ 暂停"
    },
    "nav_resume": {
        "vi": "▶ Tiếp tục",
        "en": "▶ Resume",
        "ko": "▶ 재개",
        "ja": "▶ 再開",
        "zh": "▶ 继续"
    },
    "nav_start": {
        "vi": "▶ Start",
        "en": "▶ Start",
        "ko": "▶ 시작",
        "ja": "▶ 開始",
        "zh": "▶ 开始"
    },
    "nav_waiting": {
        "vi": "Chờ...",
        "en": "Waiting...",
        "ko": "대기 중...",
        "ja": "待機中...",
        "zh": "等待中..."
    },
    "nav_done": {
        "vi": "✅ Hoàn thành!",
        "en": "✅ Done!",
        "ko": "✅ 완료!",
        "ja": "✅ 完了！",
        "zh": "✅ 已完成！"
    },
    "nav_failed": {
        "vi": "❌ Thất bại.",
        "en": "❌ Failed.",
        "ko": "❌ 실패.",
        "ja": "❌ 失敗しました。",
        "zh": "❌ 失败。"
    },
    "nav_stopped": {
        "vi": "⬛ Đã dừng.",
        "en": "⬛ Stopped.",
        "ko": "⬛ 정지됨.",
        "ja": "⬛ 停止しました。",
        "zh": "⬛ 已停止。"
    },
    "nav_paused": {
        "vi": "⏸ Tạm dừng tại {}.",
        "en": "⏸ Paused at {}.",
        "ko": "⏸ {}에서 일시정지.",
        "ja": "⏸ {}で一時停止。",
        "zh": "⏸ 已在 {} 暂停。"
    },
    "nav_going": {
        "vi": "→ Đến {} ({:.2f}, {:.2f})",
        "en": "→ Going to {} ({:.2f}, {:.2f})",
        "ko": "→ {} ({:.2f}, {:.2f}) 이동 중",
        "ja": "→ {}へ移動 ({:.2f}, {:.2f})",
        "zh": "→ 前往 {} ({:.2f}, {:.2f})"
    },
    "nav_remaining": {
        "vi": "Goal {} — còn {:.2f} m",
        "en": "Goal {} — {:.2f} m left",
        "ko": "목표 {} — {:.2f} m 남음",
        "ja": "ゴール{} — 残り {:.2f} m",
        "zh": "目标 {} — 剩余 {:.2f} 米"
    },
    "nav_stopping": {
        "vi": "⏳ Dừng {}s...",
        "en": "⏳ Waiting {}s...",
        "ko": "⏳ {}초 대기...",
        "ja": "⏳ {}秒待機...",
        "zh": "⏳ 等待 {} 秒..."
    },
    "nav_confirm": {
        "vi": "✔ Xác nhận & tiếp",
        "en": "✔ Confirm & Continue",
        "ko": "✔ 확인 및 계속",
        "ja": "✔ 確認して続行",
        "zh": "✔ 确认并继续"
    },
    "nav_pose_estimate": {
        "vi": "POSE ESTIMATE (AMCL)",
        "en": "POSE ESTIMATE (AMCL)",
        "ko": "위치 추정 (AMCL)",
        "ja": "位置推定 (AMCL)",
        "zh": "位置估计 (AMCL)"
    },
    "nav_pose_hint": {
        "vi": "Click & kéo trên map để đặt vị trí robot.",
        "en": "Click & drag on map to set robot pose.",
        "ko": "지도에서 클릭 & 드래그하여 로봇 위치 설정.",
        "ja": "マップ上でクリック＆ドラッグしてロボット位置を設定します。",
        "zh": "在地图上点击并拖动以设置机器人位置。"
    },
    "nav_pose_enable": {
        "vi": "✛ Bật Pose Estimate",
        "en": "✛ Enable Pose Estimate",
        "ko": "✛ 위치 추정 활성화",
        "ja": "✛ 位置推定を有効化",
        "zh": "✛ 启用位置估计"
    },
    "nav_pose_active": {
        "vi": "✛ Đang đặt — click map",
        "en": "✛ Setting — click map",
        "ko": "✛ 설정 중 — 지도 클릭",
        "ja": "✛ 設定中 — マップをクリック",
        "zh": "✛ 设置中 — 点击地图"
    },
    "route_name": {
        "vi": "Tên lộ trình",
        "en": "Route Name",
        "ko": "경로 이름",
        "ja": "ルート名",
        "zh": "路线名称"
    },
    "route_name_ph": {
        "vi": "VD: Khu A → Kho B",
        "en": "e.g. Zone A → Warehouse B",
        "ko": "예: 구역 A → 창고 B",
        "ja": "例：エリアA → 倉庫B",
        "zh": "例如：A 区 → B 仓库"
    },
    "route_save": {
        "vi": "💾 Lưu lộ trình",
        "en": "💾 Save Route",
        "ko": "💾 경로 저장",
        "ja": "💾 ルート保存",
        "zh": "💾 保存路线"
    },
    "route_waypoints": {
        "vi": "WAYPOINTS",
        "en": "WAYPOINTS",
        "ko": "경유지",
        "ja": "ウェイポイント",
        "zh": "航点"
    },
    "route_wp_hint": {
        "vi": "Click trái lên map để thêm điểm.",
        "en": "Left-click map to add waypoint.",
        "ko": "지도를 좌클릭하여 경유지 추가.",
        "ja": "マップを左クリックしてポイントを追加します。",
        "zh": "左键点击地图添加航点。"
    },
    "route_wp_add_end": {
        "vi": "⊕ Thêm điểm kết thúc",
        "en": "⊕ Add End Point",
        "ko": "⊕ 종료 지점 추가",
        "ja": "⊕ 終点を追加",
        "zh": "⊕ 添加终点"
    },
    "route_wp_clear": {
        "vi": "🗑 Xoá tất cả",
        "en": "🗑 Clear All",
        "ko": "🗑 모두 삭제",
        "ja": "🗑 すべて削除",
        "zh": "🗑 全部清除"
    },
    "route_map_none": {
        "vi": "Chưa chọn map",
        "en": "No map selected",
        "ko": "지도 미선택",
        "ja": "マップ未選択",
        "zh": "未选择地图"
    },
    "route_select_map": {
        "vi": "📂 Chọn map...",
        "en": "📂 Select map...",
        "ko": "📂 지도 선택...",
        "ja": "📂 マップを選択...",
        "zh": "📂 选择地图..."
    },
    "route_no_routes": {
        "vi": "Chưa có lộ trình.\nTạo trong Navigation.",
        "en": "No routes yet.\nCreate in Navigation.",
        "ko": "경로 없음.\n내비게이션에서 생성하세요.",
        "ja": "ルートがありません。\nナビゲーションで作成してください。",
        "zh": "暂无路线。\n请在导航页面创建。"
    },
    "route_title": {
        "vi": "LỘ TRÌNH",
        "en": "ROUTES",
        "ko": "경로",
        "ja": "ルート",
        "zh": "路线"
    },
    "delete_route": {
        "vi": "Xoá lộ trình",
        "en": "Delete Route",
        "ko": "경로 삭제",
        "ja": "ルート削除",
        "zh": "删除路线"
    },
    "delete_map": {
        "vi": "Xoá map",
        "en": "Delete Map",
        "ko": "지도 삭제",
        "ja": "マップ削除",
        "zh": "删除地图"
    },
    "route_run": {
        "vi": "▶ Chạy",
        "en": "▶ Run",
        "ko": "▶ 실행",
        "ja": "▶ 実行",
        "zh": "▶ 运行"
    },
    "route_edit": {
        "vi": "✏ Chỉnh sửa",
        "en": "✏ Edit",
        "ko": "✏ 편집",
        "ja": "✏ 編集",
        "zh": "✏ 编辑"
    },
    "route_repeat_title": {
        "vi": "LẶP LẠI LỘ TRÌNH",
        "en": "REPEAT ROUTE",
        "ko": "경로 반복",
        "ja": "ルート繰り返し",
        "zh": "重复路线"
    },
    "route_repeat_once": {
        "vi": "Chạy 1 lần",
        "en": "Run once",
        "ko": "1회 실행",
        "ja": "1回実行",
        "zh": "运行一次"
    },
    "route_repeat_count": {
        "vi": "Lặp lại N lần",
        "en": "Repeat N times",
        "ko": "N회 반복",
        "ja": "N回繰り返す",
        "zh": "重复 N 次"
    },
    "route_repeat_infinite": {
        "vi": "Lặp vô hạn (đến khi Stop)",
        "en": "Loop until stopped",
        "ko": "정지할 때까지 반복",
        "ja": "停止するまで繰り返す",
        "zh": "循环运行直到停止"
    },
    "route_repeat_times": {
        "vi": "Số lần lặp",
        "en": "Repeat count",
        "ko": "반복 횟수",
        "ja": "繰り返し回数",
        "zh": "重复次数"
    },
    "route_repeat_progress": {
        "vi": "Chuyến {}/{}",
        "en": "Run {}/{}",
        "ko": "{}/{} 회차",
        "ja": "{}/{} 回目",
        "zh": "第 {}/{} 次"
    },
    "route_repeat_progress_inf": {
        "vi": "Chuyến {} (vô hạn)",
        "en": "Run {} (infinite)",
        "ko": "{}회차 (무한)",
        "ja": "{} 回目（無限）",
        "zh": "第 {} 次（无限）"
    },
    "route_repeat_waiting": {
        "vi": "⏳ Nghỉ giữa các chuyến ({}s)...",
        "en": "⏳ Pausing between runs ({}s)...",
        "ko": "⏳ 회차 사이 대기 ({}초)...",
        "ja": "⏳ 実行間の待機（{}秒）...",
        "zh": "⏳ 两次运行之间等待（{}秒）..."
    },
    "route_busy_warning": {
        "vi": "Đang chạy/lặp lộ trình. Vui lòng Stop trước khi chọn lộ trình khác.",
        "en": "A route is running/repeating. Please Stop before selecting another route.",
        "ko": "경로가 실행/반복 중입니다. 다른 경로를 선택하기 전에 정지하세요.",
        "ja": "ルートを実行／繰り返し中です。別のルートを選ぶ前に停止してください。",
        "zh": "路线正在运行或循环。选择其他路线前请先停止。"
    },
    "mode_title": {
        "vi": "CHẾ ĐỘ CHẠY",
        "en": "RUN MODE",
        "ko": "실행 모드",
        "ja": "実行モード",
        "zh": "运行模式"
    },
    "mode_auto": {
        "vi": "⚙ Auto",
        "en": "⚙ Auto",
        "ko": "⚙ 자동",
        "ja": "⚙ 自動",
        "zh": "⚙ 自动"
    },
    "mode_manual": {
        "vi": "🖐 Manual",
        "en": "🖐 Manual",
        "ko": "🖐 수동",
        "ja": "🖐 手動",
        "zh": "🖐 手动"
    },
    "manual_step_move": {
        "vi": "▶ Đến {}",
        "en": "▶ Go to {}",
        "ko": "▶ {}(으)로 이동",
        "ja": "▶ {}へ移動",
        "zh": "▶ 前往 {}"
    },
    "manual_step_task": {
        "vi": "▶ Thực hiện Task {}",
        "en": "▶ Run task at {}",
        "ko": "▶ {} 작업 실행",
        "ja": "▶ {}のタスクを実行",
        "zh": "▶ 执行 {} 的任务"
    },
    "manual_step_confirm": {
        "vi": "✔ Xác nhận & đi tiếp {}",
        "en": "✔ Confirm & continue {}",
        "ko": "✔ 확인 후 {} 계속",
        "ja": "✔ 確認して{}へ続行",
        "zh": "✔ 确认并继续前往 {}"
    },
    "manual_step_finish": {
        "vi": "✅ Hoàn tất",
        "en": "✅ Finish",
        "ko": "✅ 완료",
        "ja": "✅ 完了",
        "zh": "✅ 完成"
    },
    "stuck_title": {
        "vi": "⚠ Lộ trình gặp sự cố tại WP {}",
        "en": "⚠ Route stuck at WP {}",
        "ko": "⚠ WP {}에서 경로 문제 발생",
        "ja": "⚠ WP {}でルート異常",
        "zh": "⚠ 路线在航点 {} 发生故障"
    },
    "stuck_retry": {
        "vi": "🔁 Thử lại/Tiếp tục",
        "en": "🔁 Retry/Continue",
        "ko": "🔁 재시도/계속",
        "ja": "🔁 再試行／続行",
        "zh": "🔁 重试／继续"
    },
    "stuck_skip": {
        "vi": "⏭ Bỏ qua điểm này",
        "en": "⏭ Skip this point",
        "ko": "⏭ 이 지점 건너뛰기",
        "ja": "⏭ このポイントをスキップ",
        "zh": "⏭ 跳过此航点"
    },
    "stuck_cancel": {
        "vi": "✕ Huỷ toàn bộ",
        "en": "✕ Cancel all",
        "ko": "✕ 전체 취소",
        "ja": "✕ すべて中止",
        "zh": "✕ 全部取消"
    },
    "task_title": {
        "vi": "TASKS",
        "en": "TASKS",
        "ko": "작업",
        "ja": "タスク",
        "zh": "任务"
    },
    "task_add": {
        "vi": "+ Thêm task",
        "en": "+ Add Task",
        "ko": "+ 작업 추가",
        "ja": "+ タスク追加",
        "zh": "+ 添加任务"
    },
    "task_del": {
        "vi": "Xoá task",
        "en": "Delete Task",
        "ko": "작업 삭제",
        "ja": "タスク削除",
        "zh": "删除任务"
    },
    "task_conveyor": {
        "vi": "Băng tải",
        "en": "Conveyor",
        "ko": "컨베이어",
        "ja": "コンベヤー",
        "zh": "输送机"
    },
    "task_wait": {
        "vi": "Dừng X giây",
        "en": "Wait X sec",
        "ko": "X초 대기",
        "ja": "X秒待機",
        "zh": "等待 X 秒"
    },
    "task_confirm": {
        "vi": "Chờ xác nhận",
        "en": "Wait Confirm",
        "ko": "확인 대기",
        "ja": "確認待ち",
        "zh": "等待确认"
    },
    "task_io": {
        "vi": "Tín hiệu IO",
        "en": "IO Signal",
        "ko": "IO 신호",
        "ja": "IO信号",
        "zh": "IO 信号"
    },
    "task_buzzer": {
        "vi": "Phát còi",
        "en": "Buzzer",
        "ko": "버저",
        "ja": "ブザー",
        "zh": "蜂鸣器"
    },
    "task_none": {
        "vi": "Không có",
        "en": "None",
        "ko": "없음",
        "ja": "なし",
        "zh": "无"
    },
    "task_order": {
        "vi": "Thứ tự:",
        "en": "Order:",
        "ko": "순서:",
        "ja": "順序：",
        "zh": "顺序："
    },
    "task_parallel": {
        "vi": "Song song",
        "en": "Parallel",
        "ko": "병렬",
        "ja": "並列",
        "zh": "并行"
    },
    "task_sequential": {
        "vi": "Tuần tự",
        "en": "Sequential",
        "ko": "순차",
        "ja": "順次",
        "zh": "顺序执行"
    },
    "conv_title": {
        "vi": "BĂNG TẢI",
        "en": "CONVEYORS",
        "ko": "컨베이어",
        "ja": "コンベヤー",
        "zh": "输送机"
    },
    "conv_belt": {
        "vi": "Băng tải {}",
        "en": "Conveyor {}",
        "ko": "컨베이어 {}",
        "ja": "コンベヤー{}",
        "zh": "输送机 {}"
    },
    "conv_receive": {
        "vi": "Nhận hàng",
        "en": "Receive",
        "ko": "수신",
        "ja": "受入",
        "zh": "接收货物"
    },
    "conv_send": {
        "vi": "Chuyển hàng",
        "en": "Transfer",
        "ko": "전송",
        "ja": "搬送",
        "zh": "转送货物"
    },
    "conv_speed": {
        "vi": "Tốc độ (%)",
        "en": "Speed (%)",
        "ko": "속도 (%)",
        "ja": "速度 (%)",
        "zh": "速度 (%)"
    },
    "conv_duration": {
        "vi": "Thời gian (s)",
        "en": "Duration (s)",
        "ko": "시간 (초)",
        "ja": "時間 (秒)",
        "zh": "时间 (秒)"
    },
    "conv_start": {
        "vi": "▶ Bắt đầu",
        "en": "▶ Start",
        "ko": "▶ 시작",
        "ja": "▶ 開始",
        "zh": "▶ 开始"
    },
    "conv_stop": {
        "vi": "⬛ Dừng",
        "en": "⬛ Stop",
        "ko": "⬛ 정지",
        "ja": "⬛ 停止",
        "zh": "⬛ 停止"
    },
    "conv_all_stop": {
        "vi": "⬛ Dừng tất cả băng tải",
        "en": "⬛ Stop All Conveyors",
        "ko": "⬛ 모든 컨베이어 정지",
        "ja": "⬛ 全コンベヤー停止",
        "zh": "⬛ 停止所有输送机"
    },
    "conv_idle": {
        "vi": "Chờ",
        "en": "Idle",
        "ko": "대기",
        "ja": "待機",
        "zh": "待机"
    },
    "conv_running": {
        "vi": "Đang chạy",
        "en": "Running",
        "ko": "실행 중",
        "ja": "運転中",
        "zh": "运行中"
    },
    "conv_status_idle": {
        "vi": "Chờ",
        "en": "Idle",
        "ko": "대기",
        "ja": "待機",
        "zh": "待机"
    },
    "conv_status_run": {
        "vi": "Đang chạy",
        "en": "Running",
        "ko": "실행 중",
        "ja": "運転中",
        "zh": "运行中"
    },
    "sensor_title": {
        "vi": "CẢM BIẾN",
        "en": "SENSORS",
        "ko": "센서",
        "ja": "センサー",
        "zh": "传感器"
    },
    "sensor_n": {
        "vi": "Sensor {}",
        "en": "Sensor {}",
        "ko": "센서 {}",
        "ja": "センサー{}",
        "zh": "传感器 {}"
    },
    "routes_select": {
        "vi": "Chọn lộ trình để chạy:",
        "en": "Select route to run:",
        "ko": "실행할 경로 선택:",
        "ja": "実行するルートを選択：",
        "zh": "选择要运行的路线："
    },
    "bumper_title": {
        "vi": "BUMPER SAFETY",
        "en": "BUMPER SAFETY",
        "ko": "범퍼 안전",
        "ja": "バンパー安全",
        "zh": "防撞安全"
    },
    "bumper_front": {
        "vi": "Trước",
        "en": "Front",
        "ko": "전방",
        "ja": "前",
        "zh": "前"
    },
    "bumper_rear": {
        "vi": "Sau",
        "en": "Rear",
        "ko": "후방",
        "ja": "後",
        "zh": "后"
    },
    "bumper_left": {
        "vi": "Trái",
        "en": "Left",
        "ko": "좌측",
        "ja": "左",
        "zh": "左"
    },
    "bumper_right": {
        "vi": "Phải",
        "en": "Right",
        "ko": "우측",
        "ja": "右",
        "zh": "右"
    },
    "bumper_ok": {
        "vi": "OK",
        "en": "OK",
        "ko": "정상",
        "ja": "正常",
        "zh": "正常"
    },
    "bumper_triggered": {
        "vi": "Kích hoạt!",
        "en": "Triggered!",
        "ko": "감지됨!",
        "ja": "作動！",
        "zh": "已触发！"
    },
    "wp_status_pending": {
        "vi": "Chưa đến",
        "en": "Pending",
        "ko": "대기",
        "ja": "未到着",
        "zh": "未到达"
    },
    "wp_status_moving": {
        "vi": "Đang đi",
        "en": "Moving",
        "ko": "이동 중",
        "ja": "移動中",
        "zh": "移动中"
    },
    "wp_status_task": {
        "vi": "Đang làm task",
        "en": "Running task",
        "ko": "작업 실행",
        "ja": "タスク実行中",
        "zh": "执行任务中"
    },
    "wp_status_done": {
        "vi": "Xong",
        "en": "Done",
        "ko": "완료",
        "ja": "完了",
        "zh": "完成"
    },
    "maplib_title": {
        "vi": "THƯ VIỆN MAP",
        "en": "MAP LIBRARY",
        "ko": "지도 라이브러리",
        "ja": "マップライブラリ",
        "zh": "地图库"
    },
    "maplib_refresh": {
        "vi": "↺ Làm mới",
        "en": "↺ Refresh",
        "ko": "↺ 새로고침",
        "ja": "↺ 更新",
        "zh": "↺ 刷新"
    },
    "maplib_use_nav": {
        "vi": "⊳ Dùng trong Nav",
        "en": "⊳ Use in Nav",
        "ko": "⊳ 내비에서 사용",
        "ja": "⊳ ナビで使用",
        "zh": "⊳ 用于导航"
    },
    "maplib_delete": {
        "vi": "🗑 Xoá",
        "en": "🗑 Delete",
        "ko": "🗑 삭제",
        "ja": "🗑 削除",
        "zh": "🗑 删除"
    },
    "maplib_created": {
        "vi": "Tạo:",
        "en": "Created:",
        "ko": "생성:",
        "ja": "作成：",
        "zh": "创建时间："
    },
    "settings_title": {
        "vi": "CÀI ĐẶT",
        "en": "SETTINGS",
        "ko": "설정",
        "ja": "設定",
        "zh": "设置"
    },
    "settings_pw_title": {
        "vi": "Đổi mật khẩu Kỹ sư",
        "en": "Change Engineer Password",
        "ko": "엔지니어 비밀번호 변경",
        "ja": "エンジニアパスワード変更",
        "zh": "修改工程师密码"
    },
    "settings_pw_old": {
        "vi": "Mật khẩu hiện tại",
        "en": "Current password",
        "ko": "현재 비밀번호",
        "ja": "現在のパスワード",
        "zh": "当前密码"
    },
    "settings_pw_new": {
        "vi": "Mật khẩu mới",
        "en": "New password",
        "ko": "새 비밀번호",
        "ja": "新しいパスワード",
        "zh": "新密码"
    },
    "settings_pw_conf": {
        "vi": "Xác nhận mật khẩu mới",
        "en": "Confirm new password",
        "ko": "새 비밀번호 확인",
        "ja": "新しいパスワード（確認）",
        "zh": "确认新密码"
    },
    "settings_pw_save": {
        "vi": "💾 Lưu mật khẩu",
        "en": "💾 Save Password",
        "ko": "💾 비밀번호 저장",
        "ja": "💾 パスワード保存",
        "zh": "💾 保存密码"
    },
    "settings_pw_ok": {
        "vi": "✅ Đã cập nhật mật khẩu!",
        "en": "✅ Password updated!",
        "ko": "✅ 비밀번호 업데이트됨!",
        "ja": "✅ パスワードを更新しました！",
        "zh": "✅ 密码已更新！"
    },
    "settings_pw_wrong": {
        "vi": "Mật khẩu hiện tại sai!",
        "en": "Current password is wrong!",
        "ko": "현재 비밀번호가 틀렸습니다!",
        "ja": "現在のパスワードが違います！",
        "zh": "当前密码错误！"
    },
    "settings_pw_nomatch": {
        "vi": "Mật khẩu mới không khớp!",
        "en": "New passwords don't match!",
        "ko": "새 비밀번호가 일치하지 않습니다!",
        "ja": "新しいパスワードが一致しません！",
        "zh": "两次输入的新密码不一致！"
    },
    "settings_future": {
        "vi": "Tính năng sắp có...",
        "en": "Coming soon...",
        "ko": "곧 제공 예정...",
        "ja": "今後の機能...",
        "zh": "更多功能即将推出..."
    },
    "settings_theme_title": {
        "vi": "GIAO DIỆN",
        "en": "APPEARANCE",
        "ko": "테마",
        "ja": "外観",
        "zh": "外观"
    },
    "settings_theme_hint": {
        "vi": "Chọn chế độ sáng/tối cho ứng dụng. Áp dụng ngay không cần khởi động lại.",
        "en": "Choose light/dark mode. Applies immediately, no restart needed.",
        "ko": "밝게/어둡게 모드를 선택하세요. 재시작 없이 즉시 적용됩니다.",
        "ja": "ライト／ダークモードを選択します。再起動せずにすぐ適用されます。",
        "zh": "选择亮色或暗色模式，无需重启即可立即应用。"
    },
    "theme_system": {
        "vi": "Hệ thống",
        "en": "System",
        "ko": "시스템",
        "ja": "システム",
        "zh": "跟随系统"
    },
    "theme_light": {
        "vi": "Sáng",
        "en": "Light",
        "ko": "밝게",
        "ja": "ライト",
        "zh": "亮色"
    },
    "theme_dark": {
        "vi": "Tối",
        "en": "Dark",
        "ko": "어둡게",
        "ja": "ダーク",
        "zh": "暗色"
    },
    "general_stop": {
        "vi": "⬛ STOP",
        "en": "⬛ STOP",
        "ko": "⬛ 정지",
        "ja": "⬛ 停止",
        "zh": "⬛ 停止"
    },
    "confirm_yes": {
        "vi": "Xác nhận",
        "en": "Confirm",
        "ko": "확인",
        "ja": "確認",
        "zh": "确认"
    },
    "confirm_no": {
        "vi": "Huỷ",
        "en": "Cancel",
        "ko": "취소",
        "ja": "キャンセル",
        "zh": "取消"
    },
    "lang_vi": {
        "vi": "Tiếng Việt",
        "en": "Tiếng Việt",
        "ko": "Tiếng Việt",
        "ja": "Tiếng Việt",
        "zh": "Tiếng Việt"
    },
    "lang_en": {
        "vi": "English",
        "en": "English",
        "ko": "English",
        "ja": "English",
        "zh": "English"
    },
    "lang_ko": {
        "vi": "한국어",
        "en": "한국어",
        "ko": "한국어",
        "ja": "한국어",
        "zh": "한국어"
    },
    "login_language": {
        "vi": "Ngôn ngữ",
        "en": "Language",
        "ko": "언어",
        "ja": "言語",
        "zh": "语言"
    },
    "settings_language_title": {
        "vi": "NGÔN NGỮ",
        "en": "LANGUAGE",
        "ko": "언어",
        "ja": "言語",
        "zh": "语言"
    },
    "settings_language_hint": {
        "vi": "Chọn ngôn ngữ hiển thị. Thay đổi được áp dụng ngay và lưu cho lần mở app tiếp theo.",
        "en": "Choose the display language. Changes apply immediately and are saved for the next launch.",
        "ko": "표시 언어를 선택하세요. 변경 사항은 즉시 적용되며 다음 실행에도 저장됩니다.",
        "ja": "表示言語を選択します。変更はすぐに適用され、次回起動時にも保持されます。",
        "zh": "选择显示语言。更改会立即生效，并保存到下次启动。"
    },
    "settings_pw_empty": {
        "vi": "Mật khẩu không được để trống!",
        "en": "Password cannot be empty!",
        "ko": "비밀번호는 비워 둘 수 없습니다!",
        "ja": "パスワードを空にすることはできません！",
        "zh": "密码不能为空！"
    },
    "lang_ja": {
        "vi": "日本語",
        "en": "日本語",
        "ko": "日本語",
        "ja": "日本語",
        "zh": "日本語"
    },
    "lang_zh": {
        "vi": "简体中文",
        "en": "简体中文",
        "ko": "简体中文",
        "ja": "简体中文",
        "zh": "简体中文"
    },

    "page_fleet": {
        "vi": "Theo dõi AGV", "en": "Fleet Monitor", "ko": "AGV 모니터링",
        "ja": "AGV監視", "zh": "AGV监控"
    },
    "fleet_title": {
        "vi": "THEO DÕI ĐỘI AGV", "en": "AGV FLEET MONITOR", "ko": "AGV 플릿 모니터",
        "ja": "AGVフリート監視", "zh": "AGV车队监控"
    },
    "fleet_map_filter": {
        "vi": "Map:", "en": "Map:", "ko": "지도:", "ja": "マップ:", "zh": "地图:"
    },
    "fleet_all_maps": {
        "vi": "Tất cả map", "en": "All maps", "ko": "모든 지도", "ja": "すべてのマップ", "zh": "所有地图"
    },
    "fleet_total": {
        "vi": "Tổng AGV", "en": "Total AGVs", "ko": "전체 AGV", "ja": "AGV総数", "zh": "AGV总数"
    },
    "fleet_online": {
        "vi": "Online", "en": "Online", "ko": "온라인", "ja": "オンライン", "zh": "在线"
    },
    "fleet_moving": {
        "vi": "Đang chạy", "en": "Moving", "ko": "이동 중", "ja": "走行中", "zh": "运行中"
    },
    "fleet_errors": {
        "vi": "Có lỗi", "en": "Errors", "ko": "오류", "ja": "エラー", "zh": "错误"
    },
    "fleet_robot_list": {
        "vi": "DANH SÁCH AGV", "en": "AGV LIST", "ko": "AGV 목록", "ja": "AGV一覧", "zh": "AGV列表"
    },
    "fleet_shared_map": {
        "vi": "MAP DÙNG CHUNG", "en": "SHARED MAP", "ko": "공유 지도", "ja": "共有マップ", "zh": "共享地图"
    },
    "fleet_robot_detail": {
        "vi": "CHI TIẾT AGV", "en": "AGV DETAILS", "ko": "AGV 상세", "ja": "AGV詳細", "zh": "AGV详情"
    },
    "fleet_col_robot": {
        "vi": "AGV", "en": "AGV", "ko": "AGV", "ja": "AGV", "zh": "AGV"
    },
    "fleet_col_status": {
        "vi": "Trạng thái", "en": "Status", "ko": "상태", "ja": "状態", "zh": "状态"
    },
    "fleet_col_map": {
        "vi": "Map", "en": "Map", "ko": "지도", "ja": "マップ", "zh": "地图"
    },
    "fleet_col_battery": {
        "vi": "Pin", "en": "Battery", "ko": "배터리", "ja": "バッテリー", "zh": "电池"
    },
    "fleet_col_mission": {
        "vi": "Nhiệm vụ", "en": "Mission", "ko": "미션", "ja": "ミッション", "zh": "任务"
    },
    "fleet_col_waypoint": {
        "vi": "WP", "en": "WP", "ko": "WP", "ja": "WP", "zh": "WP"
    },
    "fleet_col_error": {
        "vi": "Lỗi", "en": "Error", "ko": "오류", "ja": "エラー", "zh": "错误"
    },
    "fleet_detail_id": {
        "vi": "Robot ID", "en": "Robot ID", "ko": "로봇 ID", "ja": "ロボットID", "zh": "机器人ID"
    },
    "fleet_detail_name": {
        "vi": "Tên", "en": "Name", "ko": "이름", "ja": "名前", "zh": "名称"
    },
    "fleet_detail_map": {
        "vi": "Map", "en": "Map", "ko": "지도", "ja": "マップ", "zh": "地图"
    },
    "fleet_detail_position": {
        "vi": "Vị trí", "en": "Position", "ko": "위치", "ja": "位置", "zh": "位置"
    },
    "fleet_detail_battery": {
        "vi": "Pin", "en": "Battery", "ko": "배터리", "ja": "バッテリー", "zh": "电池"
    },
    "fleet_detail_connection": {
        "vi": "Kết nối", "en": "Connection", "ko": "연결", "ja": "接続", "zh": "连接"
    },
    "fleet_detail_nav": {
        "vi": "Nav2", "en": "Nav2", "ko": "Nav2", "ja": "Nav2", "zh": "Nav2"
    },
    "fleet_detail_mission": {
        "vi": "Nhiệm vụ", "en": "Mission", "ko": "미션", "ja": "ミッション", "zh": "任务"
    },
    "fleet_detail_waypoint": {
        "vi": "Waypoint", "en": "Waypoint", "ko": "웨이포인트", "ja": "ウェイポイント", "zh": "航点"
    },
    "fleet_detail_error": {
        "vi": "Lỗi", "en": "Error", "ko": "오류", "ja": "エラー", "zh": "错误"
    },
    "fleet_detail_last_seen": {
        "vi": "Nhận dữ liệu cách đây", "en": "Last seen", "ko": "마지막 수신", "ja": "最終受信", "zh": "最后接收"
    }

}

def set_lang(lang: str, persist: bool = True) -> bool:
    """Đổi ngôn ngữ. Trả về True khi mã ngôn ngữ hợp lệ."""
    global _lang
    code = str(lang or "").strip().lower()
    if code not in SUPPORTED_LANGS:
        return False

    _lang = code
    if persist:
        cfg = _read_config()
        cfg["language"] = code
        _write_config(cfg)
    return True


def get_lang() -> str:
    return _lang


def tr(key: str, *args) -> str:
    row = _S.get(key)
    if row is None:
        return key

    text = row.get(_lang) or row.get("en") or row.get("vi") or key
    if args:
        try:
            text = text.format(*args)
        except (IndexError, KeyError, ValueError):
            pass
    return text
