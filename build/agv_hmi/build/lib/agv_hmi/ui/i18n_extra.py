"""i18n_extra.py - strings for the 4.5.0 features, merged into i18n._S.

Columns: vi, en, ko, ja, zh.  Use `tr("key")` as usual.
"""
from agv_hmi.ui.i18n import _S


def _add(key, vi, en, ko=None, ja=None, zh=None):
    _S[key] = {"vi": vi, "en": en, "ko": ko or en, "ja": ja or en, "zh": zh or en}


# ── Map cleaning ────────────────────────────────────────────────────────
_add("mc_title", "Làm sạch map (tường thẳng)", "Clean map (straight walls)",
     "맵 정리 (직선 벽)", "マップ整形（直線の壁）", "地图清理（直墙）")
_add("mc_noise", "Lọc nhiễu (m)", "Noise filter (m)", "노이즈 필터 (m)", "ノイズ除去 (m)", "噪点过滤 (m)")
_add("mc_minwall", "Tường tối thiểu (m)", "Min wall length (m)", "최소 벽 길이 (m)", "最小壁長 (m)", "最短墙长 (m)")
_add("mc_gap", "Lấp khe hở (m)", "Bridge gaps (m)", "틈 메우기 (m)", "隙間補完 (m)", "补缝 (m)")
_add("mc_snap", "Bám góc vuông (°)", "Snap angle (°)", "각도 정렬 (°)", "角度スナップ (°)", "角度吸附 (°)")
_add("mc_thick", "Độ dày tường (cm)", "Wall thickness (cm)", "벽 두께 (cm)", "壁の厚さ (cm)", "墙厚 (cm)")
_add("mc_run", "✨ Làm sạch map", "✨ Clean map", "✨ 맵 정리", "✨ マップ整形", "✨ 清理地图")
_add("mc_raw", "↩ Xem map gốc", "↩ Show raw map", "↩ 원본 보기", "↩ 元のマップ", "↩ 显示原图")
_add("mc_save", "💾 Lưu map sạch", "💾 Save clean map", "💾 정리된 맵 저장", "💾 整形マップ保存", "💾 保存清理地图")
_add("mc_empty", "Chưa có map để làm sạch.", "No map to clean yet.", "정리할 맵이 없습니다.", "整形するマップがありません。", "暂无可清理的地图。")
_add("mc_working", "Đang xử lý…", "Working…", "처리 중…", "処理中…", "处理中…")
_add("mc_done", "Xong: {} tường, bỏ {} ô nhiễu", "Done: {} walls, {} noise cells removed",
     "완료: 벽 {}개, 노이즈 {}셀 제거", "完了: 壁{}本、ノイズ{}セル除去", "完成：{} 面墙，去除 {} 个噪点")
_add("mc_saved", "Đã lưu map sạch:\n{}", "Clean map saved:\n{}", "저장됨:\n{}", "保存しました:\n{}", "已保存:\n{}")
_add("mc_hint_frozen", "Đang xem map đã làm sạch (cập nhật SLAM tạm dừng)",
     "Viewing the cleaned map (live SLAM updates paused)",
     "정리된 맵 표시 중 (SLAM 갱신 일시 중지)", "整形済みマップ表示中（SLAM更新は一時停止）", "正在查看清理后的地图（SLAM 更新已暂停）")
# ── Map view tools ──────────────────────────────────────────────────────
_add("mv_follow", "Bám theo robot", "Follow robot", "로봇 따라가기", "ロボット追従", "跟随机器人")
_add("mv_grid", "Lưới 1 m", "1 m grid", "1 m 격자", "1 mグリッド", "1 米网格")
_add("mv_minimap", "Bản đồ nhỏ", "Minimap", "미니맵", "ミニマップ", "小地图")
_add("mv_measure", "📏 Đo khoảng cách", "📏 Measure", "📏 거리 측정", "📏 距離測定", "📏 测距")

# ── Alarm center ────────────────────────────────────────────────────────
_add("page_alarms", "Cảnh báo", "Alarms", "경보", "アラーム", "报警")
_add("al_time", "Thời gian", "Time", "시간", "時刻", "时间")
_add("al_level", "Mức", "Level", "수준", "レベル", "级别")
_add("al_source", "Nguồn", "Source", "출처", "ソース", "来源")
_add("al_message", "Nội dung", "Message", "내용", "内容", "内容")
_add("al_state", "Trạng thái", "State", "상태", "状態", "状态")
_add("al_active", "ĐANG BẬT", "ACTIVE", "활성", "発生中", "进行中")
_add("al_cleared", "Đã hết", "Cleared", "해제됨", "解除", "已恢复")
_add("al_ack_all", "✔ Xác nhận tất cả", "✔ Acknowledge all", "✔ 모두 확인", "✔ すべて確認", "✔ 全部确认")
_add("al_clear", "🧹 Xoá lịch sử", "🧹 Clear history", "🧹 기록 삭제", "🧹 履歴消去", "🧹 清除历史")
_add("al_only_active", "Chỉ hiện đang bật", "Active only", "활성만", "発生中のみ", "仅显示进行中")
_add("al_none", "Không có cảnh báo.", "No alarms.", "경보 없음.", "アラームなし。", "无报警。")
_add("lvl_info", "Thông tin", "Info", "정보", "情報", "信息")
_add("lvl_warn", "Cảnh báo", "Warning", "경고", "警告", "警告")
_add("lvl_error", "Lỗi", "Error", "오류", "エラー", "错误")
_add("lvl_critical", "NGHIÊM TRỌNG", "CRITICAL", "치명적", "重大", "严重")
_add("al_plc_offline", "PLC Mega2560 mất kết nối", "Mega2560 PLC offline", "PLC 연결 끊김", "PLC 切断", "PLC 离线")
_add("al_estop", "Dừng khẩn cấp đang nhấn", "Emergency stop pressed", "비상정지 작동", "非常停止 作動", "急停已按下")
_add("al_driver_offline", "Driver KEYA không phản hồi", "KEYA driver not responding", "KEYA 드라이버 무응답", "KEYAドライバー応答なし", "KEYA 驱动器无响应")
_add("al_driver_fault", "Driver KEYA báo lỗi {}", "KEYA driver fault {}", "KEYA 오류 {}", "KEYA異常 {}", "KEYA 故障 {}")
_add("al_lidar", "Mất dữ liệu lidar (/scan)", "No lidar data (/scan)", "라이다 데이터 없음", "LiDARデータなし", "无激光雷达数据")
_add("al_loc", "Định vị kém (σ={:.2f} m) — có thể lạc, hãy đặt lại pose", "Poor localization (σ={:.2f} m) — set pose again",
     "위치 추정 불량 (σ={:.2f} m)", "自己位置が不安定 (σ={:.2f} m)", "定位较差 (σ={:.2f} m)")
_add("al_battery", "Pin yếu: {}%", "Low battery: {}%", "배터리 부족: {}%", "電池残量低下: {}%", "电量低: {}%")
_add("al_bumper", "Bumper {} bị chạm", "{} bumper triggered", "{} 범퍼 접촉", "{}バンパー接触", "{} 保险杠触发")
_add("al_ros", "Mất kết nối ROS", "ROS connection lost", "ROS 연결 끊김", "ROS接続断", "ROS 连接丢失")
_add("al_nav2", "Nav2: {}", "Nav2: {}", "Nav2: {}", "Nav2: {}", "Nav2: {}")
_add("side_left", "trái", "left", "왼쪽", "左", "左")
_add("side_right", "phải", "right", "오른쪽", "右", "右")

# ── Soft e-stop / banner ────────────────────────────────────────────────
_add("estop_btn", "⛔ DỪNG", "⛔ STOP", "⛔ 정지", "⛔ 停止", "⛔ 停止")
_add("estop_tip", "Dừng mềm: vận tốc 0, tắt motor, huỷ nhiệm vụ. Không thay thế E-stop phần cứng.",
     "Soft stop: zero velocity, motors off, mission cancelled. Not a replacement for the hardware e-stop.",
     "소프트 정지", "ソフト停止", "软停止")
_add("estop_banner", "⛔ DỪNG KHẨN CẤP — nhả nút E-stop rồi bấm \"Bật motor\" để chạy lại",
     "⛔ EMERGENCY STOP — release the e-stop, then press \"Enable motors\" to resume",
     "⛔ 비상정지 — 해제 후 \"모터 켜기\"", "⛔ 非常停止 — 解除後「モーター有効」", "⛔ 紧急停止 — 释放后点击“启用电机”")
_add("estop_soft_done", "Đã dừng mềm: motor tắt, nhiệm vụ huỷ", "Soft stop done: motors off, mission cancelled",
     "소프트 정지 완료", "ソフト停止しました", "已软停止")
_add("estop_mission_cancelled", "E-stop: nhiệm vụ đang chạy đã bị huỷ", "E-stop: running mission cancelled",
     "비상정지: 임무 취소", "非常停止: ミッション中止", "急停：任务已取消")

# ── Pre-flight checklist ────────────────────────────────────────────────
_add("pf_title", "Kiểm tra trước khi chạy", "Pre-flight check", "운행 전 점검", "走行前チェック", "运行前检查")
_add("pf_ros", "Kết nối ROS", "ROS connection", "ROS 연결", "ROS接続", "ROS 连接")
_add("pf_lidar", "Lidar có dữ liệu", "Lidar streaming", "라이다 수신", "LiDAR受信", "激光雷达数据")
_add("pf_estop", "PLC online, E-stop đã nhả", "PLC online, e-stop released", "PLC 온라인, 비상정지 해제", "PLC接続・非常停止解除", "PLC 在线，急停已释放")
_add("pf_driver", "Driver KEYA online, không lỗi", "KEYA driver online, no fault", "KEYA 정상", "KEYA正常", "KEYA 正常")
_add("pf_loc", "Định vị (AMCL) tốt", "Localization (AMCL) healthy", "위치 추정 양호", "自己位置OK", "定位良好")
_add("pf_battery", "Pin đủ", "Battery OK", "배터리 충분", "電池OK", "电量充足")
_add("pf_run_anyway", "Vẫn chạy (Engineer)", "Run anyway (Engineer)", "그래도 실행 (Engineer)", "強制実行 (Engineer)", "仍然运行 (Engineer)")
_add("pf_recheck", "Kiểm tra lại", "Re-check", "다시 확인", "再チェック", "重新检查")
_add("pf_cancel", "Huỷ", "Cancel", "취소", "キャンセル", "取消")
_add("pf_start", "▶ Bắt đầu", "▶ Start", "▶ 시작", "▶ 開始", "▶ 开始")
_add("pf_blocked", "Chưa đủ điều kiện chạy. Xử lý các mục đỏ rồi kiểm tra lại.",
     "Not ready to run. Fix the red items and re-check.", "실행 조건 미충족", "実行条件を満たしていません", "条件不足，请处理红色项目后重试")
_add("pf_ready", "Sẵn sàng chạy.", "Ready to run.", "준비 완료.", "準備完了。", "已就绪。")

# ── Diagnostics ─────────────────────────────────────────────────────────
_add("page_diag", "Chẩn đoán", "Diagnostics", "진단", "診断", "诊断")
_add("dg_tab_topics", "Topic & thiết bị", "Topics & devices", "토픽/장치", "トピック/機器", "话题与设备")
_add("dg_tab_plots", "Biểu đồ", "Live plots", "실시간 그래프", "リアルタイムグラフ", "实时曲线")
_add("dg_tab_logs", "Nhật ký ROS", "ROS log", "ROS 로그", "ROSログ", "ROS 日志")
_add("dg_tab_bag", "Ghi rosbag", "rosbag", "rosbag", "rosbag", "rosbag")
_add("dg_tab_hw", "Phần cứng", "Hardware", "하드웨어", "ハードウェア", "硬件")
_add("dg_topic", "Topic", "Topic", "토픽", "トピック", "话题")
_add("dg_rate", "Tần số", "Rate", "주기", "周波数", "频率")
_add("dg_status", "Trạng thái", "Status", "상태", "状態", "状态")
_add("dg_ok", "OK", "OK", "정상", "OK", "正常")
_add("dg_slow", "Chậm", "Slow", "느림", "低速", "偏慢")
_add("dg_none", "Không có dữ liệu", "No data", "데이터 없음", "データなし", "无数据")
_add("dg_devices", "Thiết bị", "Devices", "장치", "機器", "设备")
_add("dg_plot_lin", "Vận tốc dài (m/s): lệnh vs thực", "Linear velocity (m/s): command vs actual", "선속도 (m/s)", "並進速度 (m/s)", "线速度 (m/s)")
_add("dg_plot_ang", "Vận tốc góc (rad/s): lệnh vs thực", "Angular velocity (rad/s): command vs actual", "각속도 (rad/s)", "角速度 (rad/s)", "角速度 (rad/s)")
_add("dg_plot_rpm", "Tốc độ bánh (rpm) trái/phải", "Wheel speed (rpm) left/right", "휠 속도 (rpm)", "車輪速度 (rpm)", "轮速 (rpm)")
_add("dg_plot_volt", "Điện áp driver (V)", "Driver voltage (V)", "드라이버 전압 (V)", "ドライバー電圧 (V)", "驱动器电压 (V)")
_add("dg_cmd", "lệnh", "command", "명령", "指令", "指令")
_add("dg_actual", "thực", "actual", "실제", "実測", "实际")
_add("dg_pause", "⏸ Tạm dừng", "⏸ Pause", "⏸ 일시정지", "⏸ 一時停止", "⏸ 暂停")
_add("dg_resume", "▶ Tiếp tục", "▶ Resume", "▶ 재개", "▶ 再開", "▶ 继续")
_add("dg_level", "Mức tối thiểu", "Min level", "최소 수준", "最小レベル", "最低级别")
_add("dg_search", "Tìm…", "Search…", "검색…", "検索…", "搜索…")
_add("dg_export", "💾 Xuất file", "💾 Export", "💾 내보내기", "💾 エクスポート", "💾 导出")
_add("dg_clear", "🧹 Xoá", "🧹 Clear", "🧹 지우기", "🧹 クリア", "🧹 清除")
_add("bag_start", "⏺ Bắt đầu ghi", "⏺ Start recording", "⏺ 녹화 시작", "⏺ 記録開始", "⏺ 开始录制")
_add("bag_stop", "⏹ Dừng ghi", "⏹ Stop recording", "⏹ 녹화 중지", "⏹ 記録停止", "⏹ 停止录制")
_add("bag_auto", "Tự ghi 60 giây khi có lỗi nghiêm trọng", "Auto-record 60 s on critical alarm",
     "치명적 경보 시 60초 자동 녹화", "重大アラームで60秒自動記録", "严重报警时自动录制 60 秒")
_add("bag_running", "Đang ghi: {}", "Recording: {}", "녹화 중: {}", "記録中: {}", "录制中: {}")
_add("bag_idle", "Không ghi", "Not recording", "녹화 안 함", "記録していません", "未录制")
_add("bag_saved", "Đã lưu: {}", "Saved: {}", "저장됨: {}", "保存: {}", "已保存: {}")
_add("bag_no_ros2", "Không chạy được `ros2 bag`: {}", "Cannot run `ros2 bag`: {}", "ros2 bag 실행 불가: {}", "ros2 bag を実行できません: {}", "无法运行 ros2 bag: {}")
_add("hw_profile", "Profile", "Profile", "프로필", "プロファイル", "配置")
_add("hw_model", "model — ESP32 + L298N (mô hình)", "model — ESP32 + L298N (test model)", "model — ESP32 + L298N", "model — ESP32 + L298N", "model — ESP32 + L298N")
_add("hw_real", "real — KEYA + Mega2560 PLC (robot thật)", "real — KEYA + Mega2560 PLC (real robot)", "real — KEYA + Mega2560", "real — KEYA + Mega2560", "real — KEYA + Mega2560")
_add("hw_plc", "Bridge PLC", "PLC bridge", "PLC 브리지", "PLCブリッジ", "PLC 桥接")
_add("hw_plc_auto", "Tự động theo profile", "Auto (by profile)", "자동", "自動", "自动")
_add("hw_plc_on", "Luôn bật", "Always on", "항상 켜기", "常にオン", "始终开启")
_add("hw_plc_off", "Tắt", "Off", "끄기", "オフ", "关闭")
_add("hw_start", "▶ Khởi động phần cứng", "▶ Start hardware", "▶ 하드웨어 시작", "▶ ハードウェア起動", "▶ 启动硬件")
_add("hw_stop", "⏹ Dừng phần cứng", "⏹ Stop hardware", "⏹ 하드웨어 중지", "⏹ ハードウェア停止", "⏹ 停止硬件")
_add("hw_running", "Đang chạy: {}", "Running: {}", "실행 중: {}", "実行中: {}", "运行中: {}")
_add("hw_stopped", "Đã dừng", "Stopped", "중지됨", "停止中", "已停止")
_add("hw_hint", "Chạy `ros2 launch hiep_robot2 agv_hardware.launch.py` trong nền. Log của các bridge xem ở tab Nhật ký ROS.",
     "Runs `ros2 launch hiep_robot2 agv_hardware.launch.py` in the background. Bridge logs appear in the ROS log tab.",
     "백그라운드에서 hiep_robot2 launch 실행", "バックグラウンドで起動", "后台运行 launch")

# overrides
_add("home_motor_enable", "Giữ 1 giây để bật motor", "Hold 1 s to enable motors",
     "1초간 눌러 모터 켜기", "1秒長押しでモーター有効", "按住 1 秒启用电机")

# ── Dashboard / reports / stations / schedule / settings ──────────────────
_add("dash_speed", "⚡ Tốc độ", "⚡ Speed", "⚡ 속도", "⚡ 速度", "⚡ 速度")
_add("dash_dist", "📏 Quãng đường (phiên)", "📏 Distance (session)", "📏 거리", "📏 距離", "📏 里程")
_add("dash_move", "⏱ Thời gian chạy", "⏱ Moving time", "⏱ 주행 시간", "⏱ 走行時間", "⏱ 行驶时间")
_add("dash_batt", "🔋 Pin", "🔋 Battery", "🔋 배터리", "🔋 電池", "🔋 电量")
_add("dash_missions", "🧭 Nhiệm vụ hôm nay", "🧭 Missions today", "🧭 오늘 임무", "🧭 本日のミッション", "🧭 今日任务")
_add("dash_cargo", "📦 Hàng hôm nay", "📦 Cargo today", "📦 오늘 화물", "📦 本日の荷物", "📦 今日货物")
_add("dash_alarms", "🔔 Cảnh báo đang bật", "🔔 Active alarms", "🔔 활성 경보", "🔔 発生中アラーム", "🔔 进行中报警")

# ── Reports / audit ──────────────────────────────────────────────────────
_add("page_reports", "Báo cáo", "Reports", "보고서", "レポート", "报表")
_add("rp_tab_stats", "Thống kê nhiệm vụ", "Mission statistics", "임무 통계", "ミッション統計", "任务统计")
_add("rp_tab_audit", "Nhật ký thao tác", "Audit log", "작업 기록", "操作ログ", "操作日志")
_add("rp_day", "Ngày", "Day", "날짜", "日付", "日期")
_add("rp_total", "Nhiệm vụ", "Missions", "임무", "ミッション", "任务")
_add("rp_ok", "Thành công", "Success", "성공", "成功", "成功")
_add("rp_fail", "Lỗi", "Failed", "실패", "失敗", "失败")
_add("rp_cancel", "Huỷ", "Cancelled", "취소", "中止", "取消")
_add("rp_avg", "TB (giây)", "Avg (s)", "평균(초)", "平均(秒)", "平均(秒)")
_add("rp_cargo", "Hàng", "Cargo", "화물", "荷物", "货物")
_add("rp_chart", "Số nhiệm vụ mỗi ngày (14 ngày gần nhất)", "Missions per day (last 14 days)", "일별 임무 (14일)", "日別ミッション (14日)", "每日任务 (近14天)")
_add("rp_export", "💾 Xuất CSV", "💾 Export CSV", "💾 CSV 내보내기", "💾 CSV出力", "💾 导出 CSV")
_add("rp_refresh", "↻ Làm mới", "↻ Refresh", "↻ 새로고침", "↻ 更新", "↻ 刷新")
_add("au_time", "Thời gian", "Time", "시간", "時刻", "时间")
_add("au_user", "Vai trò", "Role", "역할", "役割", "角色")
_add("au_action", "Thao tác", "Action", "작업", "操作", "操作")
_add("au_detail", "Chi tiết", "Detail", "상세", "詳細", "详情")
# ── Stations ─────────────────────────────────────────────────────────────
_add("page_stations", "Trạm", "Stations", "스테이션", "ステーション", "站点")
_add("st_name", "Tên", "Name", "이름", "名前", "名称")
_add("st_kind", "Loại", "Type", "유형", "種類", "类型")
_add("st_kind_normal", "Thường", "Normal", "일반", "通常", "普通")
_add("st_kind_charge", "Sạc", "Charge", "충전", "充電", "充电")
_add("st_kind_home", "Chờ (home)", "Home", "홈", "ホーム", "待命")
_add("st_add_here", "➕ Lưu vị trí robot làm trạm", "➕ Save robot position as station", "➕ 현재 위치 저장", "➕ 現在位置を保存", "➕ 保存当前位置")
_add("st_delete", "🗑 Xoá", "🗑 Delete", "🗑 삭제", "🗑 削除", "🗑 删除")
_add("st_set_pose", "✛ Đặt pose robot tại trạm", "✛ Set robot pose here", "✛ 로봇 위치 지정", "✛ ロボット位置設定", "✛ 设置机器人位姿")
_add("st_goto", "▶ Đi tới trạm", "▶ Go to station", "▶ 이동", "▶ 移動", "▶ 前往")
_add("st_goto_charge", "🔋 Về trạm sạc", "🔋 Go to charging station", "🔋 충전소로", "🔋 充電ステーションへ", "🔋 前往充电站")
_add("st_prompt_name", "Tên trạm:", "Station name:", "스테이션 이름:", "ステーション名:", "站点名称:")
_add("st_no_pose", "Chưa có vị trí robot (cần AMCL).", "No robot pose yet (AMCL needed).", "로봇 위치 없음", "ロボット位置なし", "尚无机器人位置")
_add("st_no_charge", "Chưa có trạm loại Sạc.", "No charging station defined.", "충전소 없음", "充電ステーションなし", "未定义充电站")
_add("st_pose_hint", "(x, y theo hệ toạ độ map)", "(x, y in map frame)", "(맵 좌표)", "(マップ座標)", "(地图坐标)")
# ── Schedule / queue ─────────────────────────────────────────────────────
_add("page_schedule", "Lịch chạy", "Schedule", "스케줄", "スケジュール", "排程")
_add("sc_jobs", "Lịch chạy theo giờ", "Timed jobs", "예약 작업", "予約ジョブ", "定时任务")
_add("sc_queue", "Hàng đợi", "Queue", "대기열", "キュー", "队列")
_add("sc_time", "Giờ", "Time", "시간", "時刻", "时间")
_add("sc_days", "Ngày", "Days", "요일", "曜日", "星期")
_add("sc_route", "Route", "Route", "경로", "ルート", "路线")
_add("sc_on", "Bật", "On", "켜짐", "有効", "启用")
_add("sc_add", "➕ Thêm lịch", "➕ Add job", "➕ 추가", "➕ 追加", "➕ 添加")
_add("sc_remove", "🗑 Xoá", "🗑 Remove", "🗑 삭제", "🗑 削除", "🗑 删除")
_add("sc_q_add", "➕ Thêm vào hàng đợi", "➕ Add to queue", "➕ 대기열에 추가", "➕ キューに追加", "➕ 加入队列")
_add("sc_q_run", "▶ Chạy hàng đợi", "▶ Run queue", "▶ 대기열 실행", "▶ キュー実行", "▶ 运行队列")
_add("sc_q_clear", "🧹 Xoá hàng đợi", "🧹 Clear queue", "🧹 대기열 비우기", "🧹 キュー消去", "🧹 清空队列")
_add("sc_auto", "Cho phép TỰ CHẠY theo lịch (không có người bấm Start)", "Allow AUTO-START from schedule (nobody presses Start)",
     "스케줄 자동 시작 허용", "スケジュール自動開始を許可", "允许按排程自动启动")
_add("sc_warn", "⚠ Robot sẽ tự di chuyển. Chỉ bật khi khu vực chạy an toàn. Chỉ chạy khi qua kiểm tra trước khi chạy và không có E-stop.",
     "⚠ The robot will move on its own. Enable only if the area is safe. Runs only when the pre-flight check passes and no e-stop is active.",
     "⚠ 로봇이 자동으로 움직입니다.", "⚠ ロボットが自動で動きます。", "⚠ 机器人将自动移动。")
_add("sc_blocked", "Lịch chạy bị chặn: chưa qua kiểm tra trước khi chạy", "Scheduled run blocked: pre-flight check failed",
     "예약 실행 차단", "予約実行がブロックされました", "排程被阻止：未通过检查")
_add("sc_started", "Bắt đầu route theo lịch/hàng đợi: {}", "Starting route from schedule/queue: {}", "시작: {}", "開始: {}", "开始: {}")
_add("sc_no_routes", "Chưa có route nào.", "No routes yet.", "경로 없음", "ルートなし", "暂无路线")
_add("day_0", "T2", "Mon", "월", "月", "一"); _add("day_1", "T3", "Tue", "화", "火", "二")
_add("day_2", "T4", "Wed", "수", "水", "三"); _add("day_3", "T5", "Thu", "목", "木", "四")
_add("day_4", "T6", "Fri", "금", "金", "五"); _add("day_5", "T7", "Sat", "토", "土", "六")
_add("day_6", "CN", "Sun", "일", "日", "日")
# ── System settings ──────────────────────────────────────────────────────
_add("ss_title", "Hệ thống HMI", "HMI system", "HMI 시스템", "HMIシステム", "HMI 系统")
_add("ss_preflight", "Kiểm tra trước khi chạy nhiệm vụ", "Pre-flight check before missions", "임무 전 점검", "ミッション前チェック", "任务前检查")
_add("ss_sound", "Âm báo khi có lỗi", "Beep on errors", "오류 시 경고음", "エラー時ビープ", "出错时蜂鸣")
_add("ss_tower", "Điều khiển đèn tháp PLC theo trạng thái", "Drive PLC tower lights from robot state", "PLC 타워 램프 제어", "PLCタワーランプ制御", "PLC 信号灯控制")
_add("ss_tower_masks", "Mặt nạ đèn (bit0..3 = trái1, trái2, phải1, phải2): rảnh / chạy / tạm dừng / lỗi",
     "Light masks (bit0..3 = left1, left2, right1, right2): idle / running / paused / error", "램프 마스크", "ランプマスク", "灯光掩码")
_add("ss_joy", "Tay cầm (/joy): giữ nút an toàn để lái", "Gamepad (/joy): hold the dead-man button to drive", "게임패드 (/joy)", "ゲームパッド (/joy)", "手柄 (/joy)")
_add("ss_joy_btn", "Nút an toàn", "Dead-man button", "데드맨 버튼", "デッドマンボタン", "安全键")
_add("ss_joy_lin", "Trục tiến/lùi", "Linear axis", "직진 축", "前後軸", "前后轴")
_add("ss_joy_ang", "Trục quay", "Angular axis", "회전 축", "旋回軸", "转向轴")
_add("ss_joy_vmax", "Tốc độ tối đa (m/s)", "Max speed (m/s)", "최대 속도 (m/s)", "最大速度 (m/s)", "最大速度 (m/s)")
_add("ss_joy_wmax", "Quay tối đa (rad/s)", "Max turn (rad/s)", "최대 회전 (rad/s)", "最大旋回 (rad/s)", "最大转速 (rad/s)")
_add("ss_touch", "Chế độ cảm ứng (nút to) + toàn màn hình", "Touch mode (big buttons) + fullscreen", "터치 모드 + 전체화면", "タッチモード＋全画面", "触控模式 + 全屏")
_add("ss_backup", "💾 Sao lưu (map, route, cấu hình)", "💾 Back up (maps, routes, settings)", "💾 백업", "💾 バックアップ", "💾 备份")
_add("ss_restore", "♻ Khôi phục từ file sao lưu", "♻ Restore from backup", "♻ 복원", "♻ 復元", "♻ 恢复")
_add("ss_backup_done", "Đã sao lưu {} file:\n{}", "Backed up {} files:\n{}", "{}개 파일 백업:\n{}", "{}ファイルをバックアップ:\n{}", "已备份 {} 个文件:\n{}")
_add("ss_restore_confirm", "Khôi phục sẽ GHI ĐÈ map, route và cấu hình hiện có bằng file sao lưu. Tiếp tục?",
     "Restoring will OVERWRITE current maps, routes and settings with the backup. Continue?", "복원하면 덮어씁니다. 계속?", "上書きされます。続行しますか？", "恢复将覆盖现有数据，是否继续？")
_add("ss_restore_done", "Đã khôi phục {} file. Khởi động lại HMI để áp dụng đầy đủ.", "Restored {} files. Restart the HMI to apply everything.",
     "{}개 파일 복원. HMI를 재시작하세요.", "{}ファイルを復元。HMIを再起動してください。", "已恢复 {} 个文件，请重启 HMI。")
# ── Zones ────────────────────────────────────────────────────────────────
_add("zn_title", "Vùng cấm / vùng chậm", "Keep-out / slow zones", "금지/감속 구역", "進入禁止/減速ゾーン", "禁区/减速区")
_add("zn_keepout", "⛔ Vẽ vùng cấm", "⛔ Draw keep-out", "⛔ 금지 구역 그리기", "⛔ 禁止ゾーン描画", "⛔ 绘制禁区")
_add("zn_slow", "🐢 Vẽ vùng chậm", "🐢 Draw slow zone", "🐢 감속 구역 그리기", "🐢 減速ゾーン描画", "🐢 绘制减速区")
_add("zn_undo", "↶ Xoá vùng cuối", "↶ Remove last zone", "↶ 마지막 구역 삭제", "↶ 最後のゾーンを削除", "↶ 删除最后一个区域")
_add("zn_hint", "Click để thêm đỉnh, double-click để đóng vùng. Vùng được lưu cùng map sạch; vùng cấm xuất thêm file *_keepout.pgm/.yaml cho Nav2.",
     "Click to add vertices, double-click to close. Zones are saved with the clean map; keep-out zones also export *_keepout.pgm/.yaml for Nav2.",
     "클릭: 꼭짓점, 더블클릭: 닫기", "クリックで頂点、ダブルクリックで閉じる", "单击添加顶点，双击闭合")
_add("zn_exported", "Đã xuất mặt nạ vùng cấm:\n{}", "Keep-out mask exported:\n{}", "내보냄:\n{}", "出力:\n{}", "已导出:\n{}")

_add("st_busy", "Đang có nhiệm vụ chạy, hãy dừng trước.", "A mission is running, stop it first.", "임무 실행 중", "ミッション実行中", "任务运行中")

# ── Robot profile & safety zones ──────────────────────────────────────────
_add("page_safety", "Robot & An toàn", "Robot & Safety", "로봇 & 안전", "ロボット＆安全", "机器人与安全")
_add("sf_profile", "Hồ sơ robot", "Robot profile", "로봇 프로필", "ロボットプロファイル", "机器人配置")
_add("sf_new", "➕ Mới", "➕ New", "➕ 새로", "➕ 新規", "➕ 新建")
_add("sf_dup", "⧉ Nhân bản", "⧉ Duplicate", "⧉ 복제", "⧉ 複製", "⧉ 复制")
_add("sf_del", "🗑 Xoá", "🗑 Delete", "🗑 삭제", "🗑 削除", "🗑 删除")
_add("sf_prompt_name", "Tên hồ sơ robot:", "Robot profile name:", "프로필 이름:", "プロファイル名:", "配置名称:")
_add("sf_robot_box", "Kích thước robot", "Robot size", "로봇 크기", "ロボットサイズ", "机器人尺寸")
_add("sf_name", "Tên", "Name", "이름", "名前", "名称")
_add("sf_length", "Dài (m)", "Length (m)", "길이 (m)", "長さ (m)", "长 (m)")
_add("sf_width", "Rộng (m)", "Width (m)", "폭 (m)", "幅 (m)", "宽 (m)")
_add("sf_cx", "Tâm thân so với base_link: x (m)", "Body centre vs base_link: x (m)", "차체 중심 x (m)", "車体中心 x (m)", "车体中心 x (m)")
_add("sf_cy", "Tâm thân so với base_link: y (m)", "Body centre vs base_link: y (m)", "차체 중심 y (m)", "車体中心 y (m)", "车体中心 y (m)")
_add("sf_lidar_box", "Lidar (vị trí so với base_link)", "Lidar (pose in base_link)", "라이다 (base_link 기준)", "LiDAR（base_link基準）", "雷达（相对 base_link）")
_add("sf_lx", "x (m, + phía trước)", "x (m, + forward)", "x (m)", "x (m)", "x (m)")
_add("sf_ly", "y (m, + bên trái)", "y (m, + left)", "y (m)", "y (m)", "y (m)")
_add("sf_lyaw", "Góc xoay (°)", "Yaw (°)", "회전 (°)", "回転 (°)", "偏航 (°)")
_add("sf_zone_box", "Khoảng cách vùng (m) — kéo thả trên hình hoặc nhập", "Zone margins (m) — drag in the picture or type", "구역 여유 (m)", "ゾーン余白 (m)", "区域边距 (m)")
_add("sf_stop", "STOP (dừng hẳn)", "STOP (full stop)", "STOP (정지)", "STOP (停止)", "STOP (停止)")
_add("sf_slow", "SAFETY (đi chậm)", "SAFETY (slow down)", "SAFETY (감속)", "SAFETY (減速)", "SAFETY (减速)")
_add("sf_front", "Trước", "Front", "앞", "前", "前")
_add("sf_rear", "Sau", "Rear", "뒤", "後", "后")
_add("sf_left", "Trái", "Left", "좌", "左", "左")
_add("sf_right", "Phải", "Right", "우", "右", "右")
_add("sf_beh_box", "Hành vi", "Behaviour", "동작", "動作", "行为")
_add("sf_slow_lin", "Tốc độ tối đa khi chậm (m/s)", "Max speed when slowing (m/s)", "감속 최대 속도 (m/s)", "減速時最大速度 (m/s)", "减速最大速度 (m/s)")
_add("sf_slow_ang", "Tốc độ quay tối đa khi chậm (rad/s)", "Max turn when slowing (rad/s)", "감속 최대 회전 (rad/s)", "減速時最大旋回 (rad/s)", "减速最大转速 (rad/s)")
_add("sf_minpts", "Số điểm lidar tối thiểu", "Min lidar points", "최소 포인트 수", "最小点数", "最少点数")
_add("sf_enabled", "Bật lớp an toàn (safety_monitor)", "Enable the safety layer (safety_monitor)", "안전 레이어 사용", "安全レイヤー有効", "启用安全层")
_add("sf_require", "Mất lidar thì dừng robot", "Stop the robot when the lidar is lost", "라이다 손실 시 정지", "LiDAR断で停止", "雷达丢失时停车")
_add("sf_apply", "💾 Áp dụng & lưu", "💾 Apply & save", "💾 적용 및 저장", "💾 適用して保存", "💾 应用并保存")
_add("sf_revert", "↩ Hoàn tác", "↩ Revert", "↩ 되돌리기", "↩ 元に戻す", "↩ 还原")
_add("sf_dirty", "Chưa áp dụng (đang chỉnh)", "Not applied yet (editing)", "미적용", "未適用", "未应用")
_add("sf_applied", "Đã lưu và gửi tới ROS (/safety_config)", "Saved and sent to ROS (/safety_config)", "저장 및 전송 완료", "保存して送信しました", "已保存并发送")
_add("sf_state_ok", "AN TOÀN", "CLEAR", "정상", "正常", "正常")
_add("sf_state_slow", "ĐI CHẬM", "SLOWING", "감속", "減速中", "减速中")
_add("sf_state_stop", "DỪNG", "STOPPED", "정지", "停止中", "已停止")
_add("sf_state_no_scan", "MẤT LIDAR", "NO LIDAR", "라이다 없음", "LiDARなし", "无雷达")
_add("sf_state_disabled", "TẮT", "DISABLED", "비활성", "無効", "已关闭")
_add("sf_state_unknown", "Chưa có safety_monitor", "No safety_monitor", "safety_monitor 없음", "safety_monitorなし", "无 safety_monitor")
_add("sf_nearest", "Vật gần nhất: {:.2f} m", "Nearest object: {:.2f} m", "가장 가까운 물체: {:.2f} m", "最近物体: {:.2f} m", "最近物体: {:.2f} m")
_add("sf_footprint", "Footprint Nav2:", "Nav2 footprint:", "Nav2 풋프린트:", "Nav2フットプリント:", "Nav2 轮廓:")
_add("sf_warn", "Đây là lớp an toàn phần mềm. Không thay thế E-stop phần cứng hay lidar an toàn đạt chuẩn.",
     "This is a software safety layer. It does not replace the hardware e-stop or a safety-rated lidar.",
     "소프트웨어 안전 레이어입니다. 하드웨어 비상정지를 대체하지 않습니다.", "ソフトウェア安全層です。ハードの非常停止の代わりにはなりません。", "这是软件安全层，不能替代硬件急停。")
_add("sf_legend", "🟧 SAFETY: đi chậm   🟥 STOP: dừng hẳn   ⚪ điểm lidar", "🟧 SAFETY: slow   🟥 STOP: full stop   ⚪ lidar points", "", "", "")
_add("sf_view_hint", "Trước = phía trên. Kéo các chấm ở cạnh hình chữ nhật. Lăn chuột để phóng to/thu nhỏ.",
     "Front = up. Drag the dots on the rectangle edges. Mouse wheel zooms.", "", "", "")
_add("al_safety_stop", "Vật cản trong vùng STOP — robot dừng", "Obstacle in the STOP zone — robot stopped", "STOP 구역에 장애물", "STOPゾーンに障害物", "STOP 区内有障碍物")
_add("al_safety_noscan", "Safety monitor mất lidar — robot bị chặn", "Safety monitor lost the lidar — robot blocked", "라이다 손실", "LiDAR喪失", "雷达丢失")
_add("pf_safety", "Safety monitor có lidar", "Safety monitor has lidar data", "안전 모니터 정상", "安全モニター正常", "安全监视正常")
_add("dash_safety", "🛡 An toàn", "🛡 Safety", "🛡 안전", "🛡 安全", "🛡 安全")

# ── Fork lift ──────────────────────────────────────────────────────────────
_add("page_forklift", "Fork lift", "Fork lift", "지게차", "フォークリフト", "叉车")
_add("rt_label", "Loại robot", "Robot type", "로봇 유형", "ロボット種別", "机器人类型")
_add("rt_agv", "AGV chuyển hàng (băng tải)", "Cargo AGV (conveyor)", "화물 AGV (컨베이어)", "搬送AGV（コンベア）", "搬运 AGV（输送带）")
_add("rt_forklift", "Fork lift (pallet)", "Fork lift (pallet)", "지게차 (팔레트)", "フォークリフト（パレット）", "叉车（托盘）")
_add("rt_mismatch", "Route này dành cho {} nhưng hồ sơ robot đang chọn là {}. Đổi hồ sơ trong trang Robot & An toàn.",
     "This route is for {} but the active robot profile is {}. Change the profile in Robot & Safety.",
     "경로 유형 불일치", "ルート種別が一致しません", "路线类型与机器人配置不符")
_add("task_fork_pick", "Nhận hàng (xúc pallet)", "Pick up pallet", "팔레트 집기", "パレットを取る", "取托盘")
_add("task_fork_drop", "Thả hàng (đặt pallet)", "Drop pallet", "팔레트 내리기", "パレットを置く", "放托盘")
_add("task_fork_lift", "Nâng càng", "Raise fork", "포크 올리기", "フォーク上昇", "升叉")
_add("task_fork_lower", "Hạ càng", "Lower fork", "포크 내리기", "フォーク下降", "降叉")
_add("task_fork_insert", "Đi vào pallet", "Drive into pallet", "팔레트 진입", "パレットに進入", "进入托盘")
_add("task_fork_retract", "Lùi ra khỏi pallet", "Back out of pallet", "팔레트에서 후진", "パレットから後退", "退出托盘")
_add("fk_height", "Chiều cao (mm)", "Height (mm)", "높이 (mm)", "高さ (mm)", "高度 (mm)")
_add("fk_approach_h", "Chiều cao khi vào pallet (mm)", "Entry height (mm)", "진입 높이 (mm)", "進入高さ (mm)", "进入高度 (mm)")
_add("fk_lift_h", "Chiều cao nâng hàng (mm)", "Lift height (mm)", "들어올림 높이 (mm)", "持上高さ (mm)", "起升高度 (mm)")
_add("fk_drop_h", "Chiều cao đặt hàng (mm)", "Drop level (mm)", "내림 높이 (mm)", "置き高さ (mm)", "放置高度 (mm)")
_add("fk_release", "Hạ thêm để nhả hàng (mm)", "Lower to release (mm)", "추가 하강 (mm)", "解放下降 (mm)", "释放下降 (mm)")
_add("fk_travel_h", "Chiều cao khi chạy (mm)", "Travel height (mm)", "주행 높이 (mm)", "走行高さ (mm)", "行驶高度 (mm)")
_add("fk_insert", "Độ sâu đi vào (m)", "Insert depth (m)", "진입 깊이 (m)", "進入深さ (m)", "进入深度 (m)")
_add("fk_retract", "Quãng lùi ra (m)", "Back-out distance (m)", "후진 거리 (m)", "後退距離 (m)", "退出距离 (m)")
_add("fk_waiting", "⏳ Fork lift: {}", "⏳ Fork lift: {}", "⏳ 지게차: {}", "⏳ フォーク: {}", "⏳ 叉车: {}")
_add("fk_failed", "Fork lift lỗi: {}", "Fork lift failed: {}", "지게차 오류: {}", "フォークリフト異常: {}", "叉车故障: {}")
_add("fk_no_node", "Không thấy forklift_node (chưa có /forklift/status)", "forklift_node not responding (no /forklift/status)",
     "forklift_node 응답 없음", "forklift_node 応答なし", "forklift_node 无响应")
_add("fk_state_idle", "Rảnh", "Idle", "대기", "待機", "空闲")
_add("fk_state_running", "Đang chạy", "Running", "실행 중", "実行中", "运行中")
_add("fk_state_done", "Xong", "Done", "완료", "完了", "完成")
_add("fk_state_error", "Lỗi", "Error", "오류", "エラー", "错误")
_add("fk_carry_yes", "Đang mang hàng", "Carrying a load", "화물 적재", "荷物あり", "载货")
_add("fk_carry_no", "Không có hàng", "No load", "화물 없음", "荷物なし", "无货")
_add("fk_set_carry", "Đặt: có hàng", "Set: has load", "적재로 설정", "荷物ありに設定", "设为载货")
_add("fk_clear_carry", "Đặt: không hàng", "Set: no load", "적재 해제", "荷物なしに設定", "设为无货")
_add("fk_goto_h", "Đi tới chiều cao", "Go to height", "높이로 이동", "高さへ移動", "升降到高度")
_add("fk_lower_all", "Hạ hết", "Lower fully", "완전 하강", "最下まで", "降到底")
_add("fk_pick", "📦 Nhận hàng", "📦 Pick up", "📦 집기", "📦 取る", "📦 取货")
_add("fk_drop", "📤 Thả hàng", "📤 Drop", "📤 내리기", "📤 置く", "📤 放货")
_add("fk_insert_btn", "➡ Vào pallet", "➡ Insert", "➡ 진입", "➡ 進入", "➡ 进入")
_add("fk_retract_btn", "⬅ Lùi ra", "⬅ Back out", "⬅ 후진", "⬅ 後退", "⬅ 退出")
_add("fk_stop", "⛔ Dừng fork lift", "⛔ Stop fork lift", "⛔ 지게차 정지", "⛔ フォーク停止", "⛔ 停止叉车")
_add("fk_reset", "Xoá lỗi", "Clear error", "오류 해제", "エラー解除", "清除错误")
_add("fk_manual_box", "Điều khiển tay", "Manual control", "수동 제어", "手動操作", "手动控制")
_add("fk_seq_box", "Chuỗi tự động (thử trước khi đưa vào route)", "Sequences (test before using in a route)", "자동 시퀀스", "自動シーケンス", "自动序列")
_add("fk_status_box", "Trạng thái", "Status", "상태", "状態", "状态")
_add("fk_bypass_on", "Bypass đang bật: {}", "Bypass active: {}", "바이패스 작동: {}", "バイパス有効: {}", "旁路已启用: {}")
_add("fk_warn", "Khi INSERT, vùng phía trước đầu càng được bypass (tốc độ ≤ 0.10 m/s). Vùng phía SAU và hai bên vẫn bảo vệ khi lùi ra. Pallet phải đã được đưa tới đúng vị trí trước càng.",
     "During INSERT the area ahead of the fork tips is bypassed (speed ≤ 0.10 m/s). The REAR and side zones stay active while backing out. The pallet must already be in front of the forks.",
     "", "", "")
_add("fk_not_forklift", "Hồ sơ robot đang chọn không phải fork lift (trang Robot & An toàn).", "The active robot profile is not a fork lift (see Robot & Safety).", "", "", "")
_add("sf_type", "Loại robot", "Robot type", "로봇 유형", "ロボット種別", "机器人类型")
_add("sf_fork_box", "Fork lift: càng và pallet", "Fork lift: forks and pallet", "지게차: 포크/팔레트", "フォークリフト：フォーク/パレット", "叉车：货叉与托盘")
_add("sf_fork_len", "Chiều dài càng (m)", "Fork length (m)", "포크 길이 (m)", "フォーク長 (m)", "货叉长度 (m)")
_add("sf_fork_wid", "Bề rộng hai càng (m)", "Overall fork width (m)", "포크 폭 (m)", "フォーク幅 (m)", "货叉总宽 (m)")
_add("sf_load_wid", "Bề rộng pallet lớn nhất (m)", "Widest pallet (m)", "최대 팔레트 폭 (m)", "最大パレット幅 (m)", "最大托盘宽 (m)")
_add("sf_max_h", "Chiều cao nâng tối đa (mm)", "Max lift height (mm)", "최대 높이 (mm)", "最大高さ (mm)", "最大高度 (mm)")
_add("sf_bypass_ahead", "Bypass phía trước đầu càng (m)", "Bypass ahead of the tips (m)", "바이패스 거리 (m)", "バイパス距離 (m)", "旁路前方距离 (m)")
_add("sf_ins_speed", "Tốc độ vào pallet (m/s)", "Insert speed (m/s)", "진입 속도 (m/s)", "進入速度 (m/s)", "进入速度 (m/s)")
_add("sf_new_forklift", "➕ Fork lift", "➕ Fork lift", "➕ 지게차", "➕ フォーク", "➕ 叉车")
_add("al_forklift", "Fork lift: {}", "Fork lift: {}", "지게차: {}", "フォークリフト: {}", "叉车: {}")
_add("al_forklift", "{}", "{}", "{}", "{}", "{}")

# ── Safety chain diagnosis ────────────────────────────────────────────────
_add("sd_title", "Kiểm tra chuỗi an toàn (vì sao vùng không có tác dụng?)", "Safety chain check (why do the zones have no effect?)")
_add("sd_not_running", "safety_monitor KHÔNG chạy (không có /safety_state) — vùng safety/stop không tác dụng.", "safety_monitor is NOT running (no /safety_state) — the zones have no effect.")
_add("sd_not_running_hint", "Chạy: ros2 launch hiep_robot2 agv_hardware.launch.py profile:=real (hoặc safety:=true), hoặc safety_monitor.launch.py", "Run: ros2 launch hiep_robot2 agv_hardware.launch.py profile:=real (or safety:=true), or safety_monitor.launch.py")
_add("sd_running", "safety_monitor đang chạy: trạng thái {}, hồ sơ '{}'", "safety_monitor running: state {}, profile '{}'")
_add("sd_disabled", "Lớp an toàn đang TẮT trong hồ sơ.", "The safety layer is DISABLED in the profile.")
_add("sd_disabled_hint", "Bật 'Bật lớp an toàn' rồi Áp dụng & lưu.", "Tick 'Enable the safety layer' then Apply & save.")
_add("sd_no_consumer", "KHÔNG node nào nghe {} — cổng an toàn không nằm trên đường lệnh, robot bỏ qua vùng.", "NOBODY listens to {} — the gate is not in the command path, the robot ignores the zones.")
_add("sd_no_consumer_hint", "Bridge động cơ phải nhận lệnh từ /cmd_vel_safe: chạy agv_hardware.launch.py safety:=true, hoặc remap node điều khiển motor (kể cả bridge Gazebo) sang /cmd_vel_safe.", "The motor bridge must read /cmd_vel_safe: run agv_hardware.launch.py safety:=true, or remap your motor node (including the Gazebo bridge) to /cmd_vel_safe.")
_add("sd_consumer_unknown", "Chưa biết ai nghe {}.", "Cannot tell who listens to {}.")
_add("sd_consumer_ok", "{} có {} node đang nghe.", "{} has {} listener(s).")
_add("sd_direct", "Các node sau vẫn nghe {} trực tiếp nên KHÔNG bị chặn: {}", "These nodes still listen to {} directly and are NOT gated: {}")
_add("sd_direct_hint", "Đổi tham số cmd_vel_topic của chúng sang /cmd_vel_safe (hoặc remap trong launch).", "Point them to /cmd_vel_safe (cmd_vel_topic parameter or a remap in the launch file).")
_add("sd_direct_ok", "Không node nào khác nghe {} trực tiếp.", "No other node listens to {} directly.")
_add("sd_no_scan", "safety_monitor không nhận được /scan mới.", "safety_monitor receives no fresh /scan.")
_add("sd_no_scan_hint", "Kiểm tra driver lidar / tên topic scan.", "Check the lidar driver / scan topic name.")
_add("sd_scan_ok", "Nhận /scan (frame '{}').", "/scan is received (frame '{}').")
_add("sd_pose_tf", "Vị trí lidar lấy từ TF: x={} y={} yaw={}° trong '{}'.", "Lidar pose from TF: x={} y={} yaw={}° in '{}'.")
_add("sd_pose_profile", "Vị trí lidar lấy từ SỐ NHẬP TAY: x={} y={} yaw={}° (không có TF).", "Lidar pose comes from the MANUAL numbers: x={} y={} yaw={}° (no TF).")
_add("sd_pose_profile_hint", "Nếu số này không đúng vị trí thật (ví dụ scan gộp đã ở base_link) thì mọi vùng bị lệch. Sửa TF hoặc số trong hồ sơ.", "If this is not the real lidar position (e.g. a merged scan already in base_link) every zone is shifted. Fix TF or the profile numbers.")
_add("sd_sig_bad", "Node đang chạy cấu hình KHÁC với hồ sơ trong HMI.", "The node runs a DIFFERENT configuration than the HMI profile.")
_add("sd_sig_bad_hint", "Bấm 'Áp dụng & lưu' (gửi /safety_config), rồi kiểm tra lại.", "Press 'Apply & save' (sends /safety_config), then check again.")
_add("sd_sig_ok", "Node chạy đúng cấu hình của hồ sơ.", "The node runs the same configuration as the profile.")
_add("sd_live", "Lệnh vào ({:.2f} m/s, {:.2f} rad/s) → ra ({:.2f} m/s, {:.2f} rad/s), trạng thái {}", "Command in ({:.2f} m/s, {:.2f} rad/s) → out ({:.2f} m/s, {:.2f} rad/s), state {}")
_add("sf_active_in_ros", "Hồ sơ đang áp dụng cho robot: {} ({})", "Profile applied to the robot: {} ({})")
_add("hw_safety", "Safety monitor", "Safety monitor")
_add("hw_forklift", "Fork lift", "Fork lift")

# ── Fork lift page: parameters + guide ───────────────────────────────────
_add("fk_tab_control", "Điều khiển", "Control")
_add("fk_tab_params", "Thông số", "Parameters")
_add("fk_tab_guide", "Hướng dẫn", "Guide")
_add("fk_grp_geom", "Hình học càng và pallet", "Forks and pallet")
_add("fk_grp_heights", "Chiều cao", "Heights")
_add("fk_grp_motion", "Chuyển động vào/ra pallet", "Driving into / out of the pallet")
_add("fk_grp_bypass", "Vùng bypass khi vào pallet", "Bypass while inserting")
_add("fk_grp_adv", "Nâng cao (điều khiển cơ cấu nâng)", "Advanced (fork control)")
_add("fk_p_flen", "Chiều dài càng (m)", "Fork length (m)")
_add("fk_p_fwid", "Bề rộng hai càng (m)", "Overall fork width (m)")
_add("fk_p_lwid", "Bề rộng pallet lớn nhất (m)", "Widest pallet (m)")
_add("fk_p_overhang", "Hàng nhô ra khỏi đầu càng (m)", "Load overhang beyond the tips (m)")
_add("fk_p_maxh", "Chiều cao nâng tối đa (mm)", "Max lift height (mm)")
_add("fk_p_insspeed", "Tốc độ vào pallet (m/s)", "Insert speed (m/s)")
_add("fk_p_retspeed", "Tốc độ lùi ra (m/s)", "Back-out speed (m/s)")
_add("fk_p_bahead", "Bypass phía trước đầu càng (m)", "Bypass ahead of the tips (m)")
_add("fk_p_bmargin", "Lề bypass hai bên càng (m)", "Bypass side margin (m)")
_add("fk_p_tol", "Sai số chiều cao cho phép (mm)", "Height tolerance (mm)")
_add("fk_p_maxspd", "Tốc độ nâng/hạ tối đa (%)", "Max fork speed (%)")
_add("fk_p_minspd", "Tốc độ nâng/hạ tối thiểu (%)", "Min fork speed (%)")
_add("fk_p_slowzone", "Giảm tốc khi còn cách đích (mm)", "Slow down within (mm)")
_add("fk_p_stall", "Phát hiện càng kẹt sau (giây)", "Stall detection after (s)")
_add("fk_p_blocked", "Phát hiện xe bị chặn sau (giây)", "Blocked detection after (s)")
_add("fk_p_hto", "Timeout bước nâng/hạ (giây)", "Fork step timeout (s)")
_add("fk_p_dto", "Timeout bước vào/lùi (giây)", "Drive step timeout (s)")
_add("fk_save", "💾 Lưu thông số fork lift", "💾 Save fork lift parameters")
_add("fk_defaults", "↺ Khôi phục mặc định", "↺ Restore defaults")
_add("fk_saved", "Đã lưu và gửi cho forklift_node.", "Saved and sent to forklift_node.")
_add("fk_unsaved", "Chưa lưu (đang chỉnh).", "Not saved yet (editing).")
_add("fk_profile_is", "Đang chỉnh hồ sơ: {} ({})", "Editing profile: {} ({})")
_add("fk_where", "Lưu tại: {}\nTham số riêng của từng waypoint nằm trong lộ trình: {}", "Stored in: {}\nPer-waypoint task values are inside the route file: {}")
_add("fk_sync_ok", "✅ forklift_node đang dùng đúng các thông số đã lưu.", "✅ forklift_node runs the saved parameters.")
_add("fk_sync_bad", "⚠ forklift_node đang dùng giá trị khác: {} — bấm 'Lưu thông số' để gửi lại.", "⚠ forklift_node uses different values: {} — press 'Save' to send them again.")
_add("fk_sync_none", "forklift_node chưa báo thông số (chưa chạy?).", "forklift_node has not reported its parameters (not running?).")
_add("fk_test_hint", "Giá trị dùng khi thử bằng nút bên dưới (lấy từ tab Thông số, không tự lưu).", "Values used by the test buttons below (taken from the Parameters tab; not saved).")
_add("fk_guide",
 "<h3>Cách cài đặt và dùng fork lift</h3><ol>"
 "<li><b>Tạo hồ sơ fork lift</b> — trang <i>Robot &amp; An toàn</i> → nút <i>➕ Fork lift</i> → đặt tên. Nhập kích thước thân xe (<i>Dài</i> tính cả càng), bề rộng, vùng SAFETY/STOP → <i>Áp dụng &amp; lưu</i>.</li>"
 "<li><b>Thông số fork lift</b> — tab <i>Thông số</i> (ngay trong trang này): kích thước càng và pallet, chiều cao vào pallet / nâng / chạy, độ sâu vào, quãng lùi, tốc độ, vùng bypass → <i>Lưu thông số fork lift</i>.</li>"
 "<li><b>Chạy phần cứng</b> — trang <i>Chẩn đoán → Phần cứng</i>: profile <b>real</b>, safety <b>true</b>, forklift <b>true</b> (càng ảo để thử) hoặc <b>plc</b> (càng thật qua Mega2560) → Khởi động.</li>"
 "<li><b>Thử từng bước</b> — tab <i>Điều khiển</i>: <i>Đi tới chiều cao</i> → <i>Vào pallet</i> → <i>Nhận hàng</i> / <i>Thả hàng</i>. Nút chuyển động phải giữ 0.8 giây. Thử ở chỗ trống, có người đứng cạnh nút E-stop.</li>"
 "<li><b>Lập chương trình</b> — trang <i>Điều hướng</i> → <i>Loại robot: Fork lift</i>. Đặt waypoint ở điểm đứng <b>trước pallet, thẳng hàng với pallet</b> → thêm task <i>Nhận hàng</i>; waypoint tiếp theo ở trước kệ → task <i>Thả hàng</i> → Lưu lộ trình rồi chạy ở trang <i>Lộ trình</i>.</li></ol>"
 "<p><b>Lưu ý:</b> khi vào pallet robot đi thẳng theo quãng đường đo bằng odometry (chưa có camera nhận pallet), nên Nav2 phải đưa robot tới trước pallet với sai số vài cm. Vùng phía sau và hai bên vẫn được bảo vệ khi lùi ra.</p>",
 "<h3>How to set up and use the fork lift</h3><ol>"
 "<li><b>Create a fork-lift profile</b> — page <i>Robot &amp; Safety</i> → <i>➕ Fork lift</i> → name it. Enter the truck size (<i>Length</i> includes the forks), width, SAFETY/STOP zones → <i>Apply &amp; save</i>.</li>"
 "<li><b>Fork-lift parameters</b> — the <i>Parameters</i> tab of this page: fork and pallet size, entry / lift / travel heights, insert depth, back-out distance, speeds, bypass → <i>Save fork lift parameters</i>.</li>"
 "<li><b>Start the hardware</b> — <i>Diagnostics → Hardware</i>: profile <b>real</b>, safety <b>true</b>, forklift <b>true</b> (virtual fork for tests) or <b>plc</b> (real fork on the Mega2560) → Start.</li>"
 "<li><b>Try each step</b> — <i>Control</i> tab: <i>Go to height</i> → <i>Insert</i> → <i>Pick up</i> / <i>Drop</i>. Motion buttons need a 0.8 s hold. Test in free space with someone at the e-stop.</li>"
 "<li><b>Program a route</b> — <i>Navigation</i> page → <i>Robot type: Fork lift</i>. Put the waypoint <b>in front of the pallet, aligned with it</b> → add the <i>Pick up</i> task; next waypoint in front of the rack → <i>Drop</i> task → save the route and run it from the <i>Routes</i> page.</li></ol>"
 "<p><b>Note:</b> the insert drives straight for a distance measured by odometry (no pallet camera yet), so Nav2 must bring the truck in front of the pallet within a few centimetres. The rear and side zones stay active when backing out.</p>")


# ── 4.7.0: collapsible bars / animation ─────────────────────────────────
_add("panel_hide", "Ẩn thanh này", "Hide this panel", "패널 숨기기", "パネルを隠す", "隐藏此面板")
_add("panel_show", "Hiện thanh này", "Show this panel", "패널 표시", "パネルを表示", "显示此面板")
_add("sb_collapse", "Thu gọn", "Collapse", "접기", "折りたたむ", "收起")
_add("sb_expand", "Mở rộng", "Expand", "펼치기", "展開", "展开")
_add("tb_menu_tip", "Thu gọn / mở rộng menu (Ctrl+B)", "Collapse / expand menu (Ctrl+B)",
     "메뉴 접기/펼치기 (Ctrl+B)", "メニューの折りたたみ/展開 (Ctrl+B)", "收起/展开菜单 (Ctrl+B)")
_add("tb_focus_tip", "Chế độ tập trung: ẩn menu và các thanh bên (F11)",
     "Focus mode: hide menu and side panels (F11)",
     "집중 모드: 메뉴와 사이드 패널 숨기기 (F11)", "集中モード: メニューとサイドパネルを隠す (F11)",
     "专注模式：隐藏菜单和侧栏 (F11)")
_add("tb_focus_off_tip", "Thoát chế độ tập trung (F11 / Esc)", "Leave focus mode (F11 / Esc)",
     "집중 모드 종료 (F11 / Esc)", "集中モードを終了 (F11 / Esc)", "退出专注模式 (F11 / Esc)")
_add("joy_hide", "Thu gọn joystick", "Fold joystick", "조이스틱 접기", "ジョイスティックを畳む", "收起摇杆")
_add("joy_show", "Hiện joystick", "Show joystick", "조이스틱 표시", "ジョイスティックを表示", "显示摇杆")
_add("set_animations", "Hiệu ứng chuyển động (tắt nếu máy HMI chậm)",
     "Animations (turn off on a slow HMI PC)",
     "애니메이션 (느린 HMI PC에서는 끄기)", "アニメーション (低速なHMI PCではオフ)", "动画效果（HMI 电脑较慢时请关闭）")


# ── 4.8.0: two lidars + Engineer teleop override ────────────────────────
_add("sf_lidar_mode", "Kiểu lidar", "Lidar set-up", "라이다 구성", "LiDAR構成", "雷达配置")
_add("sf_lmode_merged", "1 scan gộp (/scan, đã ở khung base)", "1 merged scan (/scan, already in the base frame)",
     "병합 스캔 1개 (/scan)", "結合スキャン1つ (/scan)", "1 路合并扫描 (/scan)")
_add("sf_lmode_dual", "2 lidar (trước + sau), mỗi cái 1 topic", "2 lidars (front + rear), one topic each",
     "라이다 2개 (전/후), 토픽 각각", "LiDAR2台（前/後）、トピック別", "2 个雷达（前+后），各自话题")
_add("sf_l1_single", "Vị trí lidar (dự phòng khi thiếu TF)", "Lidar pose (fallback when TF is missing)",
     "라이다 위치 (TF 없을 때)", "LiDAR位置（TFなし時）", "雷达位置（缺少 TF 时）")
_add("sf_l1_box", "Lidar 1 (trước)", "Lidar 1 (front)", "라이다 1 (전방)", "LiDAR 1（前）", "雷达 1（前）")
_add("sf_l2_box", "Lidar 2 (sau)", "Lidar 2 (rear)", "라이다 2 (후방)", "LiDAR 2（後）", "雷达 2（后）")
_add("sf_ltopic", "Topic scan", "Scan topic", "스캔 토픽", "スキャントピック", "扫描话题")
_add("sf_lidar_hint",
     "Vị trí lấy từ TF nếu có; số nhập ở đây chỉ là dự phòng khi thiếu TF. Mũi tên trên hình là hướng 0° của scan (lidar sau thường yaw 180°). Nhấn Enter sau khi sửa topic.",
     "The pose comes from TF when available; the numbers here are only a fallback. The arrow in the drawing is the 0° direction of the scan (a rear lidar is usually yaw 180°). Press Enter after editing a topic.")
_add("sd_lidar_ok", "Lidar {} ({}): có scan, vị trí lấy từ TF x={} y={} yaw={}°.",
     "Lidar {} ({}): scan OK, pose from TF x={} y={} yaw={}°.")
_add("sd_lidar_profile", "Lidar {} ({}): có scan nhưng vị trí là SỐ NHẬP TAY x={} y={} yaw={}° (không có TF).",
     "Lidar {} ({}): scan OK but the pose is the MANUAL numbers x={} y={} yaw={}° (no TF).")
_add("sd_lidar_profile_hint", "Nếu số không đúng vị trí thật thì vùng bị lệch. Kiểm tra TF từ khung lidar tới base.",
     "If the numbers are not the real position the zones are shifted. Check the TF from the lidar frame to the base.")
_add("sd_lidar_lost", "Lidar {} ({}) KHÔNG có scan — robot bị chặn (an toàn).",
     "Lidar {} ({}) has NO scan — the robot is blocked (fail safe).")
_add("sd_lidar_lost_hint", "Kiểm tra topic ở trên, cáp/nguồn lidar, và gz bridge nếu đang mô phỏng.",
     "Check the topic above, the lidar cable/power, and the gz bridge in simulation.")
_add("sd_override", "Vùng safety đang bị TẮT bởi Engineer (teleop: {}).", "Safety zones are switched OFF by the Engineer (teleop: {}).")
_add("sd_override_hint", "Tắt công tắc trên joystick, hoặc chạy mission để bật lại. Tự bật lại sau ≤ 2 s nếu HMI mất kết nối.",
     "Switch it off on the joystick, or start a mission. It re-enables itself within 2 s if the HMI link is lost.")
_add("sf_state_override", "TẮT SAFETY (teleop)", "SAFETY OFF (teleop)", "안전 해제 (원격)", "安全解除（遠隔）", "安全已关闭（遥控）")
_add("al_safety_override", "Engineer đang tắt vùng safety (teleop)", "The Engineer switched the safety zones off (teleop)",
     "엔지니어가 안전 구역을 껐습니다", "エンジニアが安全ゾーンをオフにしました", "工程师已关闭安全区")
_add("joy_safety_on", "🛡 Safety: BẬT", "🛡 Safety: ON", "🛡 안전: 켜짐", "🛡 安全: ON", "🛡 安全：开")
_add("joy_safety_off", "⚠ SAFETY TẮT", "⚠ SAFETY OFF", "⚠ 안전 해제", "⚠ 安全OFF", "⚠ 安全已关")
_add("joy_safety_tip", "Chỉ Engineer: tắt vùng slow/stop khi teleop. Tự bật lại khi chạy mission hoặc mất kết nối HMI.",
     "Engineer only: switch the slow/stop zones off while teleoperating. Re-enabled automatically when a mission starts or the HMI link is lost.")
_add("joy_safety_title", "Tắt vùng safety?", "Switch the safety zones off?", "안전 구역을 끄시겠습니까?", "安全ゾーンをオフにしますか？", "关闭安全区？")
_add("joy_safety_confirm",
     "Robot sẽ KHÔNG chậm hoặc dừng khi có người/vật cản trong vùng safety. Chỉ dùng khi teleop để thoát chỗ kẹt và phải có người quan sát robot.\n\nTiếp tục?",
     "The robot will NOT slow down or stop for people/obstacles in the safety zones. Use it only to teleoperate out of a tight spot, with someone watching the robot.\n\nContinue?")
_add("joy_safety_toast_on", "⚠ Vùng safety đã TẮT (teleop)", "⚠ Safety zones are OFF (teleop)")
_add("joy_safety_toast_off", "🛡 Vùng safety đã bật lại", "🛡 Safety zones are back ON")


# ── 4.9.0: manual map editing + missions keep running on other pages ────
_add("ed_title", "✏️ Sửa map bằng tay", "✏️ Edit map by hand", "✏️ 맵 수동 편집", "✏️ マップ手動編集", "✏️ 手动编辑地图")
_add("ed_start", "✏️ Bật chế độ sửa", "✏️ Start editing", "✏️ 편집 시작", "✏️ 編集開始", "✏️ 开始编辑")
_add("ed_stop", "✔ Xong sửa", "✔ Finish editing", "✔ 편집 종료", "✔ 編集終了", "✔ 完成编辑")
_add("ed_t_wall", "🖊 Vẽ tường", "🖊 Draw wall", "🖊 벽 그리기", "🖊 壁を描く", "🖊 画墙")
_add("ed_t_erase", "🧽 Tẩy (về trống)", "🧽 Erase (free)", "🧽 지우개 (빈 공간)", "🧽 消しゴム（空き）", "🧽 擦除（空闲）")
_add("ed_t_unknown", "◼ Xoá sạch (chưa biết)", "◼ Clear (unknown)", "◼ 지우기 (미확인)", "◼ クリア（未知）", "◼ 清除（未知）")
_add("ed_t_line", "📏 Đường thẳng", "📏 Straight wall", "📏 직선 벽", "📏 直線の壁", "📏 直墙")
_add("ed_t_rect_clear", "⬚ Xoá vùng chữ nhật", "⬚ Clear rectangle", "⬚ 사각형 지우기", "⬚ 矩形を消去", "⬚ 清除矩形")
_add("ed_t_rect_wall", "▮ Tường chữ nhật", "▮ Solid rectangle", "▮ 사각형 벽", "▮ 矩形の壁", "▮ 实心矩形")
_add("ed_brush", "Cỡ bút / độ dày tường", "Brush / wall thickness", "브러시 / 벽 두께", "ブラシ／壁の太さ", "笔刷 / 墙厚")
_add("ed_cells", "Số ô (làm dày/mỏng)", "Cells (thicken/thin)", "셀 수 (두껍게/얇게)", "セル数（太く/細く）", "格数（加厚/变薄）")
_add("ed_snap", "Bám góc 0/45/90° (đường thẳng)", "Snap to 0/45/90° (straight wall)", "각도 맞춤 (직선)", "角度スナップ（直線）", "角度吸附（直线）")
_add("ed_undo", "↶ Hoàn tác", "↶ Undo", "↶ 실행 취소", "↶ 元に戻す", "↶ 撤销")
_add("ed_redo", "↷ Làm lại", "↷ Redo", "↷ 다시 실행", "↷ やり直し", "↷ 重做")
_add("ed_thicken", "➕ Làm dày tường", "➕ Thicken walls", "➕ 벽 두껍게", "➕ 壁を太く", "➕ 加厚墙")
_add("ed_thin", "➖ Làm mỏng tường", "➖ Thin walls", "➖ 벽 얇게", "➖ 壁を細く", "➖ 变薄墙")
_add("ed_specks", "🧹 Xoá điểm nhiễu nhỏ", "🧹 Remove small specks", "🧹 작은 점 제거", "🧹 小さなノイズ除去", "🧹 清除小噪点")
_add("ed_save", "💾 Lưu map đã sửa", "💾 Save edited map", "💾 편집한 맵 저장", "💾 編集マップを保存", "💾 保存已编辑地图")
_add("ed_hint",
     "Chuột trái: vẽ / kéo. Chuột phải: kéo map. Lăn chuột: zoom. Ctrl+Z / Ctrl+Y: hoàn tác / làm lại. Esc: huỷ nét đang kéo. Mỗi nét vẽ là 1 bước hoàn tác.",
     "Left mouse: draw / drag. Right mouse: pan. Wheel: zoom. Ctrl+Z / Ctrl+Y: undo / redo. Esc: cancel the current drag. Each stroke is one undo step.")
_add("ed_on", "Đang sửa map (cập nhật SLAM tạm dừng). Tường vector (nếu có) được chuyển thành ô để sửa.",
     "Editing the map (live SLAM updates paused). Vector walls, if any, were turned into cells so they can be edited.")
_add("ed_off", "Đã thoát chế độ sửa. Bấm 'Lưu map đã sửa' để ghi ra file.", "Editing finished. Press 'Save edited map' to write it to a file.")
_add("ed_no_map", "Chưa có map để sửa.", "No map to edit yet.")
_add("ed_thick_done", "Đã đổi {} ô.", "{} cells changed.")
_add("ed_specks_done", "Đã xoá {} ô nhiễu.", "{} noise cells removed.")
_add("ed_saved", "Đã lưu map đã sửa:\n{}", "Edited map saved:\n{}")
_add("ed_discard", "Thao tác này sẽ thay map đang hiển thị và MẤT các chỉnh sửa tay chưa lưu. Tiếp tục?",
     "This replaces the map on screen and LOSES the hand edits you have not saved. Continue?")
_add("mv_measure_clear", "🗑 Xoá phép đo", "🗑 Clear measurement", "🗑 측정 지우기", "🗑 計測を消去", "🗑 清除测量")
_add("mv_measure_hint", "Bấm nhiều điểm để đo đường gấp khúc (Σ = tổng). Với 1 đoạn hiện thêm Δx, Δy và góc. Esc: xoá.",
     "Click several points to measure a polyline (Σ = total). One segment also shows Δx, Δy and the angle. Esc: clear.")

_add("mb_running", "▶ Đang chạy lộ trình · điểm {}/{} · {}", "▶ Mission running · waypoint {}/{} · {}")
_add("mb_paused", "⏸ Lộ trình tạm dừng · điểm {}/{} · {}", "⏸ Mission paused · waypoint {}/{} · {}")
_add("mb_confirm", "⏳ Chờ bạn xác nhận · điểm {}/{} · {}", "⏳ Waiting for your confirmation · waypoint {}/{} · {}")
_add("mb_stuck", "⚠ KẸT · điểm {}/{} · {} — bấm 'Xem' để chọn Thử lại / Bỏ qua / Huỷ",
     "⚠ STUCK · waypoint {}/{} · {} — press 'View' to Retry / Skip / Cancel")
_add("mb_view", "Xem", "View"); _add("mb_stop", "■ Dừng", "■ Stop"); _add("mb_ok", "✔ Xác nhận", "✔ Confirm")
_add("mb_stop_confirm", "Dừng lộ trình đang chạy?", "Stop the running mission?")
_add("mb_stuck_toast", "⚠ AGV kẹt tại điểm {} — mở thẻ Routes để xử lý", "⚠ AGV is stuck at waypoint {} — open Routes to handle it")
_add("mb_confirm_toast", "⏳ AGV đang chờ xác nhận để tiếp tục", "⏳ The AGV is waiting for confirmation to continue")
_add("mb_done_toast", "✅ Lộ trình đã hoàn thành", "✅ Mission completed")
_add("mb_mapping_title", "Đang chạy lộ trình", "A mission is running")
_add("mb_mapping_warn",
     "Mapping (SLAM) xung đột với Nav2 (cùng /map và TF map→odom). Vào thẻ Mapping sẽ DỪNG lộ trình đang chạy. Tiếp tục?",
     "Mapping (SLAM) conflicts with Nav2 (same /map and map→odom TF). Opening Mapping will STOP the running mission. Continue?")

# ── 4.9.1: open an old saved map for editing ────────────────────────────
_add("ed_open", "📂 Mở map đã lưu để sửa", "📂 Open a saved map to edit", "📂 저장된 맵 열기", "📂 保存済みマップを開く", "📂 打开已保存地图编辑")
_add("ed_opened", "Đã mở '{}' để sửa. Cập nhật SLAM tạm dừng. Lưu ra tên mới để giữ bản gốc.",
     "Opened '{}' for editing. Live SLAM updates are paused. Save under a new name to keep the original.")
_add("ed_open_fail", "Không đọc được map: {}", "Could not read the map: {}")
_add("ed_open_mapping", "Đang chạy Mapping. Dừng Mapping để mở map cũ?", "Mapping is running. Stop it to open the old map?")

# ── 4.10.0: 6 main tabs + sub-tabs, status chips on the title bar ───────
_add("sub_overview", "Tổng quan", "Overview", "개요", "概要", "概览")
_add("sub_navigation", "Điều hướng", "Navigation", "내비게이션", "ナビゲーション", "导航")
_add("sub_mapping", "Vẽ map", "Mapping", "매핑", "マッピング", "建图")
_add("sub_routes", "Lộ trình", "Routes", "경로", "ルート", "路线")
_add("sub_general", "Chung", "General", "일반", "一般", "常规")
_add("chip_safety_tip", "Trạng thái vùng safety. Bấm để mở cấu hình (Engineer).", "Safety zone state. Click to open its set-up (Engineer).")
_add("chip_nav_on", "Nav2 sẵn sàng", "Nav2 ready", "Nav2 준비", "Nav2 準備完了", "Nav2 就绪")
_add("chip_nav_off", "Nav2 tắt", "Nav2 off", "Nav2 꺼짐", "Nav2 停止", "Nav2 关闭")
_add("chip_nav_tip", "Nav2 (NavigateToPose) có sẵn sàng nhận lệnh không. Bấm để mở thẻ Navigation.",
     "Whether Nav2 (NavigateToPose) can take goals. Click to open Navigation.")
_add("chip_mission_idle", "Rảnh", "Idle", "대기", "待機", "空闲")
_add("chip_mission_run", "Đang chạy {}", "Running {}", "실행 중 {}", "実行中 {}", "运行中 {}")
_add("chip_mission_wait", "Chờ xác nhận {}", "Confirm? {}", "확인 대기 {}", "確認待ち {}", "等待确认 {}")
_add("chip_mission_pause", "Tạm dừng {}", "Paused {}", "일시정지 {}", "一時停止 {}", "已暂停 {}")
_add("chip_mission_stuck", "KẸT {}", "STUCK {}", "막힘 {}", "スタック {}", "卡住 {}")
_add("chip_mission_tip", "Lộ trình đang chạy (điểm hiện tại/tổng). Bấm để xem hoặc mở Routes.",
     "The running mission (current/total waypoint). Click to view it or open Routes.")
_add("sub_conveyors", "Băng tải", "Conveyors", "컨베이어", "コンベア", "输送带")

# ── 4.11.0: Fleet control (start the fleet and connect robots from the HMI) ──
_add("fc_title", "🛰 Điều khiển Fleet — bật và kết nối không cần gõ lệnh", "🛰 Fleet control — start and connect without typing commands")
_add("fc_pkg", "Gói ROS cần thiết", "Required ROS packages")
_add("fc_pkg_check", "Kiểm tra lại", "Check again")
_add("fc_pkg_checking", "Đang kiểm tra gói ROS...", "Checking ROS packages...")
_add("fc_pkg_fix_build", "Chưa có hoặc bản cũ (thiếu robot_fleet_side.launch.py). Build lại:  cd ~/hiep_ros2 && colcon build --packages-select agv_fleet agv_fleet_domains && source install/setup.fish",
     "Missing or too old (no robot_fleet_side.launch.py). Rebuild:  cd ~/hiep_ros2 && colcon build --packages-select agv_fleet agv_fleet_domains && source install/setup.fish")
_add("fc_pkg_fix_apt", "Thiếu cầu nối domain. Cài:  sudo apt install ros-jazzy-domain-bridge", "The domain bridge is missing. Install:  sudo apt install ros-jazzy-domain-bridge")
_add("fc_robot_box", "Robot này — Fleet Agent + cầu nối domain", "This robot — Fleet Agent + domain bridge")
_add("fc_server_box", "Fleet Server — máy trung tâm", "Fleet Server — central computer")
_add("fc_server_hint", "Chỉ bật trên MỘT máy (máy trung tâm). Các robot gửi trạng thái về Fleet Server qua cầu nối domain.",
     "Start it on ONE computer only (the central one). Robots report to it through the domain bridge.")
_add("fc_robot_id", "Mã robot", "Robot ID"); _add("fc_robot_name", "Tên hiển thị", "Display name")
_add("fc_robot_domain", "Domain ROS của robot", "Robot ROS domain"); _add("fc_fleet_domain", "Domain Fleet", "Fleet domain")
_add("fc_namespace", "Namespace topic robot", "Robot topic namespace"); _add("fc_publish_hz", "Gửi trạng thái (Hz)", "State rate (Hz)")
_add("fc_snapshot_hz", "Gửi snapshot (Hz)", "Snapshot rate (Hz)"); _add("fc_warn_to", "Cảnh báo sau (s)", "Warn after (s)")
_add("fc_off_to", "Offline sau (s)", "Offline after (s)")
_add("fc_auto_connect", "Tự kết nối khi mở HMI", "Connect automatically when the HMI starts")
_add("fc_auto_server", "Tự bật khi mở HMI", "Start automatically when the HMI starts")
_add("fc_btn_connect", "🔌 Kết nối robot vào Fleet", "🔌 Connect robot to the fleet"); _add("fc_btn_disconnect", "⏏ Ngắt kết nối", "⏏ Disconnect")
_add("fc_btn_server_on", "▶ Bật Fleet Server", "▶ Start Fleet Server"); _add("fc_btn_server_off", "■ Tắt Fleet Server", "■ Stop Fleet Server")
_add("fc_run_on", "Đang chạy", "Running"); _add("fc_run_off", "Chưa chạy", "Not running")
_add("fc_run_ext", "Đang chạy (từ phiên trước hoặc terminal)", "Running (from an earlier session or a terminal)")
_add("fc_run_died", "Đã dừng bất thường (mã {}) — xem log bên dưới", "Stopped unexpectedly (code {}) — see the log below")
_add("fc_log", "Log:", "Log:"); _add("fc_log_agent", "Agent + cầu nối", "Agent + bridge"); _add("fc_log_server", "Fleet Server", "Fleet Server")
_add("fc_log_empty", "(chưa có log)", "(no log yet)")
_add("fc_err_robot_id", "Mã robot phải dài 3–32 ký tự, chữ HOA/số/_/-, bắt đầu bằng chữ (ví dụ AGV001).",
     "Robot ID: 3–32 characters, UPPERCASE letters/digits/_/-, starting with a letter (e.g. AGV001).")
_add("fc_err_domain_range", "Domain {} phải nằm trong khoảng 0–{}.", "The {} domain must be 0–{}.")
_add("fc_err_same_domain", "Domain robot và domain Fleet phải KHÁC nhau (cầu nối cần 2 domain).", "Robot domain and Fleet domain must DIFFER (a bridge needs two domains).")
_add("fc_err_rate", "Tần số phải trong khoảng 0.5–20 Hz.", "Rates must be 0.5–20 Hz.")
_add("fc_err_timeouts", "Thời gian 'offline' phải lớn hơn thời gian 'cảnh báo'.", "'Offline after' must be larger than 'Warn after'.")
_add("fc_err_pkg", "Thiếu gói ROS: {}. Xem hướng dẫn ngay phía trên.", "Missing ROS package(s): {}. See the hint above.")
_add("fc_dom_ok", "HMI đang ở domain {} = domain robot: nhận /fleet/snapshot qua cầu nối.", "The HMI is in domain {} = the robot domain: it receives /fleet/snapshot through the bridge.")
_add("fc_dom_fleet", "HMI đang ở domain {} (domain Fleet): nhận /fleet/snapshot trực tiếp từ Fleet Server.", "The HMI is in domain {} (the Fleet domain): it receives /fleet/snapshot directly from the Fleet Server.")
_add("fc_dom_mismatch", "⚠ HMI đang ở domain {}, còn domain robot là {} và domain Fleet là {}: HMI sẽ KHÔNG thấy dữ liệu Fleet. Mở HMI với ROS_DOMAIN_ID bằng một trong hai domain đó.",
     "⚠ The HMI is in domain {} but the robot domain is {} and the Fleet domain is {}: the HMI will NOT see Fleet data. Start the HMI with ROS_DOMAIN_ID set to one of them.")
_add("fc_st_connected", "✅ Robot {} đã kết nối Fleet — thấy {} AGV.", "✅ Robot {} is connected to the fleet — {} AGV seen.")
_add("fc_st_my_stale", "⚠ Robot {} có trong Fleet nhưng trạng thái là '{}'.", "⚠ Robot {} is in the fleet but its state is '{}'.")
_add("fc_st_not_listed", "⚠ Agent của {} đang chạy nhưng Fleet Server chưa thấy robot này. Kiểm tra domain, cầu nối và log.",
     "⚠ The agent of {} is running but the Fleet Server does not see this robot. Check the domains, the bridge and the log.")
_add("fc_st_snapshot", "✅ Đang nhận /fleet/snapshot — {} AGV.", "✅ Receiving /fleet/snapshot — {} AGV.")
_add("fc_st_nothing", "Chưa bật gì. Bật Fleet Server (máy trung tâm) và/hoặc kết nối robot này vào Fleet.", "Nothing is running. Start the Fleet Server (central computer) and/or connect this robot to the fleet.")
_add("fc_st_agent_waiting", "Agent đã chạy nhưng chưa nhận /fleet/snapshot — Fleet Server chưa bật hoặc cầu nối chưa thông.",
     "The agent is running but /fleet/snapshot has not arrived — the Fleet Server is not started or the bridge is not connected yet.")
_add("fc_st_server_no_robot", "Fleet Server đã chạy nhưng chưa có robot nào kết nối.", "The Fleet Server is running but no robot has connected yet.")
_add("fc_st_both_waiting", "Agent và Server đã chạy, đang chờ dữ liệu đầu tiên...", "Agent and Server are running, waiting for the first data...")
_add("fleet_waiting", "⏳ Chưa nhận /fleet/snapshot. Mở khung 'Điều khiển Fleet' ở trên để bật Fleet Server và kết nối robot.",
     "⏳ No /fleet/snapshot yet. Use the 'Fleet control' box above to start the Fleet Server and connect the robot.")
_add("chip_fleet_ok", "Fleet {} AGV", "Fleet {} AGV"); _add("chip_fleet_off", "Fleet tắt", "Fleet off"); _add("chip_fleet_wait", "Fleet chờ", "Fleet waiting")
_add("chip_fleet_tip", "Trạng thái kết nối Fleet. Bấm để mở thẻ Fleet.", "Fleet connection state. Click to open Fleet.")
