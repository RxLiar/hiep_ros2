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
_add("sf_lidar_box", "Vị trí lidar (so với base_link)", "Lidar pose (in base_link)", "라이다 위치", "LiDAR位置", "雷达位置")
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
