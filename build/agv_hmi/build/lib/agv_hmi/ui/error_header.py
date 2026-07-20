"""
error_header.py — Header báo lỗi robot + Nav2 status.

2 nguồn độc lập:
  - robot_status (/robot_status topic): OK / WARN / ERROR từ robot.
  - nav2_status  (action server status): planner/controller feedback.

Ưu tiên hiển thị: robot_error > nav2_error > nav2_warn > robot_warn > ok.
"""
import json
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QVBoxLayout
from agv_hmi.ui.i18n import tr


class ErrorHeader(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedHeight(40)

        # Robot status (từ /robot_status)
        self._robot_level   = "ok"
        self._robot_message = ""

        # Nav2 status (từ action server)
        self._nav2_level    = "ok"
        self._nav2_message  = ""

        self._build()
        self._refresh()

    def _build(self):
        lay = QHBoxLayout(self)
        lay.setContentsMargins(14, 0, 14, 0)
        lay.setSpacing(6)

        self._icon = QLabel("✅")
        self._icon.setStyleSheet("font-size:14px;background:transparent;")

        col = QVBoxLayout()
        col.setSpacing(0)

        self._msg = QLabel(tr("no_errors"))
        self._msg.setStyleSheet(
            "font-size:12px;font-weight:600;background:transparent;")

        self._sub = QLabel("")
        self._sub.setStyleSheet(
            "font-size:10px;color:#8B949E;background:transparent;")
        self._sub.setVisible(False)

        col.addWidget(self._msg)
        col.addWidget(self._sub)

        lay.addWidget(self._icon)
        lay.addLayout(col)
        lay.addStretch()

    # ── Public API: robot_status ─────────────────────────────────────

    def clear_error(self):
        self._robot_level   = "ok"
        self._robot_message = ""
        self._refresh()

    def set_warning(self, message: str):
        self._robot_level   = "warn"
        self._robot_message = message
        self._refresh()

    def set_error(self, message: str):
        self._robot_level   = "error"
        self._robot_message = message
        self._refresh()

    def update_from_ros(self, status_msg: str):
        s = (status_msg or "").strip()
        if not s:
            self.clear_error(); return

        try:
            d = json.loads(s)
            if isinstance(d, dict):
                level = str(d.get("level",
                    d.get("severity", d.get("status", "")))).lower()
                msg = str(d.get("message",
                    d.get("msg", d.get("error", "")))).strip()
                if level in ("ok","normal","none","0","false"):
                    self.clear_error(); return
                if not msg: msg = s
                if level in ("warn","warning","1"):
                    self.set_warning(msg); return
                self.set_error(msg); return
        except Exception:
            pass

        upper = s.upper()
        if upper in ("OK","NO ERROR","NO ERRORS","NORMAL","NONE","CLEAR"):
            self.clear_error()
        elif upper.startswith("WARN"):
            self.set_warning(s)
        else:
            self.set_error(s)

    # ── Public API: nav2_status ───────────────────────────────────────

    def clear_nav2(self):
        self._nav2_level   = "ok"
        self._nav2_message = ""
        self._refresh()

    def set_nav2_warning(self, message: str):
        self._nav2_level   = "warn"
        self._nav2_message = message
        self._refresh()

    def set_nav2_error(self, message: str):
        self._nav2_level   = "error"
        self._nav2_message = message
        self._refresh()

    # ── Render ───────────────────────────────────────────────────────

    def _refresh(self):
        """
        Tính level/message hiển thị dựa trên cả 2 nguồn.
        Ưu tiên: robot_error > nav2_error > nav2_warn > robot_warn > ok.
        Nếu có cả 2 nguồn cùng level, hiện robot ở dòng chính,
        nav2 ở dòng phụ (nhỏ hơn).
        """
        rl = self._robot_level
        nl = self._nav2_level

        # Xác định level tổng hợp
        if rl == "error":
            top_level = "error"
            top_msg   = self._robot_message
            sub_msg   = (f"Nav2: {self._nav2_message}"
                         if nl != "ok" else "")
        elif nl == "error":
            top_level = "error"
            top_msg   = self._nav2_message
            sub_msg   = (f"Robot: {self._robot_message}"
                         if rl != "ok" else "")
        elif rl == "warn":
            top_level = "warn"
            top_msg   = self._robot_message
            sub_msg   = (f"Nav2: {self._nav2_message}"
                         if nl != "ok" else "")
        elif nl == "warn":
            top_level = "warn"
            top_msg   = self._nav2_message
            sub_msg   = ""
        else:
            top_level = "ok"
            top_msg   = ""
            sub_msg   = ""

        self._apply_style(top_level, top_msg, sub_msg)

    def _apply_style(self, level: str, main_msg: str, sub_msg: str):
        if level == "ok":
            self._icon.setText("✅")
            self._msg.setText(tr("no_errors"))
            self._msg.setStyleSheet(
                "font-size:12px;font-weight:600;"
                "color:#3FB950;background:transparent;")
            self.setStyleSheet(
                "ErrorHeader{background:#1B3629;"
                "border-bottom:1px solid #2EA043;}")
            self._sub.setVisible(False)

        elif level == "warn":
            self._icon.setText("🟡")
            self._msg.setText("Cảnh báo: " + main_msg)
            self._msg.setStyleSheet(
                "font-size:12px;font-weight:600;"
                "color:#E3B341;background:transparent;")
            self.setStyleSheet(
                "ErrorHeader{background:#2E2000;"
                "border-bottom:1px solid #9E6A03;}")
            if sub_msg:
                self._sub.setText(sub_msg)
                self._sub.setVisible(True)
            else:
                self._sub.setVisible(False)

        else:  # error
            self._icon.setText("🔴")
            self._msg.setText(tr("error_prefix") + main_msg)
            self._msg.setStyleSheet(
                "font-size:12px;font-weight:600;"
                "color:#F85149;background:transparent;")
            self.setStyleSheet(
                "ErrorHeader{background:#3D1515;"
                "border-bottom:1px solid #8B0000;}")
            if sub_msg:
                self._sub.setText(sub_msg)
                self._sub.setVisible(True)
            else:
                self._sub.setVisible(False)

    def retranslate(self):
        self._refresh()