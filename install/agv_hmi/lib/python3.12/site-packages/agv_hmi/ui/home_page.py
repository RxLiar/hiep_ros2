import json
import os

from PyQt6.QtWidgets import (
    QScrollArea,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QFrame,
    QPushButton,   # [NEW]
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QPixmap

from agv_hmi.ui.i18n import tr
from agv_hmi.ui.hold_button import HoldButton

LOGO_PATH = "/home/hiep0247/Downloads/TBD_logo.png"

CONN_OFFLINE = "offline"
CONN_IDLE = "idle"
CONN_ONLINE = "online"

CONN_COLOR = {
    CONN_OFFLINE: ("#F85149", "#3D1515"),
    CONN_IDLE: ("#E3B341", "#2E2000"),
    CONN_ONLINE: ("#3FB950", "#1B3629"),
}


class _StatusCard(QFrame):
    clicked = pyqtSignal()

    def __init__(
        self,
        icon: str,
        title: str,
        value: str = "—",
        accent: str = "#58A6FF",
        clickable: bool = False,
    ):
        super().__init__()
        self._accent = accent
        self._clickable = clickable
        self.setMinimumSize(160, 130)

        if clickable:
            self.setCursor(Qt.CursorShape.PointingHandCursor)

        self._build(icon, title, value)
        self._set_style(accent, "#1C2D40")

    def _build(self, icon, title, value):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 14, 16, 14)
        lay.setSpacing(6)

        top = QHBoxLayout()
        self._icon_lbl = QLabel(icon)
        self._icon_lbl.setStyleSheet("font-size:20px;background:transparent;")
        top.addWidget(self._icon_lbl)
        top.addStretch()
        lay.addLayout(top)

        self._title_lbl = QLabel(title)
        self._title_lbl.setStyleSheet(
            "font-size:11px;"
            "font-weight:600;"
            "color:#8B949E;"
            "letter-spacing:0.06em;"
            "background:transparent;"
        )
        lay.addWidget(self._title_lbl)

        self._value_lbl = QLabel(value)
        self._value_lbl.setWordWrap(True)
        self._value_lbl.setStyleSheet(
            "font-size:18px;"
            "font-weight:700;"
            "color:#E6EDF3;"
            "background:transparent;"
        )
        lay.addWidget(self._value_lbl)
        lay.addStretch()

    def _set_style(self, accent: str, bg: str):
        self.setStyleSheet(
            f"QFrame{{"
            f"background:{bg};"
            f"border:1px solid {accent};"
            "border-radius:12px;"
            "}"
            f"QFrame:hover{{border-color:{accent};}}"
        )

    def set_value(
        self,
        value: str,
        color: str = "#E6EDF3",
        bg: str = "#1C2D40",
        accent: str | None = None,
    ):
        self._value_lbl.setText(value)
        self._value_lbl.setStyleSheet(
            f"font-size:18px;"
            f"font-weight:700;"
            f"color:{color};"
            f"background:transparent;"
        )
        self._set_style(accent or self._accent, bg)

    def set_title(self, title: str):
        self._title_lbl.setText(title)

    def set_icon(self, icon: str):
        self._icon_lbl.setText(icon)

    def mousePressEvent(self, e):
        if self._clickable:
            self.clicked.emit()
        super().mousePressEvent(e)


class HomePage(QWidget):
    connection_toggle = pyqtSignal()
    # [NEW] Nút "Đổi người vận hành" — quay về màn hình Login.
    switch_user_requested = pyqtSignal()
    # True = enable drive motors, False = disable (std_srvs/SetBool /motor_enable).
    motor_enable_requested = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self._conn_state = CONN_OFFLINE
        self._agv_moving = False
        self._cargo = [False, False, False, False]
        self._motor_armed = False
        self._plc_online = False
        self._estop = False

        self._build()

        # No update for a few seconds => treat the source as gone.
        self._plc_timer = QTimer()
        self._plc_timer.setSingleShot(True)
        self._plc_timer.timeout.connect(self._on_plc_timeout)
        self._driver_timer = QTimer()
        self._driver_timer.setSingleShot(True)
        self._driver_timer.timeout.connect(self._on_driver_timeout)

        self._conn_timer = QTimer()
        self._conn_timer.setSingleShot(True)
        self._conn_timer.timeout.connect(lambda: self.update_connection(CONN_OFFLINE))
        self._conn_timer.start(5000)

    def _build(self):
        # Scrollable page: the dashboard sits below the status cards on small screens.
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setStyleSheet("QScrollArea{background:transparent;border:none;}"
                             "QScrollArea > QWidget > QWidget{background:transparent;}")
        inner = QWidget()
        scroll.setWidget(inner)
        outer.addWidget(scroll)
        root = QVBoxLayout(inner)
        root.setContentsMargins(28, 24, 28, 28)
        root.setSpacing(22)

        hero = QFrame()
        hero.setStyleSheet(
            "QFrame{"
            "background:#010409;"
            "border:1px solid #21262D;"
            "border-radius:16px;"
            "}"
        )

        hero_lay = QHBoxLayout(hero)
        hero_lay.setContentsMargins(22, 18, 22, 18)
        hero_lay.setSpacing(18)

        self._logo_lbl = QLabel()
        self._logo_lbl.setFixedSize(120, 78)
        self._logo_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._logo_lbl.setStyleSheet(
            "background:#0D1117;"
            "border:1px solid #30363D;"
            "border-radius:12px;"
        )
        self._load_logo()
        hero_lay.addWidget(self._logo_lbl)

        title_col = QVBoxLayout()
        self._title_lbl = QLabel(tr("app_name"))
        self._title_lbl.setStyleSheet(
            "font-size:24px;"
            "font-weight:800;"
            "color:#E6EDF3;"
            "background:transparent;"
        )

        self._sub_lbl = QLabel("Pacific Autonomous Robot HMI · ROS2 Jazzy")
        self._sub_lbl.setStyleSheet(
            "font-size:12px;"
            "color:#8B949E;"
            "background:transparent;"
        )

        title_col.addWidget(self._title_lbl)
        title_col.addWidget(self._sub_lbl)
        title_col.addStretch()

        hero_lay.addLayout(title_col)
        hero_lay.addStretch()

        # Motor enable / disable (KEYA bridge starts with motors disabled).
        self._motor_btn = HoldButton("⏻  " + tr("home_motor_enable"), hold_ms=1000)
        self._motor_btn.confirmed.connect(self._on_motor_hold_confirmed)
        self._motor_btn.setObjectName("BtnPrimary")
        self._motor_btn.setFixedHeight(38)
        self._motor_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._motor_btn.clicked.connect(self._on_motor_btn_clicked)
        hero_lay.addWidget(self._motor_btn, alignment=Qt.AlignmentFlag.AlignVCenter)

        # [NEW] Nút đổi người vận hành, đặt góc phải của hero card.
        self._switch_user_btn = QPushButton("🔄  " + tr("home_switch_user"))
        self._switch_user_btn.setObjectName("BtnPrimary")
        self._switch_user_btn.setFixedHeight(38)
        self._switch_user_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._switch_user_btn.clicked.connect(self.switch_user_requested)
        hero_lay.addWidget(self._switch_user_btn, alignment=Qt.AlignmentFlag.AlignVCenter)

        root.addWidget(hero)

        grid = QGridLayout()
        grid.setSpacing(14)

        self._conn_card = _StatusCard(
            "📡",
            tr("home_connection"),
            tr("status_offline"),
            accent="#E3B341",
            clickable=True,
        )
        self._conn_card.clicked.connect(self.connection_toggle)
        grid.addWidget(self._conn_card, 0, 0)

        self._agv_card = _StatusCard(
            "🤖",
            tr("home_agv_status"),
            tr("home_stopped"),
            accent="#58A6FF",
        )
        grid.addWidget(self._agv_card, 0, 1)

        self._conv_cards: list[_StatusCard] = []
        for i in range(4):
            c = _StatusCard(
                "📦",
                tr("home_conv", i + 1),
                tr("home_no_cargo"),
                accent="#30363D",
            )
            self._conv_cards.append(c)
            grid.addWidget(c, 1 + i // 2, i % 2)

        self._motor_card = _StatusCard(
            "⚙️", tr("home_motor"), tr("home_motor_unknown"), accent="#30363D")
        grid.addWidget(self._motor_card, 3, 0)

        self._safety_card = _StatusCard(
            "📡", tr("home_plc_estop"), tr("home_plc_offline"), accent="#E3B341")
        grid.addWidget(self._safety_card, 3, 1)

        root.addLayout(grid)
        self._root_lay = root
        root.addStretch()

    def _load_logo(self):
        if os.path.exists(LOGO_PATH):
            pix = QPixmap(LOGO_PATH)
            if not pix.isNull():
                self._logo_lbl.setPixmap(
                    pix.scaled(
                        112,
                        72,
                        Qt.AspectRatioMode.KeepAspectRatio,
                        Qt.TransformationMode.SmoothTransformation,
                    )
                )
                return

        self._logo_lbl.setText("TBD")
        self._logo_lbl.setStyleSheet(
            "background:#185FA5;"
            "border-radius:12px;"
            "color:white;"
            "font-size:18px;"
            "font-weight:800;"
            "qproperty-alignment:AlignCenter;"
        )

    def update_connection(self, state: str):
        self._conn_state = state

        self._conn_timer.stop()
        if state != CONN_OFFLINE:
            self._conn_timer.start(5000)

        labels = {
            CONN_OFFLINE: tr("status_offline"),
            CONN_IDLE: tr("status_idle"),
            CONN_ONLINE: tr("status_online"),
        }

        icons = {
            CONN_OFFLINE: "📡",
            CONN_IDLE: "📶",
            CONN_ONLINE: "✅",
        }

        color, bg = CONN_COLOR.get(state, CONN_COLOR[CONN_OFFLINE])
        self._conn_card.set_value(
            labels.get(state, state),
            color=color,
            bg=bg,
            accent=color,
        )
        self._conn_card.set_icon(icons.get(state, "📡"))

    def update_agv_status(self, moving: bool):
        self._agv_moving = moving

        if moving:
            self._agv_card.set_value(
                tr("home_moving"),
                color="#3FB950",
                bg="#1B3629",
                accent="#3FB950",
            )
            self._agv_card.set_icon("🚗")
        else:
            self._agv_card.set_value(
                tr("home_stopped"),
                color="#F85149",
                bg="#1C1C1C",
                accent="#484F58",
            )
            self._agv_card.set_icon("🤖")

    def update_conveyor_cargo(self, belt_id: int, has_cargo: bool):
        if 0 <= belt_id < 4:
            self._cargo[belt_id] = has_cargo
            c = self._conv_cards[belt_id]

            if has_cargo:
                c.set_value(
                    tr("home_has_cargo"),
                    color="#3FB950",
                    bg="#1B3629",
                    accent="#3FB950",
                )
                c.set_icon("📦")
            else:
                c.set_value(
                    tr("home_no_cargo"),
                    color="#8B949E",
                    bg="#161B22",
                    accent="#30363D",
                )
                c.set_icon("⬜")

    def add_dashboard(self, widget):
        """Insert the live dashboard above the stretch at the bottom of the page."""
        self._root_lay.insertWidget(self._root_lay.count() - 1, widget)

    # ── Hardware layer: PLC / e-stop / drive motors ──────────────────

    def _on_motor_btn_clicked(self):
        # Disabling is immediate; ENABLING needs a 1 s hold (see HoldButton).
        if not self._motor_armed:
            return
        self._motor_btn.setEnabled(False)
        QTimer.singleShot(3000, lambda: self._motor_btn.setEnabled(True))
        self.motor_enable_requested.emit(False)

    def _on_motor_hold_confirmed(self):
        if self._motor_armed:
            return
        self._motor_btn.setEnabled(False)
        QTimer.singleShot(3000, lambda: self._motor_btn.setEnabled(True))
        self.motor_enable_requested.emit(True)

    def motor_request_done(self):
        self._motor_btn.setEnabled(True)

    def _sync_motor_button(self):
        key = "home_motor_disable" if self._motor_armed else "home_motor_enable"
        self._motor_btn.setText("⏻  " + tr(key))

    def update_plc_connection(self, state: str):
        self._plc_online = (state == CONN_ONLINE)
        self._plc_timer.stop()
        if self._plc_online:
            self._plc_timer.start(5000)
        self._refresh_safety_card()

    def update_estop(self, active: bool):
        self._estop = bool(active)
        self._refresh_safety_card()

    def _on_plc_timeout(self):
        self._plc_online = False
        self._refresh_safety_card()

    def _refresh_safety_card(self):
        if not self._plc_online:
            # With the PLC gone the e-stop state is unknown: never show "OK".
            self._safety_card.set_value(
                tr("home_plc_offline"), color="#E3B341", bg="#2E2000", accent="#E3B341")
            self._safety_card.set_icon("📡")
        elif self._estop:
            self._safety_card.set_value(
                tr("home_estop_active"), color="#F85149", bg="#3D1515", accent="#F85149")
            self._safety_card.set_icon("🛑")
        else:
            self._safety_card.set_value(
                tr("home_estop_ok"), color="#3FB950", bg="#1B3629", accent="#3FB950")
            self._safety_card.set_icon("✅")

    def update_driver_status(self, payload: str):
        try:
            d = json.loads(payload or "{}")
            if not isinstance(d, dict):
                return
        except Exception:
            return
        self._driver_timer.stop()
        self._driver_timer.start(3000)

        online = bool(d.get("online"))
        armed = bool(d.get("armed"))
        estop = bool(d.get("estop"))
        fault = d.get("fault") or [0, 0]
        try:
            fault_codes = [int(x) for x in fault]
        except (TypeError, ValueError):
            fault_codes = [0, 0]
        self._motor_armed = armed and online

        if not online:
            self._motor_card.set_value(
                tr("home_motor_driver_offline"), color="#F85149", bg="#3D1515", accent="#F85149")
            self._motor_card.set_icon("⚠️")
        elif any(fault_codes):
            codes = "/".join(f"{c:04X}" for c in fault_codes)
            self._motor_card.set_value(
                f"{tr('home_motor_fault')} {codes}", color="#F85149", bg="#3D1515", accent="#F85149")
            self._motor_card.set_icon("⚠️")
        elif estop:
            self._motor_card.set_value(
                tr("home_estop_active"), color="#F85149", bg="#3D1515", accent="#F85149")
            self._motor_card.set_icon("🛑")
        elif armed:
            self._motor_card.set_value(
                tr("home_motor_on"), color="#3FB950", bg="#1B3629", accent="#3FB950")
            self._motor_card.set_icon("⚙️")
        else:
            self._motor_card.set_value(
                tr("home_motor_off"), color="#E3B341", bg="#2E2000", accent="#E3B341")
            self._motor_card.set_icon("⚙️")
        self._sync_motor_button()

    def _on_driver_timeout(self):
        # No /keya_driver_status (e.g. ESP32 test model): unknown, not "off".
        self._motor_armed = False
        self._motor_card.set_value(
            tr("home_motor_unknown"), color="#8B949E", bg="#161B22", accent="#30363D")
        self._motor_card.set_icon("⚙️")
        self._sync_motor_button()

    def retranslate(self):
        self._title_lbl.setText(tr("app_name"))
        self._motor_card.set_title(tr("home_motor"))
        self._safety_card.set_title(tr("home_plc_estop"))
        self._sync_motor_button()
        self._refresh_safety_card()
        self._conn_card.set_title(tr("home_connection"))
        self._agv_card.set_title(tr("home_agv_status"))
        self._switch_user_btn.setText("🔄  " + tr("home_switch_user"))

        for i, c in enumerate(self._conv_cards):
            c.set_title(tr("home_conv", i + 1))

        self.update_connection(self._conn_state)
        self.update_agv_status(self._agv_moving)

        for i, has in enumerate(self._cargo):
            self.update_conveyor_cargo(i, has)
