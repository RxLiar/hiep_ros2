import os

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QFrame,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QPixmap

from agv_hmi.ui.i18n import tr

LOGO_PATH = "/home/hiep0247/Downloads/logo_busan.png"

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

    def __init__(self):
        super().__init__()
        self._conn_state = CONN_OFFLINE
        self._agv_moving = False
        self._cargo = [False, False, False, False]

        self._build()

        self._conn_timer = QTimer()
        self._conn_timer.setSingleShot(True)
        self._conn_timer.timeout.connect(lambda: self.update_connection(CONN_OFFLINE))
        self._conn_timer.start(5000)

    def _build(self):
        root = QVBoxLayout(self)
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

        self._sub_lbl = QLabel("Busan Autonomous Robot HMI · ROS2 Jazzy")
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

        root.addLayout(grid)
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

        self._logo_lbl.setText("BUSAN")
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

    def retranslate(self):
        self._title_lbl.setText(tr("app_name"))
        self._conn_card.set_title(tr("home_connection"))
        self._agv_card.set_title(tr("home_agv_status"))

        for i, c in enumerate(self._conv_cards):
            c.set_title(tr("home_conv", i + 1))

        self.update_connection(self._conn_state)
        self.update_agv_status(self._agv_moving)

        for i, has in enumerate(self._cargo):
            self.update_conveyor_cargo(i, has)