import os

from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton
from PyQt6.QtCore import Qt, QPoint, pyqtSignal
from PyQt6.QtGui import QMouseEvent, QPixmap

from agv_hmi.ui.i18n import tr
from agv_hmi.ui.language_selector import LanguageSelector

LOGO_PATH = "/home/hiep0247/Downloads/TBD_logo.png"

CONN_COLORS = {
    "offline": "#F85149",
    "idle": "#E3B341",
    "online": "#3FB950",
}


class TitleBar(QWidget):
    minimize_clicked = pyqtSignal()
    maximize_clicked = pyqtSignal()
    close_clicked = pyqtSignal()
    language_changed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(48)
        self.setObjectName("TitleBar")
        self._drag_pos: QPoint | None = None
        self._main_win = parent
        self._connection_state = "offline"
        self._build()

    def _build(self):
        lay = QHBoxLayout(self)
        lay.setContentsMargins(14, 0, 10, 0)
        lay.setSpacing(8)

        self._logo = QLabel()
        self._logo.setFixedSize(34, 34)
        self._logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._logo.setStyleSheet(
            "background:#0D1117;"
            "border:1px solid #30363D;"
            "border-radius:7px;"
        )
        self._set_logo()
        lay.addWidget(self._logo)

        self._name_lbl = QLabel(tr("app_name"))
        self._name_lbl.setStyleSheet(
            "font-size:13px;"
            "font-weight:700;"
            "color:#E6EDF3;"
            "background:transparent;"
        )
        lay.addWidget(self._name_lbl)

        self._page_lbl = QLabel("")
        self._page_lbl.setStyleSheet(
            "font-size:13px;"
            "color:#8B949E;"
            "background:transparent;"
        )
        lay.addWidget(self._page_lbl)

        lay.addStretch()

        self._conn_dot = QLabel("●")
        self._conn_dot.setStyleSheet("color:#F85149;font-size:10px;background:transparent;")
        self._conn_lbl = QLabel(tr("status_offline"))
        self._conn_lbl.setStyleSheet("font-size:11px;color:#F85149;background:transparent;")
        lay.addWidget(self._conn_dot)
        lay.addWidget(self._conn_lbl)

        sep1 = QLabel("|")
        sep1.setStyleSheet("color:#30363D;background:transparent;")
        lay.addWidget(sep1)

        self._bat_icon = QLabel("🔋")
        self._bat_icon.setStyleSheet("font-size:14px;background:transparent;")
        self._bat_lbl = QLabel("—%")
        self._bat_lbl.setStyleSheet("font-size:11px;color:#8B949E;background:transparent;")
        lay.addWidget(self._bat_icon)
        lay.addWidget(self._bat_lbl)

        ros_chip = QLabel("ROS2 Jazzy")
        ros_chip.setObjectName("ChipNormal")
        lay.addWidget(ros_chip)

        sep2 = QLabel("|")
        sep2.setStyleSheet("color:#30363D;background:transparent;")
        lay.addWidget(sep2)

        self._lang_selector = LanguageSelector(compact=True)
        self._lang_selector.language_changed.connect(self.language_changed)
        lay.addWidget(self._lang_selector)

        lay.addSpacing(8)

        for sym, slot in [
            ("⎯", self.minimize_clicked),
            ("□", self.maximize_clicked),
            ("✕", self.close_clicked),
        ]:
            b = QPushButton(sym)
            b.setFixedSize(32, 32)
            is_close = sym == "✕"
            b.setStyleSheet(
                f"QPushButton{{"
                f"background:{'#8B0000' if is_close else '#21262D'};"
                "border:none;"
                "border-radius:6px;"
                "color:#8B949E;"
                "font-size:13px;"
                "}"
                f"QPushButton:hover{{"
                f"background:{'#B22222' if is_close else '#30363D'};"
                "color:#E6EDF3;"
                "}"
            )
            b.clicked.connect(slot)
            lay.addWidget(b)

    def _set_logo(self):
        if os.path.exists(LOGO_PATH):
            pix = QPixmap(LOGO_PATH)
            if not pix.isNull():
                self._logo.setPixmap(
                    pix.scaled(
                        30,
                        30,
                        Qt.AspectRatioMode.KeepAspectRatio,
                        Qt.TransformationMode.SmoothTransformation,
                    )
                )
                return

        self._logo.setText("B")
        self._logo.setStyleSheet(
            "background:#185FA5;"
            "border-radius:7px;"
            "color:white;"
            "font-size:18px;"
            "font-weight:800;"
            "qproperty-alignment:AlignCenter;"
        )

    def set_page_title(self, title: str):
        self._page_lbl.setText(f"  /  {title}" if title else "")

    def set_battery(self, pct: int):
        if pct < 0:
            self._bat_lbl.setText("—%")
            self._bat_icon.setText("🔋")
            self._bat_lbl.setStyleSheet("font-size:11px;color:#8B949E;background:transparent;")
            return

        self._bat_lbl.setText(f"{pct}%")

        if pct > 60:
            color = "#3FB950"
            icon = "🔋"
        elif pct > 20:
            color = "#E3B341"
            icon = "🪫"
        else:
            color = "#F85149"
            icon = "🪫"

        self._bat_icon.setText(icon)
        self._bat_lbl.setStyleSheet(f"font-size:11px;color:{color};background:transparent;")

    def set_connection(self, state: str):
        self._connection_state = state

        color = CONN_COLORS.get(state, "#F85149")
        labels = {
            "offline": tr("status_offline"),
            "idle": tr("status_idle"),
            "online": tr("status_online"),
        }

        self._conn_dot.setStyleSheet(f"color:{color};font-size:10px;background:transparent;")
        self._conn_lbl.setText(labels.get(state, state))
        self._conn_lbl.setStyleSheet(f"font-size:11px;color:{color};background:transparent;")

    def retranslate(self):
        self._name_lbl.setText(tr("app_name"))
        self.set_connection(self._connection_state)
        self._lang_selector.retranslate()

    def mousePressEvent(self, e: QMouseEvent):
        if e.button() == Qt.MouseButton.LeftButton:
            self._drag_pos = (
                e.globalPosition().toPoint() - self._main_win.frameGeometry().topLeft()
                if self._main_win
                else e.globalPosition().toPoint()
            )

    def mouseMoveEvent(self, e: QMouseEvent):
        if self._drag_pos and e.buttons() == Qt.MouseButton.LeftButton:
            if self._main_win:
                self._main_win.move(e.globalPosition().toPoint() - self._drag_pos)

    def mouseReleaseEvent(self, e: QMouseEvent):
        self._drag_pos = None

    def mouseDoubleClickEvent(self, e: QMouseEvent):
        self.maximize_clicked.emit()