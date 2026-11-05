import os

from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton
from PyQt6.QtCore import Qt, QPoint, pyqtSignal
from PyQt6.QtGui import QMouseEvent, QPixmap

from agv_hmi.ui.i18n import tr
from agv_hmi.ui.language_selector import LanguageSelector
from agv_hmi.ui.nav_button import IconButton

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
    estop_clicked = pyqtSignal()
    alarms_clicked = pyqtSignal()
    menu_clicked = pyqtSignal()       # 4.7.0: collapse / expand the sidebar
    focus_clicked = pyqtSignal()      # 4.7.0: focus mode (hide sidebar + side panels)

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

        self._menu_btn = IconButton("menu", 32)
        self._menu_btn.setToolTip(tr("tb_menu_tip"))
        self._menu_btn.clicked.connect(self.menu_clicked)
        lay.addWidget(self._menu_btn)

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

        self._focus_btn = IconButton("focus", 30)
        self._focus_btn.setToolTip(tr("tb_focus_tip"))
        self._focus_btn.clicked.connect(self.focus_clicked)
        lay.addWidget(self._focus_btn)

        self._alarm_btn = QPushButton("🔔")
        self._alarm_btn.setFixedHeight(30)
        self._alarm_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._alarm_btn.clicked.connect(self.alarms_clicked)
        self._alarm_btn.setStyleSheet(
            "QPushButton{background:#21262D;color:#8B949E;border:none;border-radius:6px;padding:0 10px;}"
            "QPushButton:hover{background:#30363D;}")
        lay.addWidget(self._alarm_btn)

        self._estop_btn = QPushButton(tr("estop_btn"))
        self._estop_btn.setFixedHeight(30)
        self._estop_btn.setToolTip(tr("estop_tip"))
        self._estop_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._estop_btn.setStyleSheet(
            "QPushButton{background:#DA3633;color:white;font-weight:800;border:none;"
            "border-radius:6px;padding:0 14px;}QPushButton:hover{background:#F85149;}")
        self._estop_btn.clicked.connect(self.estop_clicked)
        lay.addWidget(self._estop_btn)

        self._lang_selector = LanguageSelector(compact=True)
        self._lang_selector.language_changed.connect(self.language_changed)
        lay.addWidget(self._lang_selector)

        lay.addSpacing(8)

        self._win_btns = {}
        for kind, slot in (("min", self.minimize_clicked),
                           ("max", self.maximize_clicked),
                           ("close", self.close_clicked)):
            b = IconButton(kind, 32)
            b.clicked.connect(slot)
            self._win_btns[kind] = b
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

    def set_maximized(self, on: bool):
        self._win_btns["max"].set_kind("restore" if on else "max")

    def set_focus_mode(self, on: bool):
        self._focus_btn.set_kind("focus_off" if on else "focus")
        self._focus_btn.setToolTip(tr("tb_focus_off_tip") if on else tr("tb_focus_tip"))

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

    _BADGE = {"info": "#79B8FF", "warn": "#E3B341", "error": "#FF7B72", "critical": "#FF4D4D"}

    def set_alarm_badge(self, count: int, worst):
        color = self._BADGE.get(worst, "#8B949E")
        self._alarm_btn.setText(f"🔔 {count}" if count else "🔔")
        self._alarm_btn.setStyleSheet(
            "QPushButton{background:#21262D;border:none;border-radius:6px;padding:0 10px;"
            f"color:{color};font-weight:{'800' if count else '400'};}}"
            "QPushButton:hover{background:#30363D;}")

    def retranslate(self):
        self._menu_btn.setToolTip(tr("tb_menu_tip"))
        self._focus_btn.setToolTip(tr("tb_focus_tip"))
        self._estop_btn.setText(tr("estop_btn"))
        self._estop_btn.setToolTip(tr("estop_tip"))
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