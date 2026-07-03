import math

from PyQt6.QtWidgets import QWidget, QHBoxLayout, QPushButton
from PyQt6.QtCore import Qt, QSize, QRectF, pyqtSignal
from PyQt6.QtGui import (
    QPixmap,
    QPainter,
    QColor,
    QPen,
    QBrush,
    QIcon,
    QPolygonF,
)

from agv_hmi.ui.i18n import LANGUAGES, get_lang, set_lang


def _star_points(cx: float, cy: float, r_outer: float, r_inner: float):
    pts = []
    start = -math.pi / 2
    for i in range(10):
        r = r_outer if i % 2 == 0 else r_inner
        a = start + i * math.pi / 5
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _draw_vietnam_flag(p: QPainter, w: int, h: int):
    p.fillRect(0, 0, w, h, QColor("#DA251D"))

    pts = _star_points(w / 2, h / 2, h * 0.28, h * 0.11)
    poly = QPolygonF()
    for x, y in pts:
        poly.append(__import__("PyQt6.QtCore").QtCore.QPointF(x, y))

    p.setBrush(QBrush(QColor("#FFFF00")))
    p.setPen(Qt.PenStyle.NoPen)
    p.drawPolygon(poly)


def _draw_uk_flag(p: QPainter, w: int, h: int):
    p.fillRect(0, 0, w, h, QColor("#012169"))

    # White diagonals
    p.setPen(QPen(QColor("white"), max(3, int(h * 0.20))))
    p.drawLine(0, 0, w, h)
    p.drawLine(w, 0, 0, h)

    # Red diagonals
    p.setPen(QPen(QColor("#C8102E"), max(2, int(h * 0.10))))
    p.drawLine(0, 0, w, h)
    p.drawLine(w, 0, 0, h)

    # White cross
    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QBrush(QColor("white")))
    p.drawRect(int(w * 0.42), 0, int(w * 0.16), h)
    p.drawRect(0, int(h * 0.38), w, int(h * 0.24))

    # Red cross
    p.setBrush(QBrush(QColor("#C8102E")))
    p.drawRect(int(w * 0.46), 0, int(w * 0.08), h)
    p.drawRect(0, int(h * 0.43), w, int(h * 0.14))


def _draw_korea_flag(p: QPainter, w: int, h: int):
    p.fillRect(0, 0, w, h, QColor("white"))

    cx = w / 2
    cy = h / 2
    r = h * 0.23
    rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)

    # Taegeuk simplified
    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QBrush(QColor("#CD2E3A")))
    p.drawPie(rect, 0, 180 * 16)
    p.setBrush(QBrush(QColor("#0047A0")))
    p.drawPie(rect, 180 * 16, 180 * 16)

    # Bars simplified
    p.setPen(QPen(QColor("black"), max(2, int(h * 0.055))))

    x1 = int(w * 0.18)
    x2 = int(w * 0.34)
    x3 = int(w * 0.66)
    x4 = int(w * 0.82)
    y_top = int(h * 0.26)
    y_bot = int(h * 0.74)

    p.drawLine(x1, y_top, x2, int(y_top - h * 0.08))
    p.drawLine(x1, int(y_top + h * 0.08), x2, y_top)

    p.drawLine(x3, int(y_top - h * 0.08), x4, y_top)
    p.drawLine(x3, y_top, x4, int(y_top + h * 0.08))

    p.drawLine(x1, int(y_bot - h * 0.08), x2, y_bot)
    p.drawLine(x1, y_bot, x2, int(y_bot + h * 0.08))

    p.drawLine(x3, y_bot, x4, int(y_bot - h * 0.08))
    p.drawLine(x3, int(y_bot + h * 0.08), x4, y_bot)


def make_flag_icon(code: str, width: int = 30, height: int = 20) -> QIcon:
    pix = QPixmap(width, height)
    pix.fill(Qt.GlobalColor.transparent)

    p = QPainter(pix)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)

    code = code.lower()
    if code == "vi":
        _draw_vietnam_flag(p, width, height)
    elif code == "en":
        _draw_uk_flag(p, width, height)
    elif code == "ko":
        _draw_korea_flag(p, width, height)
    else:
        p.fillRect(0, 0, width, height, QColor("#30363D"))

    p.setPen(QPen(QColor("#30363D"), 1))
    p.setBrush(Qt.BrushStyle.NoBrush)
    p.drawRoundedRect(0, 0, width - 1, height - 1, 3, 3)
    p.end()

    return QIcon(pix)


class FlagLangButton(QPushButton):
    def __init__(self, code: str, label: str, compact: bool = True):
        super().__init__()
        self.code = code
        self.label = label
        self.compact = compact

        self.setCheckable(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setIcon(make_flag_icon(code))
        self.setIconSize(QSize(30, 20))
        self.setToolTip(label)

        if compact:
            self.setText("")
            self.setFixedSize(42, 30)
        else:
            self.setText(code.upper())
            self.setFixedHeight(30)
            self.setMinimumWidth(64)

        self._apply_style(False)

    def _apply_style(self, checked: bool):
        if checked:
            self.setStyleSheet(
                "QPushButton{"
                "background:#1C2D40;"
                "border:1.5px solid #58A6FF;"
                "border-radius:7px;"
                "color:#58A6FF;"
                "font-size:11px;"
                "font-weight:700;"
                "padding:2px 6px;"
                "}"
            )
        else:
            self.setStyleSheet(
                "QPushButton{"
                "background:#161B22;"
                "border:1px solid #30363D;"
                "border-radius:7px;"
                "color:#8B949E;"
                "font-size:11px;"
                "font-weight:600;"
                "padding:2px 6px;"
                "}"
                "QPushButton:hover{"
                "border-color:#58A6FF;"
                "color:#E6EDF3;"
                "}"
            )

    def setChecked(self, checked: bool):
        super().setChecked(checked)
        self._apply_style(checked)


class LanguageSelector(QWidget):
    language_changed = pyqtSignal(str)

    def __init__(self, compact: bool = True, parent=None):
        super().__init__(parent)
        self.compact = compact
        self._buttons: list[FlagLangButton] = []
        self._build()
        self.set_current(get_lang(), emit=False)

    def _build(self):
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(5)

        for meta in LANGUAGES:
            code = meta["code"]
            label = meta["label"]
            btn = FlagLangButton(code, label, compact=self.compact)
            btn.clicked.connect(lambda _, c=code: self.set_current(c, emit=True))
            self._buttons.append(btn)
            lay.addWidget(btn)

        lay.addStretch()

    def set_current(self, code: str, emit: bool = True):
        if code not in ("vi", "en", "ko"):
            return

        set_lang(code)

        for btn in self._buttons:
            btn.setChecked(btn.code == code)

        if emit:
            self.language_changed.emit(code)

    def retranslate(self):
        current = get_lang()
        for btn in self._buttons:
            btn.setChecked(btn.code == current)
            btn.setToolTip(btn.label)