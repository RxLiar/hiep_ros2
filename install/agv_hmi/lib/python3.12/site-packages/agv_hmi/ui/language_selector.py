"""Reusable language selector for the AGV HMI.

compact=True:
    Small QComboBox suitable for TitleBar and Sidebar.
compact=False:
    Five flag buttons arranged in a responsive grid for Settings.
"""
import math

from PyQt6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QGridLayout,
    QPushButton,
    QComboBox,
)
from PyQt6.QtCore import Qt, QSize, QRectF, QPointF, pyqtSignal
from PyQt6.QtGui import (
    QPixmap,
    QPainter,
    QColor,
    QPen,
    QBrush,
    QIcon,
    QPolygonF,
)

from agv_hmi.ui.i18n import (
    LANGUAGES,
    SUPPORTED_LANGS,
    get_lang,
    set_lang,
)


def _star_points(cx: float, cy: float, r_outer: float, r_inner: float):
    points = []
    start = -math.pi / 2
    for i in range(10):
        radius = r_outer if i % 2 == 0 else r_inner
        angle = start + i * math.pi / 5
        points.append(QPointF(
            cx + radius * math.cos(angle),
            cy + radius * math.sin(angle),
        ))
    return points


def _draw_vietnam_flag(p: QPainter, w: int, h: int):
    p.fillRect(0, 0, w, h, QColor("#DA251D"))
    polygon = QPolygonF(_star_points(w / 2, h / 2, h * 0.28, h * 0.11))
    p.setBrush(QBrush(QColor("#FFFF00")))
    p.setPen(Qt.PenStyle.NoPen)
    p.drawPolygon(polygon)


def _draw_uk_flag(p: QPainter, w: int, h: int):
    p.fillRect(0, 0, w, h, QColor("#012169"))

    p.setPen(QPen(QColor("white"), max(3, int(h * 0.20))))
    p.drawLine(0, 0, w, h)
    p.drawLine(w, 0, 0, h)

    p.setPen(QPen(QColor("#C8102E"), max(2, int(h * 0.10))))
    p.drawLine(0, 0, w, h)
    p.drawLine(w, 0, 0, h)

    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QBrush(QColor("white")))
    p.drawRect(int(w * 0.42), 0, int(w * 0.16), h)
    p.drawRect(0, int(h * 0.38), w, int(h * 0.24))

    p.setBrush(QBrush(QColor("#C8102E")))
    p.drawRect(int(w * 0.46), 0, int(w * 0.08), h)
    p.drawRect(0, int(h * 0.43), w, int(h * 0.14))


def _draw_korea_flag(p: QPainter, w: int, h: int):
    p.fillRect(0, 0, w, h, QColor("white"))

    cx = w / 2
    cy = h / 2
    radius = h * 0.23
    rect = QRectF(cx - radius, cy - radius, radius * 2, radius * 2)

    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QBrush(QColor("#CD2E3A")))
    p.drawPie(rect, 0, 180 * 16)
    p.setBrush(QBrush(QColor("#0047A0")))
    p.drawPie(rect, 180 * 16, 180 * 16)

    p.setPen(QPen(QColor("black"), max(2, int(h * 0.055))))
    x1, x2 = int(w * 0.18), int(w * 0.34)
    x3, x4 = int(w * 0.66), int(w * 0.82)
    y_top, y_bot = int(h * 0.26), int(h * 0.74)

    p.drawLine(x1, y_top, x2, int(y_top - h * 0.08))
    p.drawLine(x1, int(y_top + h * 0.08), x2, y_top)
    p.drawLine(x3, int(y_top - h * 0.08), x4, y_top)
    p.drawLine(x3, y_top, x4, int(y_top + h * 0.08))
    p.drawLine(x1, int(y_bot - h * 0.08), x2, y_bot)
    p.drawLine(x1, y_bot, x2, int(y_bot + h * 0.08))
    p.drawLine(x3, y_bot, x4, int(y_bot - h * 0.08))
    p.drawLine(x3, int(y_bot + h * 0.08), x4, y_bot)


def _draw_japan_flag(p: QPainter, w: int, h: int):
    p.fillRect(0, 0, w, h, QColor("white"))
    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QBrush(QColor("#BC002D")))
    radius = h * 0.28
    p.drawEllipse(QPointF(w / 2, h / 2), radius, radius)


def _draw_china_flag(p: QPainter, w: int, h: int):
    p.fillRect(0, 0, w, h, QColor("#DE2910"))
    polygon = QPolygonF(_star_points(
        w * 0.25,
        h * 0.32,
        h * 0.21,
        h * 0.085,
    ))
    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QBrush(QColor("#FFDE00")))
    p.drawPolygon(polygon)


def make_flag_icon(code: str, width: int = 30, height: int = 20) -> QIcon:
    pixmap = QPixmap(width, height)
    pixmap.fill(Qt.GlobalColor.transparent)

    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)

    code = str(code).lower()
    drawers = {
        "vi": _draw_vietnam_flag,
        "en": _draw_uk_flag,
        "ko": _draw_korea_flag,
        "ja": _draw_japan_flag,
        "zh": _draw_china_flag,
    }
    drawer = drawers.get(code)
    if drawer is None:
        painter.fillRect(0, 0, width, height, QColor("#30363D"))
    else:
        drawer(painter, width, height)

    painter.setPen(QPen(QColor("#30363D"), 1))
    painter.setBrush(Qt.BrushStyle.NoBrush)
    painter.drawRoundedRect(0, 0, width - 1, height - 1, 3, 3)
    painter.end()
    return QIcon(pixmap)


class FlagLangButton(QPushButton):
    def __init__(self, code: str, short: str, label: str):
        super().__init__()
        self.code = code
        self.short = short
        self.label = label

        self.setCheckable(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setIcon(make_flag_icon(code, 32, 22))
        self.setIconSize(QSize(32, 22))
        self.setText(f"  {short}   {label}")
        self.setToolTip(label)
        self.setFixedHeight(40)
        self.setMinimumWidth(150)
        self._apply_style(False)

    def _apply_style(self, checked: bool):
        if checked:
            self.setStyleSheet(
                "QPushButton{"
                "background:#1C2D40;"
                "border:2px solid #58A6FF;"
                "border-radius:8px;"
                "color:#58A6FF;"
                "font-size:12px;"
                "font-weight:700;"
                "text-align:left;"
                "padding:4px 10px;"
                "}"
            )
        else:
            self.setStyleSheet(
                "QPushButton{"
                "background:#161B22;"
                "border:1px solid #30363D;"
                "border-radius:8px;"
                "color:#C9D1D9;"
                "font-size:12px;"
                "font-weight:600;"
                "text-align:left;"
                "padding:4px 10px;"
                "}"
                "QPushButton:hover{"
                "border-color:#58A6FF;"
                "background:#1A2030;"
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
        self._combo: QComboBox | None = None
        self._build()
        self.set_current(get_lang(), emit=False)

    def _build(self):
        if self.compact:
            layout = QHBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)

            self._combo = QComboBox()
            self._combo.setObjectName("LanguageCombo")
            self._combo.setFixedHeight(30)
            self._combo.setMinimumWidth(86)
            self._combo.setIconSize(QSize(28, 19))
            self._combo.setCursor(Qt.CursorShape.PointingHandCursor)

            for meta in LANGUAGES:
                self._combo.addItem(
                    make_flag_icon(meta["code"], 28, 19),
                    meta["short"],
                    meta["code"],
                )
                index = self._combo.count() - 1
                self._combo.setItemData(index, meta["label"], Qt.ItemDataRole.ToolTipRole)

            self._combo.currentIndexChanged.connect(self._on_combo_changed)
            layout.addWidget(self._combo)
            return

        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(8)

        for index, meta in enumerate(LANGUAGES):
            button = FlagLangButton(
                meta["code"],
                meta["short"],
                meta["label"],
            )
            button.clicked.connect(
                lambda _checked=False, code=meta["code"]: self.set_current(code)
            )
            self._buttons.append(button)
            layout.addWidget(button, index // 3, index % 3)

        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)

    def _on_combo_changed(self, index: int):
        if self._combo is None or index < 0:
            return
        code = str(self._combo.itemData(index) or "")
        self.set_current(code, emit=True)

    def set_current(self, code: str, emit: bool = True):
        code = str(code or "").strip().lower()
        if code not in SUPPORTED_LANGS:
            return
        if not set_lang(code):
            return

        for button in self._buttons:
            button.setChecked(button.code == code)

        if self._combo is not None:
            for index in range(self._combo.count()):
                if self._combo.itemData(index) == code:
                    self._combo.blockSignals(True)
                    self._combo.setCurrentIndex(index)
                    self._combo.blockSignals(False)
                    break

        if emit:
            self.language_changed.emit(code)

    def retranslate(self):
        """Native language names do not change; only sync current selection."""
        current = get_lang()
        for button in self._buttons:
            button.setChecked(button.code == current)
        if self._combo is not None:
            for index in range(self._combo.count()):
                if self._combo.itemData(index) == current:
                    self._combo.blockSignals(True)
                    self._combo.setCurrentIndex(index)
                    self._combo.blockSignals(False)
                    break
