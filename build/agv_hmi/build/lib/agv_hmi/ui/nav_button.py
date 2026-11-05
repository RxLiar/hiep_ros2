"""nav_button.py - animated sidebar button + small painted icon buttons (4.7.0).

Painted by hand (no QSS) so hover / active states fade smoothly and the button works as an
icon-only rail item when the sidebar is collapsed.
"""
from __future__ import annotations

from PyQt6.QtCore import QPointF, QRectF, QSize, Qt, QVariantAnimation
from PyQt6.QtGui import QColor, QFont, QFontMetrics, QPainter, QPen
from PyQt6.QtWidgets import QAbstractButton, QPushButton, QSizePolicy

from agv_hmi.ui import anim


class _Fader:
    """Tiny helper: animate a float attribute 0..1 on a widget."""

    def __init__(self, widget, attr: str, ms: int):
        self._w, self._attr, self._ms = widget, attr, ms
        self._a = QVariantAnimation(widget)
        self._a.setEasingCurve(anim.EASE_OUT)
        self._a.valueChanged.connect(self._on)

    def _on(self, v):
        setattr(self._w, self._attr, float(v))
        self._w.update()

    def to(self, target: float, animate: bool = True):
        self._a.stop()
        d = anim.dur(self._ms) if animate else 0
        if d == 0:
            setattr(self._w, self._attr, target)
            self._w.update()
            return
        self._a.setDuration(d)
        self._a.setStartValue(getattr(self._w, self._attr))
        self._a.setEndValue(target)
        self._a.start()


class NavButton(QPushButton):
    """Sidebar entry: icon + label, animated hover and active indicator."""

    def __init__(self, icon: str, label: str, parent=None):
        super().__init__(parent)
        self.setObjectName("NavBtnAnim")
        self._icon, self._label = icon, label
        self._compact = False
        self._hover = 0.0
        self._act = 0.0
        self._hover_f = _Fader(self, "_hover", anim.DUR_FAST)
        self._act_f = _Fader(self, "_act", anim.DUR_MED)
        self.setFixedHeight(38)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)

    # API ------------------------------------------------------------
    def label(self) -> str:
        return self._label

    def set_label(self, text: str):
        self._label = text
        if self._compact:
            self.setToolTip(text)
        self.update()

    def set_icon_text(self, icon: str):
        self._icon = icon
        self.update()

    def set_compact(self, compact: bool):
        self._compact = bool(compact)
        self.setToolTip(self._label if compact else "")
        self.update()

    def set_active(self, on: bool, animate: bool = True):
        self.setProperty("active", "true" if on else "false")
        self._act_f.to(1.0 if on else 0.0, animate)

    def is_active(self) -> bool:
        return self.property("active") == "true"

    def sizeHint(self) -> QSize:
        return QSize(180, 38)

    # events ---------------------------------------------------------
    def enterEvent(self, e):
        self._hover_f.to(1.0)
        super().enterEvent(e)

    def leaveEvent(self, e):
        self._hover_f.to(0.0)
        super().leaveEvent(e)

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.TextAntialiasing)
        c = anim.theme_colors()
        h = max(self._hover, 0.6 if self.isDown() else 0.0)
        a = self._act
        r = QRectF(self.rect()).adjusted(6, 2, -6, -2)

        if h > 0.01 and a < 0.99:
            bg = QColor(c["hover"])
            bg.setAlphaF(min(1.0, h * (1.0 - a)))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(bg)
            p.drawRoundedRect(r, 8, 8)
        if a > 0.01:
            bg = QColor(c["active"])
            bg.setAlphaF(a)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(bg)
            p.drawRoundedRect(r, 8, 8)
            bar_h = r.height() * 0.6 * a
            p.setBrush(c["accent"])
            p.drawRoundedRect(QRectF(2.0, r.center().y() - bar_h / 2, 3.0, bar_h), 1.5, 1.5)

        col = anim.mix(anim.mix(c["muted"], c["text"], h), c["accent"], a)
        p.setPen(col)

        f_icon = QFont(self.font())
        f_icon.setPixelSize(16)
        p.setFont(f_icon)
        if self._compact:
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, self._icon)
            return
        icon_rect = QRectF(r.left() + 8, r.top(), 28, r.height())
        p.drawText(icon_rect, Qt.AlignmentFlag.AlignCenter, self._icon)

        f = QFont(self.font())
        f.setPixelSize(13)
        f.setWeight(QFont.Weight.DemiBold if a > 0.5 else QFont.Weight.Normal)
        p.setFont(f)
        tx = icon_rect.right() + 6
        text = QFontMetrics(f).elidedText(
            self._label, Qt.TextElideMode.ElideRight, int(r.right() - tx - 6))
        p.drawText(QRectF(tx, r.top(), r.right() - tx, r.height()),
                   Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, text)


class IconButton(QAbstractButton):
    """Small painted icon button: menu, focus, focus_off, min, max, restore, close."""

    def __init__(self, kind: str, size: int = 32, parent=None):
        super().__init__(parent)
        self._kind = kind
        self._hover = 0.0
        self._hover_f = _Fader(self, "_hover", anim.DUR_FAST)
        self.setFixedSize(size, size)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)

    def set_kind(self, kind: str):
        if kind != self._kind:
            self._kind = kind
            self.update()

    def enterEvent(self, e):
        self._hover_f.to(1.0)
        super().enterEvent(e)

    def leaveEvent(self, e):
        self._hover_f.to(0.0)
        super().leaveEvent(e)

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c = anim.theme_colors()
        danger = self._kind == "close"
        h = max(self._hover, 0.7 if self.isDown() else 0.0)
        base = c["card"]
        hi = c["danger"] if danger else c["hover"]
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(anim.mix(base, hi, h))
        p.drawRoundedRect(r, 7, 7)

        fg = anim.mix(c["muted"], QColor("#FFFFFF") if danger else c["text"], h)
        pen = QPen(fg, 1.7)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.translate(self.width() / 2.0, self.height() / 2.0)
        k = self._kind
        if k == "menu":
            for y in (-4.5, 0, 4.5):
                p.drawLine(QPointF(-6, y), QPointF(6, y))
        elif k in ("focus", "focus_off"):
            # focus: corner brackets on the outside (expand); focus_off: brackets turned inwards
            corner = 6.0 if k == "focus" else 2.5
            arm = 3.5
            for sx, sy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
                cx, cy = sx * corner, sy * corner
                d = -1.0 if k == "focus" else 1.0
                p.drawPolyline([QPointF(cx + sx * arm * d, cy), QPointF(cx, cy),
                                QPointF(cx, cy + sy * arm * d)])
        elif k == "min":
            p.drawLine(QPointF(-5, 3), QPointF(5, 3))
        elif k == "max":
            p.drawRoundedRect(QRectF(-5, -5, 10, 10), 1.5, 1.5)
        elif k == "restore":
            p.drawRoundedRect(QRectF(-5, -2, 7, 7), 1.5, 1.5)
            p.drawPolyline([QPointF(-2, -5), QPointF(5, -5), QPointF(5, 2)])
        elif k == "close":
            p.drawLine(QPointF(-4.5, -4.5), QPointF(4.5, 4.5))
            p.drawLine(QPointF(-4.5, 4.5), QPointF(4.5, -4.5))
