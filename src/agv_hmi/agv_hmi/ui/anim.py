"""anim.py - small animation / theme helpers shared by the 4.7.0 UI widgets.

* ``enabled()``  - animations can be switched off in Settings (slow HMI PCs).
* ``fade_in()``  - short opacity fade used for page changes and toasts.
* ``AnimatedStack`` - QStackedWidget that fades the new page in.
* ``theme_colors()`` - colours for custom-painted widgets (follows the active QSS theme).
"""
from __future__ import annotations

from PyQt6.QtCore import QEasingCurve, QPropertyAnimation
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QApplication, QGraphicsOpacityEffect, QStackedWidget, QWidget

from agv_hmi.core import prefs

DUR_FAST = 130
DUR_MED = 230
DUR_SLOW = 320
EASE_OUT = QEasingCurve.Type.OutCubic
EASE_INOUT = QEasingCurve.Type.InOutCubic

_DARK = {
    "bg": "#0D1117", "bg_deep": "#010409", "card": "#161B22",
    "hover": "#1B2330", "active": "#1C2D40", "border": "#21262D",
    "text": "#E6EDF3", "muted": "#8B949E", "accent": "#58A6FF",
    "danger": "#DA3633", "danger_hi": "#F85149",
}
_LIGHT = {
    "bg": "#FFFFFF", "bg_deep": "#F6F8FA", "card": "#F6F8FA",
    "hover": "#EAEEF2", "active": "#DDF4FF", "border": "#D0D7DE",
    "text": "#1F2328", "muted": "#59636E", "accent": "#0969DA",
    "danger": "#CF222E", "danger_hi": "#A40E26",
}


def enabled() -> bool:
    try:
        return bool(prefs.get("ui_animations"))
    except Exception:
        return True


def dur(ms: int) -> int:
    """Duration to use: 0 when animations are off."""
    return int(ms) if enabled() else 0


def is_dark() -> bool:
    app = QApplication.instance()
    return not (app is not None and app.property("hmi_theme") == "light")


def theme_colors() -> dict[str, QColor]:
    src = _DARK if is_dark() else _LIGHT
    return {k: QColor(v) for k, v in src.items()}


def mix(a: QColor, b: QColor, t: float) -> QColor:
    t = max(0.0, min(1.0, t))
    return QColor(
        int(a.red() + (b.red() - a.red()) * t),
        int(a.green() + (b.green() - a.green()) * t),
        int(a.blue() + (b.blue() - a.blue()) * t),
        int(a.alpha() + (b.alpha() - a.alpha()) * t),
    )


def fade_in(widget: QWidget, ms: int = DUR_MED):
    """Fade ``widget`` from transparent to opaque, then drop the effect (keeps painting fast)."""
    if widget is None or not enabled() or not widget.isVisible():
        return None
    eff = QGraphicsOpacityEffect(widget)
    eff.setOpacity(0.0)
    widget.setGraphicsEffect(eff)
    anim = QPropertyAnimation(eff, b"opacity", widget)
    anim.setDuration(ms)
    anim.setStartValue(0.0)
    anim.setEndValue(1.0)
    anim.setEasingCurve(EASE_OUT)

    def _done():
        if widget.graphicsEffect() is eff:
            widget.setGraphicsEffect(None)

    anim.finished.connect(_done)
    anim.start(QPropertyAnimation.DeletionPolicy.DeleteWhenStopped)
    return anim


class AnimatedStack(QStackedWidget):
    """QStackedWidget whose pages fade in when the index changes."""

    def setCurrentIndex(self, index: int) -> None:  # noqa: N802 - Qt naming
        if index == self.currentIndex():
            return
        super().setCurrentIndex(index)
        fade_in(self.currentWidget(), DUR_MED)
