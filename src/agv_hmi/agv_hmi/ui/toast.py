"""toast.py - non-blocking pop-up messages in the corner of the main window."""
from PyQt6.QtCore import Qt, QTimer, QPropertyAnimation
from PyQt6.QtWidgets import QGraphicsOpacityEffect, QLabel, QVBoxLayout, QWidget

from agv_hmi.ui import anim

_COLORS = {"info": "#1F6FEB", "ok": "#238636", "warn": "#9E6A03", "error": "#DA3633", "critical": "#8B0000"}
_EDGE = {"info": "#79B8FF", "ok": "#56D364", "warn": "#E3B341", "error": "#FF7B72", "critical": "#FF4D4D"}
_ICON = {"info": "ℹ", "ok": "✓", "warn": "⚠", "error": "✕", "critical": "⛔"}


class _Toast(QLabel):
    """Pop-up that fades in, waits ``ms`` and fades out."""

    def __init__(self, text: str, level: str, ms: int, parent):
        super().__init__(f"{_ICON.get(level, 'ℹ')}   {text}", parent)
        self.setWordWrap(True)
        self.setMaximumWidth(380)
        self.setStyleSheet(
            f"background:{_COLORS.get(level, '#1F6FEB')};color:white;"
            f"border-left:4px solid {_EDGE.get(level, '#79B8FF')};"
            "padding:10px 14px;border-radius:8px;font-size:13px;")
        self._eff = QGraphicsOpacityEffect(self)
        self._eff.setOpacity(0.0 if anim.enabled() else 1.0)
        self.setGraphicsEffect(self._eff)
        self._fade(1.0, anim.DUR_MED)
        self._life = QTimer(self)
        self._life.setSingleShot(True)
        self._life.timeout.connect(self._leave)
        self._life.start(max(0, int(ms)))

    def _fade(self, end: float, ms: int, on_done=None):
        d = anim.dur(ms)
        if d == 0:
            self._eff.setOpacity(end)
            if on_done:
                on_done()
            return
        self._a = QPropertyAnimation(self._eff, b"opacity", self)
        self._a.setDuration(d)
        self._a.setStartValue(self._eff.opacity())
        self._a.setEndValue(end)
        self._a.setEasingCurve(anim.EASE_OUT)
        if on_done:
            self._a.finished.connect(on_done)
        self._a.start()

    def _leave(self):
        self._fade(0.0, anim.DUR_SLOW, self.deleteLater)


class ToastManager(QWidget):
    """Child overlay of the main window; call show_toast() from anywhere."""

    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self._lay = QVBoxLayout(self)
        self._lay.setContentsMargins(0, 0, 0, 0)
        self._lay.setSpacing(8)
        self._lay.addStretch()
        self.raise_()

    def show_toast(self, text: str, level: str = "info", ms: int = 3500):
        while self._lay.count() > 5:          # at most 4 toasts at once
            old = self._lay.takeAt(1)
            if old.widget():
                old.widget().deleteLater()
        t = _Toast(text, level, ms, self)
        self._lay.addWidget(t)
        self.reposition()
        self.raise_()

    def reposition(self):
        p = self.parentWidget()
        if p is None:
            return
        w, h = 400, min(p.height() - 140, 360)
        self.setGeometry(p.width() - w - 18, p.height() - h - 18, w, h)
