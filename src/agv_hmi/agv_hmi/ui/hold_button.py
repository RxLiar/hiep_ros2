"""hold_button.py - press and HOLD to confirm a risky action."""
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QRectF
from PyQt6.QtGui import QPainter, QColor
from PyQt6.QtWidgets import QPushButton


class HoldButton(QPushButton):
    confirmed = pyqtSignal()

    def __init__(self, text: str = "", hold_ms: int = 1000, parent=None):
        super().__init__(text, parent)
        self._hold_ms = max(200, int(hold_ms))
        self._elapsed = 0
        self._timer = QTimer(self)
        self._timer.setInterval(30)
        self._timer.timeout.connect(self._tick)
        self._fired = False

    def mousePressEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton and self.isEnabled():
            self._elapsed, self._fired = 0, False
            self._timer.start()
        super().mousePressEvent(ev)

    def mouseReleaseEvent(self, ev):
        fired = self._fired
        self._stop()
        if fired:               # a completed hold must not also count as a click
            self._fired = False
            self.setDown(False)
            ev.accept()
            return
        super().mouseReleaseEvent(ev)

    def leaveEvent(self, ev):
        self._stop()
        super().leaveEvent(ev)

    def _stop(self):
        self._timer.stop()
        self._elapsed = 0
        self.update()

    def _tick(self):
        self._elapsed += self._timer.interval()
        if self._elapsed >= self._hold_ms and not self._fired:
            self._fired = True
            self._timer.stop()
            self.confirmed.emit()
        self.update()

    def paintEvent(self, ev):
        super().paintEvent(ev)
        if self._elapsed <= 0:
            return
        p = QPainter(self)
        frac = min(1.0, self._elapsed / self._hold_ms)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(255, 255, 255, 70))
        p.drawRoundedRect(QRectF(0, 0, self.width() * frac, self.height()), 6, 6)
        p.end()
