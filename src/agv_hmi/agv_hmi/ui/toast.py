"""toast.py - non-blocking pop-up messages in the corner of the main window."""
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

_COLORS = {"info": "#1F6FEB", "ok": "#238636", "warn": "#9E6A03", "error": "#DA3633", "critical": "#8B0000"}


class _Toast(QLabel):
    def __init__(self, text: str, level: str, ms: int, parent):
        super().__init__(text, parent)
        self.setWordWrap(True)
        self.setMaximumWidth(380)
        self.setStyleSheet(
            f"background:{_COLORS.get(level, '#1F6FEB')};color:white;"
            "padding:10px 14px;border-radius:8px;font-size:13px;")
        QTimer.singleShot(ms, self.deleteLater)


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
