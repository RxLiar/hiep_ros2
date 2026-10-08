"""preflight.py - checklist dialog shown before a mission starts."""
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton)

from agv_hmi.core.health_model import HealthModel
from agv_hmi.ui.i18n import tr


class PreflightDialog(QDialog):
    """exec() returns Accepted when the mission may start."""

    def __init__(self, health: HealthModel, allow_override: bool, parent=None):
        super().__init__(parent)
        self._health = health
        self._allow_override = allow_override
        self.setWindowTitle(tr("pf_title"))
        self.setModal(True)
        self.setMinimumWidth(460)
        lay = QVBoxLayout(self)
        lay.setSpacing(8)
        head = QLabel("🛫 " + tr("pf_title"))
        head.setStyleSheet("font-size:17px;font-weight:700;")
        lay.addWidget(head)
        self._rows = QVBoxLayout()
        lay.addLayout(self._rows)
        self._summary = QLabel()
        self._summary.setWordWrap(True)
        lay.addWidget(self._summary)

        btns = QHBoxLayout()
        self._recheck = QPushButton(tr("pf_recheck"))
        self._recheck.clicked.connect(self.refresh)
        self._override = QPushButton(tr("pf_run_anyway"))
        self._override.clicked.connect(self.accept)
        self._override.setVisible(allow_override)
        self._cancel = QPushButton(tr("pf_cancel"))
        self._cancel.clicked.connect(self.reject)
        self._start = QPushButton(tr("pf_start"))
        self._start.setObjectName("BtnSuccess")
        self._start.clicked.connect(self.accept)
        for b in (self._recheck, self._override):
            btns.addWidget(b)
        btns.addStretch()
        btns.addWidget(self._cancel)
        btns.addWidget(self._start)
        lay.addLayout(btns)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.refresh)
        self._timer.start(1000)
        self.refresh()

    def refresh(self):
        while self._rows.count():
            w = self._rows.takeAt(0).widget()
            if w:
                w.deleteLater()
        blocked = False
        for c in self._health.checks():
            icon = "✅" if c.ok else ("❌" if c.blocking else "⚠️")
            blocked |= (not c.ok and c.blocking)
            lbl = QLabel(f"{icon}  {tr(c.label_key)}" + (f"   — {c.detail}" if c.detail and not c.ok else ""))
            lbl.setStyleSheet("font-size:14px;" + ("" if c.ok else "color:#FF7B72;" if c.blocking else "color:#E3B341;"))
            self._rows.addWidget(lbl)
        self._start.setEnabled(not blocked)
        self._override.setEnabled(blocked)
        self._summary.setText(tr("pf_blocked") if blocked else tr("pf_ready"))
        self._summary.setStyleSheet("color:#FF7B72;" if blocked else "color:#3FB950;")
