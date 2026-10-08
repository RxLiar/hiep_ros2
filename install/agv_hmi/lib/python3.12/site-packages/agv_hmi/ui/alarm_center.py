"""alarm_center.py - alarm list page + badge helper."""
from datetime import datetime

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QBrush
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTableWidget,
                             QTableWidgetItem, QPushButton, QCheckBox, QHeaderView,
                             QAbstractItemView, QLabel)

from agv_hmi.core.alarm_manager import AlarmManager
from agv_hmi.ui.i18n import tr

_BG = {"info": "#16324F", "warn": "#4A3B0B", "error": "#4F1717", "critical": "#6B0F0F"}
_FG = {"info": "#79B8FF", "warn": "#E3B341", "error": "#FF7B72", "critical": "#FFFFFF"}


class AlarmCenterPage(QWidget):
    def __init__(self, manager: AlarmManager):
        super().__init__()
        self._mgr = manager
        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 14, 16, 14)
        lay.setSpacing(10)

        bar = QHBoxLayout()
        self._title = QLabel()
        self._title.setStyleSheet("font-size:18px;font-weight:700;")
        bar.addWidget(self._title)
        bar.addStretch()
        self._only_active = QCheckBox()
        self._only_active.toggled.connect(self.refresh)
        bar.addWidget(self._only_active)
        self._ack_btn = QPushButton()
        self._ack_btn.clicked.connect(self._mgr.ack_all)
        self._clr_btn = QPushButton()
        self._clr_btn.clicked.connect(self._mgr.clear_history)
        bar.addWidget(self._ack_btn)
        bar.addWidget(self._clr_btn)
        lay.addLayout(bar)

        self._table = QTableWidget(0, 5)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.verticalHeader().setVisible(False)
        hh = self._table.horizontalHeader()
        hh.setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        for c in (0, 1, 2, 4):
            hh.setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        self._table.cellDoubleClicked.connect(self._on_dbl)
        lay.addWidget(self._table, 1)

        self._empty = QLabel()
        self._empty.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self._empty)

        self._mgr.changed.connect(self.refresh)
        self.retranslate()

    def retranslate(self):
        self._title.setText("🔔 " + tr("page_alarms"))
        self._only_active.setText(tr("al_only_active"))
        self._ack_btn.setText(tr("al_ack_all"))
        self._clr_btn.setText(tr("al_clear"))
        self._empty.setText(tr("al_none"))
        self._table.setHorizontalHeaderLabels(
            [tr("al_time"), tr("al_level"), tr("al_source"), tr("al_message"), tr("al_state")])
        self.refresh()

    def refresh(self, *_):
        items = self._mgr.items()
        if self._only_active.isChecked():
            items = [i for i in items if i["t_cleared"] is None]
        self._empty.setVisible(not items)
        self._table.setRowCount(len(items))
        for r, it in enumerate(items):
            active = it["t_cleared"] is None
            state = tr("al_active") if active else tr("al_cleared")
            if active and not it["acked"]:
                state += " •"
            cells = [datetime.fromtimestamp(it["t_raised"]).strftime("%H:%M:%S"),
                     tr("lvl_" + it["level"]), it["source"], it["message"], state]
            for c, text in enumerate(cells):
                cell = QTableWidgetItem(text)
                cell.setData(Qt.ItemDataRole.UserRole, it["id"])
                if active:
                    cell.setBackground(QBrush(QColor(_BG[it["level"]])))
                    cell.setForeground(QBrush(QColor(_FG[it["level"]])))
                    if not it["acked"]:
                        f = cell.font(); f.setBold(True); cell.setFont(f)
                self._table.setItem(r, c, cell)

    def _on_dbl(self, row, _col):
        item = self._table.item(row, 0)
        if item is not None:
            self._mgr.ack(int(item.data(Qt.ItemDataRole.UserRole)))
