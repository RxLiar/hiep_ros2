"""stations_page.py - named stations: save current pose, set pose here, go to station."""
import math

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
                             QPushButton, QHeaderView, QAbstractItemView, QLabel, QInputDialog,
                             QComboBox, QMessageBox)

from agv_hmi.core import stations as ST
from agv_hmi.ui.i18n import tr


class StationsPage(QWidget):
    set_pose_requested = pyqtSignal(float, float, float)   # x, y, yaw
    goto_requested = pyqtSignal(dict)                      # station dict

    def __init__(self):
        super().__init__()
        self._pose = None
        lay = QVBoxLayout(self); lay.setContentsMargins(16, 14, 16, 14)
        self._title = QLabel(); self._title.setStyleSheet("font-size:18px;font-weight:700;")
        lay.addWidget(self._title)
        self._table = QTableWidget(0, 5)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._table.verticalHeader().setVisible(False)
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        lay.addWidget(self._table, 1)
        self._hint = QLabel(); self._hint.setStyleSheet("color:#8B949E;"); lay.addWidget(self._hint)
        row = QHBoxLayout()
        self._b_add, self._b_del = QPushButton(), QPushButton()
        self._b_pose, self._b_go, self._b_charge = QPushButton(), QPushButton(), QPushButton()
        self._b_go.setObjectName("BtnSuccess")
        self._b_add.clicked.connect(self._add_here); self._b_del.clicked.connect(self._delete)
        self._b_pose.clicked.connect(self._set_pose); self._b_go.clicked.connect(self._goto)
        self._b_charge.clicked.connect(self._goto_charge)
        for b in (self._b_add, self._b_del, self._b_pose, self._b_go, self._b_charge):
            row.addWidget(b)
        lay.addLayout(row)
        self._msg = QLabel(); self._msg.setWordWrap(True); lay.addWidget(self._msg)
        self.retranslate()

    def set_current_pose(self, x: float, y: float, yaw: float):
        self._pose = (x, y, yaw)

    def showEvent(self, ev):
        super().showEvent(ev); self.refresh()

    def refresh(self):
        items = ST.load()
        self._table.setRowCount(len(items))
        for r, s in enumerate(items):
            vals = (s["name"], tr(f"st_kind_{s.get('kind', 'normal')}"), f"{s['x']:.2f}", f"{s['y']:.2f}",
                    f"{math.degrees(s.get('yaw', 0.0)):.0f}°")
            for c, v in enumerate(vals):
                it = QTableWidgetItem(v)
                if c == 0:
                    it.setData(Qt.ItemDataRole.UserRole, s["id"])
                self._table.setItem(r, c, it)

    def _selected(self):
        r = self._table.currentRow()
        if r < 0:
            return None
        sid = self._table.item(r, 0).data(Qt.ItemDataRole.UserRole)
        return next((s for s in ST.load() if s["id"] == sid), None)

    def _add_here(self):
        if self._pose is None:
            self._msg.setText("⚠ " + tr("st_no_pose")); return
        name, ok = QInputDialog.getText(self, tr("page_stations"), tr("st_prompt_name"))
        if not ok:
            return
        kinds = [tr("st_kind_normal"), tr("st_kind_charge"), tr("st_kind_home")]
        label, ok = QInputDialog.getItem(self, tr("page_stations"), tr("st_kind"), kinds, 0, False)
        if not ok:
            return
        ST.add(name, *self._pose, kind=ST.KINDS[kinds.index(label)])
        self._msg.setText(""); self.refresh()

    def _delete(self):
        s = self._selected()
        if s:
            ST.remove(s["id"]); self.refresh()

    def _set_pose(self):
        s = self._selected()
        if s:
            self.set_pose_requested.emit(s["x"], s["y"], s.get("yaw", 0.0))

    def _goto(self):
        s = self._selected()
        if s:
            self.goto_requested.emit(s)

    def _goto_charge(self):
        s = ST.first_of_kind("charge")
        if s is None:
            self._msg.setText("⚠ " + tr("st_no_charge")); return
        self.goto_requested.emit(s)

    def retranslate(self):
        self._title.setText("📍 " + tr("page_stations"))
        self._table.setHorizontalHeaderLabels([tr("st_name"), tr("st_kind"), "X (m)", "Y (m)", "Yaw"])
        self._hint.setText(tr("st_pose_hint"))
        for b, k in ((self._b_add, "st_add_here"), (self._b_del, "st_delete"), (self._b_pose, "st_set_pose"),
                     (self._b_go, "st_goto"), (self._b_charge, "st_goto_charge")):
            b.setText(tr(k))
        self.refresh()
