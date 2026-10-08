"""schedule_page.py - timed jobs + run queue for routes."""
from PyQt6.QtCore import Qt, pyqtSignal, QTime
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
                             QPushButton, QHeaderView, QAbstractItemView, QLabel, QComboBox,
                             QTimeEdit, QCheckBox, QListWidget, QDialog, QDialogButtonBox, QGroupBox)

import agv_hmi.ui.route_manager as RM
from agv_hmi.core import prefs
from agv_hmi.core.scheduler import Scheduler
from agv_hmi.ui.i18n import tr


class _JobDialog(QDialog):
    def __init__(self, routes, parent=None):
        super().__init__(parent)
        self.setWindowTitle(tr("sc_add"))
        v = QVBoxLayout(self)
        self.route = QComboBox()
        for r in routes:
            self.route.addItem(r.get("name", "?"), r.get("id"))
        self.time = QTimeEdit(QTime(8, 0)); self.time.setDisplayFormat("HH:mm")
        v.addWidget(QLabel(tr("sc_route"))); v.addWidget(self.route)
        v.addWidget(QLabel(tr("sc_time"))); v.addWidget(self.time)
        row = QHBoxLayout(); self.days = []
        for i in range(7):
            cb = QCheckBox(tr(f"day_{i}")); cb.setChecked(i < 5); self.days.append(cb); row.addWidget(cb)
        v.addLayout(row)
        bb = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        bb.accepted.connect(self.accept); bb.rejected.connect(self.reject); v.addWidget(bb)


class SchedulePage(QWidget):
    run_queue_requested = pyqtSignal()
    auto_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.scheduler = Scheduler()
        self.queue: list[dict] = []          # route dicts waiting to run
        lay = QVBoxLayout(self); lay.setContentsMargins(16, 14, 16, 14)

        self._auto = QCheckBox(); self._auto.setChecked(bool(prefs.get("schedule_enabled") or False))
        self._auto.toggled.connect(self._on_auto)
        self._warn = QLabel(); self._warn.setWordWrap(True); self._warn.setStyleSheet("color:#E3B341;")
        lay.addWidget(self._auto); lay.addWidget(self._warn)

        self._jobs_box = QGroupBox(); jv = QVBoxLayout(self._jobs_box)
        self._jobs = QTableWidget(0, 4)
        self._jobs.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._jobs.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._jobs.verticalHeader().setVisible(False)
        self._jobs.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        jv.addWidget(self._jobs)
        jr = QHBoxLayout()
        self._j_add, self._j_del, self._j_tog = QPushButton(), QPushButton(), QPushButton()
        self._j_add.clicked.connect(self._add_job); self._j_del.clicked.connect(self._del_job)
        self._j_tog.clicked.connect(self._toggle_job)
        for b in (self._j_add, self._j_tog, self._j_del):
            jr.addWidget(b)
        jv.addLayout(jr); lay.addWidget(self._jobs_box, 2)

        self._q_box = QGroupBox(); qv = QVBoxLayout(self._q_box)
        self._q_route = QComboBox(); qv.addWidget(self._q_route)
        self._q_list = QListWidget(); qv.addWidget(self._q_list)
        qr = QHBoxLayout()
        self._q_add, self._q_run, self._q_clear = QPushButton(), QPushButton(), QPushButton()
        self._q_run.setObjectName("BtnSuccess")
        self._q_add.clicked.connect(self._queue_add)
        self._q_run.clicked.connect(self.run_queue_requested)
        self._q_clear.clicked.connect(self.clear_queue)
        for b in (self._q_add, self._q_run, self._q_clear):
            qr.addWidget(b)
        qv.addLayout(qr); lay.addWidget(self._q_box, 2)
        self.retranslate()

    # -- queue API used by MainWindow ----------------------------------------
    def enqueue(self, route: dict):
        self.queue.append(route); self._refresh_queue()

    def pop_next(self):
        if not self.queue:
            return None
        r = self.queue.pop(0); self._refresh_queue(); return r

    def clear_queue(self):
        self.queue.clear(); self._refresh_queue()

    def auto_enabled(self) -> bool:
        return self._auto.isChecked()

    def _on_auto(self, on):
        prefs.set("schedule_enabled", bool(on)); self.auto_changed.emit(bool(on))

    def _refresh_queue(self):
        self._q_list.clear()
        for i, r in enumerate(self.queue, 1):
            self._q_list.addItem(f"{i}. {r.get('name', '?')}")

    def _refresh_routes(self):
        self._q_route.clear()
        for r in RM.load_all():
            self._q_route.addItem(r.get("name", "?"), r.get("id"))

    def showEvent(self, ev):
        super().showEvent(ev); self._refresh_routes(); self._refresh_jobs()

    def _refresh_jobs(self):
        jobs = self.scheduler.jobs
        self._jobs.setRowCount(len(jobs))
        for r, j in enumerate(jobs):
            days = " ".join(tr(f"day_{d}") for d in j.get("days", []))
            vals = ("✔" if j.get("enabled") else "—", j.get("time", ""), days, j.get("route_name", ""))
            for c, v in enumerate(vals):
                it = QTableWidgetItem(v)
                if c == 0:
                    it.setData(Qt.ItemDataRole.UserRole, j["id"])
                self._jobs.setItem(r, c, it)

    def _sel_job(self):
        r = self._jobs.currentRow()
        return self._jobs.item(r, 0).data(Qt.ItemDataRole.UserRole) if r >= 0 else None

    def _add_job(self):
        routes = RM.load_all()
        if not routes:
            return
        d = _JobDialog(routes, self)
        if d.exec() == QDialog.DialogCode.Accepted:
            self.scheduler.add(d.route.currentData(), d.route.currentText(), d.time.time().toString("HH:mm"),
                               [i for i, cb in enumerate(d.days) if cb.isChecked()])
            self._refresh_jobs()

    def _del_job(self):
        jid = self._sel_job()
        if jid:
            self.scheduler.remove(jid); self._refresh_jobs()

    def _toggle_job(self):
        jid = self._sel_job()
        if jid:
            cur = next((j for j in self.scheduler.jobs if j["id"] == jid), None)
            if cur:
                self.scheduler.set_enabled(jid, not cur.get("enabled")); self._refresh_jobs()

    def _queue_add(self):
        rid = self._q_route.currentData()
        route = RM.get_route(rid) if rid else None
        if route:
            self.enqueue(route)

    def retranslate(self):
        self._auto.setText(tr("sc_auto")); self._warn.setText(tr("sc_warn"))
        self._jobs_box.setTitle(tr("sc_jobs")); self._q_box.setTitle(tr("sc_queue"))
        self._jobs.setHorizontalHeaderLabels([tr("sc_on"), tr("sc_time"), tr("sc_days"), tr("sc_route")])
        for b, k in ((self._j_add, "sc_add"), (self._j_del, "sc_remove"), (self._j_tog, "sc_on"),
                     (self._q_add, "sc_q_add"), (self._q_run, "sc_q_run"), (self._q_clear, "sc_q_clear")):
            b.setText(tr(k))
        self._refresh_routes(); self._refresh_jobs()
