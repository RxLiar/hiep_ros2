"""reports_page.py - mission statistics (from mission_logger) + audit log."""
import csv
import os
from collections import defaultdict
from datetime import date, datetime, timedelta

from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QPainter, QColor, QFont
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QTableWidget,
                             QTableWidgetItem, QPushButton, QHeaderView, QAbstractItemView,
                             QLabel, QFileDialog)

import agv_hmi.ui.mission_logger as ML
from agv_hmi.core import audit_log
from agv_hmi.ui.i18n import tr


def day_stats(records: list[dict]) -> dict:
    """{iso_day: {total, ok, fail, cancel, dur_sum, cargo}}"""
    out = defaultdict(lambda: dict(total=0, ok=0, fail=0, cancel=0, dur=0.0, cargo=0))
    for r in records:
        d = str(r.get("started_at", ""))[:10]
        if len(d) != 10:
            continue
        s = out[d]
        s["total"] += 1
        st = r.get("status")
        s["ok" if st == "success" else "cancel" if st == "cancelled" else "fail"] += 1
        s["dur"] += float(r.get("duration_sec", 0) or 0)
        s["cargo"] += int(r.get("cargo_count", 0) or 0)
    return dict(out)


class _BarChart(QWidget):
    def __init__(self):
        super().__init__()
        self._data: list[tuple[str, int, int]] = []     # (label, ok, other)
        self.setMinimumHeight(160)

    def set_data(self, data):
        self._data = data; self.update()

    def paintEvent(self, _):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        n = len(self._data)
        if not n:
            return
        mx = max(1, max(a + b for _, a, b in self._data))
        w, h = self.width(), self.height() - 22
        bw = w / n
        f = QFont(); f.setPixelSize(10); p.setFont(f)
        for i, (lab, ok, other) in enumerate(self._data):
            x = i * bw + bw * 0.2
            for val, color, base in ((ok, "#3FB950", 0), (other, "#F85149", ok)):
                if val:
                    hh = val / mx * (h - 14)
                    p.fillRect(QRectF(x, h - (base + val) / mx * (h - 14), bw * 0.6, hh), QColor(color))
            p.setPen(QColor("#8B949E"))
            p.drawText(QRectF(i * bw, h + 4, bw, 14), Qt.AlignmentFlag.AlignCenter, lab)
            if ok + other:
                p.drawText(QRectF(i * bw, h - (ok + other) / mx * (h - 14) - 14, bw, 12),
                           Qt.AlignmentFlag.AlignCenter, str(ok + other))
        p.end()


class ReportsPage(QWidget):
    def __init__(self, role: str = "operator"):
        super().__init__()
        self._role = role
        lay = QVBoxLayout(self); lay.setContentsMargins(12, 10, 12, 10)
        self._tabs = QTabWidget(); lay.addWidget(self._tabs)

        # stats tab
        w = QWidget(); v = QVBoxLayout(w)
        bar = QHBoxLayout(); bar.addStretch()
        self._refresh = QPushButton(); self._refresh.clicked.connect(self.refresh)
        self._export = QPushButton(); self._export.clicked.connect(self._export_csv)
        bar.addWidget(self._refresh); bar.addWidget(self._export); v.addLayout(bar)
        self._chart_lbl = QLabel(); self._chart_lbl.setStyleSheet("font-weight:700;")
        self._chart = _BarChart()
        v.addWidget(self._chart_lbl); v.addWidget(self._chart)
        self._table = self._make_table(7); v.addWidget(self._table, 1)
        self._tabs.addTab(w, "")

        # audit tab (engineer only)
        self._audit_w = None
        if role == "engineer":
            a = QWidget(); av = QVBoxLayout(a)
            ab = QHBoxLayout(); ab.addStretch()
            self._a_refresh = QPushButton(); self._a_refresh.clicked.connect(self._load_audit)
            self._a_export = QPushButton(); self._a_export.clicked.connect(self._export_audit)
            ab.addWidget(self._a_refresh); ab.addWidget(self._a_export); av.addLayout(ab)
            self._a_table = self._make_table(4); av.addWidget(self._a_table)
            self._tabs.addTab(a, ""); self._audit_w = a
        self.retranslate()

    @staticmethod
    def _make_table(cols):
        t = QTableWidget(0, cols)
        t.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        t.verticalHeader().setVisible(False)
        t.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        return t

    def showEvent(self, ev):
        super().showEvent(ev); self.refresh()

    def refresh(self):
        recs = ML.load_all()
        stats = day_stats(recs)
        days = sorted(stats, reverse=True)[:30]
        self._table.setRowCount(len(days))
        for r, d in enumerate(days):
            s = stats[d]
            avg = s["dur"] / s["total"] if s["total"] else 0
            for c, v in enumerate((d, s["total"], s["ok"], s["fail"], s["cancel"], f"{avg:.0f}", s["cargo"])):
                self._table.setItem(r, c, QTableWidgetItem(str(v)))
        last14 = [(date.today() - timedelta(days=i)).isoformat() for i in range(13, -1, -1)]
        self._chart.set_data([(d[5:], stats.get(d, {}).get("ok", 0),
                               stats.get(d, {}).get("fail", 0) + stats.get(d, {}).get("cancel", 0)) for d in last14])
        if self._audit_w is not None:
            self._load_audit()

    def _load_audit(self):
        rows = audit_log.read_recent(500)
        self._a_table.setRowCount(len(rows))
        for r, rec in enumerate(rows):
            for c, k in enumerate(("at", "role", "action", "detail")):
                self._a_table.setItem(r, c, QTableWidgetItem(str(rec.get(k, ""))))

    def _export_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, tr("rp_export"),
                                              os.path.expanduser(f"~/missions_{datetime.now():%Y%m%d}.csv"), "CSV (*.csv)")
        if not path:
            return
        recs = ML.load_all()
        keys = ["id", "route_name", "started_at", "finished_at", "duration_sec", "status",
                "waypoints_done", "waypoints_total", "cargo_count", "notes"]
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore"); w.writeheader(); w.writerows(recs)

    def _export_audit(self):
        path, _ = QFileDialog.getSaveFileName(self, tr("rp_export"),
                                              os.path.expanduser(f"~/audit_{datetime.now():%Y%m%d}.csv"), "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=["at", "user", "role", "action", "detail"], extrasaction="ignore")
            w.writeheader(); w.writerows(audit_log.read_recent(5000))

    def retranslate(self):
        self._tabs.setTabText(0, tr("rp_tab_stats"))
        if self._audit_w is not None:
            self._tabs.setTabText(1, tr("rp_tab_audit"))
            self._a_refresh.setText(tr("rp_refresh")); self._a_export.setText(tr("rp_export"))
            self._a_table.setHorizontalHeaderLabels([tr("au_time"), tr("au_user"), tr("au_action"), tr("au_detail")])
        self._refresh.setText(tr("rp_refresh")); self._export.setText(tr("rp_export"))
        self._chart_lbl.setText(tr("rp_chart"))
        self._table.setHorizontalHeaderLabels([tr(k) for k in
            ("rp_day", "rp_total", "rp_ok", "rp_fail", "rp_cancel", "rp_avg", "rp_cargo")])
