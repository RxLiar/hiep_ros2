"""dashboard.py - live summary cards + vehicle diagram for the Home page."""
import math
import time
from datetime import date

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QWidget, QGridLayout, QVBoxLayout, QLabel, QFrame

from agv_hmi.ui.i18n import tr
from agv_hmi.ui.vehicle_diagram import VehicleDiagram
import agv_hmi.ui.mission_logger as ML


class _Card(QFrame):
    def __init__(self, icon: str):
        super().__init__()
        self.setObjectName("DashCard")
        self.setStyleSheet("#DashCard{border:1px solid #30363D;border-radius:10px;}")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(0)
        self._title = QLabel(icon)
        self._title.setStyleSheet("font-size:11px;color:#8B949E;")
        self._value = QLabel("—")
        self._value.setStyleSheet("font-size:22px;font-weight:700;")
        lay.addWidget(self._title); lay.addWidget(self._value)

    def set(self, title: str, value: str, color: str | None = None):
        self._title.setText(title)
        self._value.setText(value)
        self._value.setStyleSheet(f"font-size:22px;font-weight:700;{'color:' + color + ';' if color else ''}")


class DashboardWidget(QWidget):
    def __init__(self):
        super().__init__()
        self._speed = 0.0
        self._dist_m = 0.0
        self._move_s = 0.0
        self._last_t = time.monotonic()
        self._batt = None
        self._alarms = 0
        lay = QGridLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setHorizontalSpacing(10); lay.setVerticalSpacing(10)
        self._c_speed, self._c_dist, self._c_time = _Card("⚡"), _Card("📏"), _Card("⏱")
        self._c_batt, self._c_miss, self._c_cargo, self._c_alarm = _Card("🔋"), _Card("🧭"), _Card("📦"), _Card("🔔")
        self._c_safety = _Card("🛡")
        self._safety = None
        cards = (self._c_speed, self._c_dist, self._c_time, self._c_alarm,
                 self._c_batt, self._c_miss, self._c_cargo, self._c_safety)
        for i, c in enumerate(cards):
            lay.addWidget(c, i // 4, i % 4)
        self.diagram = VehicleDiagram()
        lay.addWidget(self.diagram, 0, 4, 2, 2)
        lay.setColumnStretch(4, 2)
        self._t = QTimer(self); self._t.timeout.connect(self._tick); self._t.start(1000)
        self._stats_t = QTimer(self); self._stats_t.timeout.connect(self.refresh_stats); self._stats_t.start(15000)
        self._missions = 0; self._cargo = 0
        self.refresh_stats()
        self.retranslate()

    # feeders
    def set_speed(self, linear: float, angular: float):
        self._speed = linear

    def set_battery(self, pct): self._batt = pct
    def set_safety(self, state): self._safety = state
    def set_alarm_count(self, n: int): self._alarms = n

    def _tick(self):
        now = time.monotonic(); dt = now - self._last_t; self._last_t = now
        if abs(self._speed) > 0.02:
            self._dist_m += abs(self._speed) * dt; self._move_s += dt
        self.retranslate()

    def refresh_stats(self):
        today = date.today().isoformat()
        try:
            recs = [r for r in ML.load_all() if str(r.get("started_at", "")).startswith(today)]
        except Exception:
            recs = []
        self._missions = len(recs)
        self._cargo = sum(int(r.get("cargo_count", 0) or 0) for r in recs)

    def retranslate(self):
        self._c_speed.set(tr("dash_speed"), f"{abs(self._speed):.2f} m/s")
        self._c_dist.set(tr("dash_dist"), f"{self._dist_m / 1000:.2f} km" if self._dist_m >= 1000 else f"{self._dist_m:.0f} m")
        m, s = divmod(int(self._move_s), 60); h, m = divmod(m, 60)
        self._c_time.set(tr("dash_move"), f"{h:d}:{m:02d}:{s:02d}")
        self._c_batt.set(tr("dash_batt"), "—" if self._batt is None else f"{self._batt}%",
                         None if self._batt is None or self._batt >= 20 else "#F85149")
        self._c_miss.set(tr("dash_missions"), str(self._missions))
        self._c_cargo.set(tr("dash_cargo"), str(self._cargo))
        self._c_alarm.set(tr("dash_alarms"), str(self._alarms), "#F85149" if self._alarms else None)
        colors = {"ok": "#3FB950", "slow": "#E3B341", "stop": "#F85149", "no_scan": "#F85149"}
        st = self._safety
        self._c_safety.set(tr("dash_safety"), tr(f"sf_state_{st}") if st else "—", colors.get(st))
