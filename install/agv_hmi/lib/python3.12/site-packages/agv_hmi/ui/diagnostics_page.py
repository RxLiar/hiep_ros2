"""diagnostics_page.py - topic health, live plots, ROS log, rosbag, hardware launch."""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from collections import deque
from datetime import datetime

from PyQt6.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt6.QtGui import QPainter, QPen, QColor, QFont, QPolygonF
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QTableWidget, QTableWidgetItem,
    QHeaderView, QAbstractItemView, QLabel, QPushButton, QComboBox, QLineEdit,
    QPlainTextEdit, QCheckBox, QFileDialog, QGridLayout, QGroupBox, QFormLayout,
)

from agv_hmi.core.health_model import HealthModel
from agv_hmi.ui.i18n import tr
from agv_hmi.ui.process_manager import ManagedLaunch

_LEVEL_NAMES = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
BAG_DIR = os.path.expanduser("~/agv_bags")
BAG_TOPICS = ["/scan", "/odom", "/wheel/odom", "/amcl_pose", "/cmd_vel", "/tf", "/tf_static",
              "/plan", "/keya_driver_status", "/emergency_stop", "/plc_connection_status",
              "/rosout", "/robot_status"]


# ── tiny real-time plot (no pyqtgraph dependency) ──────────────────────────
class LivePlot(QWidget):
    def __init__(self, window_s: float = 60.0):
        super().__init__()
        self._win = window_s
        self._title = ""
        self._series: list[dict] = []       # {"name", "color", "pts": deque[(t, v)]}
        self.paused = False
        self.setMinimumHeight(130)

    def set_title(self, t: str):
        self._title = t
        self.update()

    def add_series(self, name: str, color: str):
        self._series.append({"name": name, "color": QColor(color), "pts": deque(maxlen=4000)})
        return len(self._series) - 1

    def rename(self, idx: int, name: str):
        self._series[idx]["name"] = name

    def push(self, idx: int, value: float, t: float | None = None):
        if self.paused:
            return
        self._series[idx]["pts"].append((t if t is not None else time.monotonic(), float(value)))

    def clear(self):
        for s in self._series:
            s["pts"].clear()
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = self.rect().adjusted(46, 22, -10, -16)
        p.fillRect(self.rect(), QColor(22, 27, 34))
        f = QFont(); f.setPixelSize(11); p.setFont(f)
        p.setPen(QColor(201, 209, 217))
        p.drawText(8, 14, self._title)
        now = time.monotonic()
        vals = [v for s in self._series for t, v in s["pts"] if now - t <= self._win]
        lo, hi = (min(vals), max(vals)) if vals else (-1.0, 1.0)
        if hi - lo < 1e-6:
            lo, hi = lo - 0.5, hi + 0.5
        pad = (hi - lo) * 0.1
        lo, hi = lo - pad, hi + pad
        p.setPen(QPen(QColor(48, 54, 61), 1))
        for i in range(5):
            y = r.top() + r.height() * i / 4
            p.drawLine(r.left(), int(y), r.right(), int(y))
            p.setPen(QColor(139, 148, 158))
            p.drawText(2, int(y) + 4, 42, 12, Qt.AlignmentFlag.AlignRight, f"{hi - (hi - lo) * i / 4:.2f}")
            p.setPen(QPen(QColor(48, 54, 61), 1))
        lx = r.left()
        for s in self._series:
            pts = [(t, v) for t, v in s["pts"] if now - t <= self._win]
            if len(pts) >= 2:
                poly = QPolygonF([QPointF(r.right() - (now - t) / self._win * r.width(),
                                          r.bottom() - (v - lo) / (hi - lo) * r.height()) for t, v in pts])
                p.setPen(QPen(s["color"], 1.6))
                p.drawPolyline(poly)
            last = f" {s['pts'][-1][1]:.2f}" if s["pts"] else ""
            p.setPen(s["color"])
            p.drawText(lx, r.bottom() + 13, f"■ {s['name']}{last}")
            lx += 140
        p.end()


# ── page ──────────────────────────────────────────────────────────────────
class DiagnosticsPage(QWidget):
    def __init__(self, health: HealthModel):
        super().__init__()
        self._health = health
        self._rates: dict[str, float] = {}
        self._cmd = (0.0, 0.0)
        self._actual = (0.0, 0.0)
        self._logs: deque = deque(maxlen=5000)
        self._bag_proc: subprocess.Popen | None = None
        self._bag_path = ""
        self._bag_stop_timer = QTimer(self)
        self._bag_stop_timer.setSingleShot(True)
        self._bag_stop_timer.timeout.connect(self.stop_bag)
        self._hw = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        self._tabs = QTabWidget()
        lay.addWidget(self._tabs)
        self._build_topics()
        self._build_plots()
        self._build_logs()
        self._build_bag()
        self._build_hw()
        self.retranslate()

        self._tick = QTimer(self)
        self._tick.timeout.connect(self._on_tick)
        self._tick.start(100)
        self._slow = QTimer(self)
        self._slow.timeout.connect(self._refresh_topics)
        self._slow.start(1000)

    # -- feeders (connected by MainWindow) ------------------------------------
    def update_rates(self, rates: dict):
        self._rates = dict(rates)
        self._health.rates = dict(rates)

    def feed_cmd_vel(self, lin: float, ang: float):
        self._cmd = (lin, ang)

    def feed_odom_twist(self, lin: float, ang: float):
        self._actual = (lin, ang)

    def feed_rosout(self, level: int, node: str, text: str, stamp: float):
        rec = (stamp or time.time(), level, node, text)
        self._logs.append(rec)
        if self._log_match(rec):
            self._log_view.appendPlainText(self._fmt_log(rec))

    # -- topics tab ------------------------------------------------------------
    def _build_topics(self):
        w = QWidget(); v = QVBoxLayout(w)
        self._t_table = QTableWidget(0, 3)
        self._t_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._t_table.verticalHeader().setVisible(False)
        self._t_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        v.addWidget(self._t_table, 2)
        self._d_table = QTableWidget(0, 2)
        self._d_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._d_table.verticalHeader().setVisible(False)
        self._d_table.horizontalHeader().setVisible(False)
        self._d_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self._dev_lbl = QLabel(); self._dev_lbl.setStyleSheet("font-weight:700;margin-top:6px;")
        v.addWidget(self._dev_lbl)
        v.addWidget(self._d_table, 2)
        self._tabs.addTab(w, "")

    _EXPECT = (("/scan", 5.0), ("/odom", 10.0), ("/wheel/odom", 10.0), ("/amcl_pose", 0.0), ("/cmd_vel", 0.0))

    def _refresh_topics(self):
        self._t_table.setHorizontalHeaderLabels([tr("dg_topic"), tr("dg_rate"), tr("dg_status")])
        self._t_table.setRowCount(len(self._EXPECT))
        for r, (topic, need) in enumerate(self._EXPECT):
            hz = self._rates.get(topic, 0.0)
            if hz <= 0.0:
                st, col = tr("dg_none"), "#8B949E" if need == 0 else "#FF7B72"
            elif hz < need:
                st, col = tr("dg_slow"), "#E3B341"
            else:
                st, col = tr("dg_ok"), "#3FB950"
            for c, text in enumerate((topic, f"{hz:.1f} Hz", st)):
                it = QTableWidgetItem(text)
                if c == 2:
                    it.setForeground(QColor(col))
                self._t_table.setItem(r, c, it)
        h = self._health
        d = h.driver if h.driver_fresh() else {}
        rows = [
            ("PLC Mega2560", "—" if h.plc_online is None else ("online" if h.plc_online else "offline")),
            ("E-stop", "PRESSED" if h.estop else "released"),
            ("KEYA driver", ("online" if d.get("online") else "offline") if d else "no data"),
            ("Motor output", ("enabled" if d.get("armed") else "disabled") if d else "—"),
            ("Fault codes", "/".join(f"{int(x):04X}" for x in (d.get("fault") or [0, 0])) if d else "—"),
            ("Voltage / Temp", f"{d.get('voltage_v', '—')} V / {d.get('temperature_c', '—')} °C" if d else "—"),
            ("Wheel rpm L/R", ", ".join(str(x) for x in (d.get("speed_rpm") or [])) if d else "—"),
            ("Safety monitor", (h.safety_state or "—") if h.safety_fresh() else "—"),
            ("Localization σ", "—" if h.amcl_sigma_m is None else f"{h.amcl_sigma_m:.3f} m"),
            ("Battery", "—" if h.battery_pct is None else f"{h.battery_pct}%"),
        ]
        self._d_table.setRowCount(len(rows))
        for r, (a, b) in enumerate(rows):
            self._d_table.setItem(r, 0, QTableWidgetItem(a))
            self._d_table.setItem(r, 1, QTableWidgetItem(str(b)))

    # -- plots tab ---------------------------------------------------------------
    def _build_plots(self):
        w = QWidget(); g = QVBoxLayout(w)
        top = QHBoxLayout()
        self._pause_btn = QPushButton(); self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause)
        top.addWidget(self._pause_btn); top.addStretch()
        g.addLayout(top)
        self._p_lin, self._p_ang, self._p_rpm, self._p_volt = (LivePlot() for _ in range(4))
        self._s_lin = (self._p_lin.add_series("cmd", "#58A6FF"), self._p_lin.add_series("actual", "#3FB950"))
        self._s_ang = (self._p_ang.add_series("cmd", "#58A6FF"), self._p_ang.add_series("actual", "#3FB950"))
        self._s_rpm = (self._p_rpm.add_series("L", "#E3B341"), self._p_rpm.add_series("R", "#F778BA"))
        self._s_volt = (self._p_volt.add_series("V", "#FF7B72"),)
        for p_ in (self._p_lin, self._p_ang, self._p_rpm, self._p_volt):
            g.addWidget(p_)
        self._tabs.addTab(w, "")

    def _on_pause(self, on: bool):
        for p_ in (self._p_lin, self._p_ang, self._p_rpm, self._p_volt):
            p_.paused = on
        self._pause_btn.setText(tr("dg_resume") if on else tr("dg_pause"))

    def _on_tick(self):
        if not self.isVisible():
            return
        for plot, idx, cmd, act in ((self._p_lin, self._s_lin, self._cmd[0], self._actual[0]),
                                    (self._p_ang, self._s_ang, self._cmd[1], self._actual[1])):
            plot.push(idx[0], cmd); plot.push(idx[1], act)
        d = self._health.driver if self._health.driver_fresh() else None
        if d:
            rpm = d.get("speed_rpm") or [0, 0]
            if len(rpm) >= 2:
                self._p_rpm.push(self._s_rpm[0], rpm[0]); self._p_rpm.push(self._s_rpm[1], rpm[1])
            if d.get("voltage_v") is not None:
                try:
                    self._p_volt.push(self._s_volt[0], float(d["voltage_v"]))
                except (TypeError, ValueError):
                    pass
        for p_ in (self._p_lin, self._p_ang, self._p_rpm, self._p_volt):
            p_.update()

    # -- logs tab ------------------------------------------------------------------
    def _build_logs(self):
        w = QWidget(); v = QVBoxLayout(w)
        bar = QHBoxLayout()
        self._lv_lbl = QLabel()
        self._lv = QComboBox(); self._lv.addItems(["DEBUG", "INFO", "WARN", "ERROR", "FATAL"]); self._lv.setCurrentIndex(1)
        self._lv.currentIndexChanged.connect(self._rerender_logs)
        self._lsearch = QLineEdit(); self._lsearch.textChanged.connect(self._rerender_logs)
        self._lexp = QPushButton(); self._lexp.clicked.connect(self._export_logs)
        self._lclr = QPushButton(); self._lclr.clicked.connect(self._clear_logs)
        for x in (self._lv_lbl, self._lv, self._lsearch, self._lexp, self._lclr):
            bar.addWidget(x)
        v.addLayout(bar)
        self._log_view = QPlainTextEdit(); self._log_view.setReadOnly(True)
        self._log_view.setMaximumBlockCount(3000)
        self._log_view.setStyleSheet("font-family:monospace;font-size:12px;")
        v.addWidget(self._log_view)
        self._tabs.addTab(w, "")

    def _min_level(self) -> int:
        return (10, 20, 30, 40, 50)[self._lv.currentIndex()]

    def _log_match(self, rec) -> bool:
        _, level, node, text = rec
        if level < self._min_level():
            return False
        q = self._lsearch.text().strip().lower()
        return not q or q in node.lower() or q in text.lower()

    @staticmethod
    def _fmt_log(rec) -> str:
        t, level, node, text = rec
        return f"{datetime.fromtimestamp(t).strftime('%H:%M:%S')} [{_LEVEL_NAMES.get(level, level):5}] {node}: {text}"

    def _rerender_logs(self, *_):
        rows = [self._fmt_log(r) for r in self._logs if self._log_match(r)][-1500:]
        self._log_view.setPlainText("\n".join(rows))

    def _clear_logs(self):
        self._logs.clear(); self._log_view.clear()

    def _export_logs(self):
        path, _ = QFileDialog.getSaveFileName(
            self, tr("dg_export"), os.path.expanduser(f"~/ros_log_{datetime.now():%Y%m%d_%H%M%S}.txt"), "Text (*.txt)")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n".join(self._fmt_log(r) for r in self._logs if self._log_match(r)))

    # -- rosbag tab ------------------------------------------------------------------
    def _build_bag(self):
        w = QWidget(); v = QVBoxLayout(w)
        self._bag_btn = QPushButton(); self._bag_btn.clicked.connect(self._toggle_bag)
        self._bag_auto = QCheckBox(); self._bag_auto.setChecked(False)
        self._bag_status = QLabel(); self._bag_status.setWordWrap(True)
        topics = QLabel("Topics: " + "  ".join(BAG_TOPICS)); topics.setWordWrap(True)
        topics.setStyleSheet("color:#8B949E;")
        for x in (self._bag_btn, self._bag_auto, self._bag_status, topics):
            v.addWidget(x)
        v.addStretch()
        self._tabs.addTab(w, "")

    def bag_running(self) -> bool:
        return self._bag_proc is not None and self._bag_proc.poll() is None

    def _toggle_bag(self):
        self.stop_bag() if self.bag_running() else self.start_bag()

    def start_bag(self, duration_s: int = 0):
        if self.bag_running():
            return
        os.makedirs(BAG_DIR, exist_ok=True)
        self._bag_path = os.path.join(BAG_DIR, datetime.now().strftime("agv_%Y%m%d_%H%M%S"))
        try:
            self._bag_proc = subprocess.Popen(
                ["ros2", "bag", "record", "-o", self._bag_path, *BAG_TOPICS],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        except Exception as exc:
            self._bag_proc = None
            self._bag_status.setText("❌ " + tr("bag_no_ros2", exc))
            return
        if duration_s > 0:
            self._bag_stop_timer.start(duration_s * 1000)
        self._update_bag_ui()

    def stop_bag(self):
        self._bag_stop_timer.stop()
        if self._bag_proc is not None:
            try:
                if self._bag_proc.poll() is None:
                    os.killpg(os.getpgid(self._bag_proc.pid), signal.SIGINT)   # lets rosbag2 finish the file
                    self._bag_proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(self._bag_proc.pid), signal.SIGKILL)
                except Exception:
                    pass
            self._bag_proc = None
        self._update_bag_ui(saved=True)

    def on_alarm(self, alarm: dict):
        """Auto-record when a critical alarm is raised."""
        if self._bag_auto.isChecked() and alarm.get("level") == "critical" and not self.bag_running():
            self.start_bag(duration_s=60)

    def _update_bag_ui(self, saved: bool = False):
        run = self.bag_running()
        self._bag_btn.setText(tr("bag_stop") if run else tr("bag_start"))
        if run:
            self._bag_status.setText("🔴 " + tr("bag_running", self._bag_path))
        elif saved and self._bag_path:
            self._bag_status.setText("✅ " + tr("bag_saved", self._bag_path))
        else:
            self._bag_status.setText(tr("bag_idle"))

    # -- hardware tab ------------------------------------------------------------------
    def _build_hw(self):
        w = QWidget(); v = QVBoxLayout(w)
        form = QFormLayout()
        self._hw_profile = QComboBox(); self._hw_profile.addItems(["model", "real"])
        self._hw_plc = QComboBox(); self._hw_plc.addItems(["auto", "true", "false"])
        self._hw_profile_lbl, self._hw_plc_lbl = QLabel(), QLabel()
        form.addRow(self._hw_profile_lbl, self._hw_profile)
        form.addRow(self._hw_plc_lbl, self._hw_plc)
        v.addLayout(form)
        self._hw_btn = QPushButton(); self._hw_btn.clicked.connect(self._toggle_hw)
        self._hw_status = QLabel(); self._hw_status.setWordWrap(True)
        self._hw_hint = QLabel(); self._hw_hint.setWordWrap(True); self._hw_hint.setStyleSheet("color:#8B949E;")
        for x in (self._hw_btn, self._hw_status, self._hw_hint):
            v.addWidget(x)
        v.addStretch()
        self._tabs.addTab(w, "")

    def hw_running(self) -> bool:
        return self._hw is not None and self._hw.is_running()

    def _toggle_hw(self):
        if self.hw_running():
            self._hw.stop()
            self._hw = None
        else:
            self._hw = ManagedLaunch("Hardware", ["ros2", "launch", "hiep_robot2", "agv_hardware.launch.py"])
            ok, msg = self._hw.start([f"profile:={self._hw_profile.currentText()}",
                                      f"plc:={self._hw_plc.currentText()}"])
            if not ok:
                self._hw_status.setText("❌ " + msg)
                self._hw = None
                return
        self._update_hw_ui()

    def _update_hw_ui(self):
        run = self.hw_running()
        self._hw_btn.setText(tr("hw_stop") if run else tr("hw_start"))
        self._hw_profile.setEnabled(not run); self._hw_plc.setEnabled(not run)
        self._hw_status.setText(
            ("🟢 " + tr("hw_running", f"{self._hw_profile.currentText()} / plc={self._hw_plc.currentText()}"))
            if run else "⬛ " + tr("hw_stopped"))

    def shutdown(self):
        try:
            self.stop_bag()
        except Exception:
            pass
        if self._hw is not None:
            try:
                self._hw.stop()
            except Exception:
                pass

    # -- i18n --------------------------------------------------------------------------
    def retranslate(self):
        for i, k in enumerate(("dg_tab_topics", "dg_tab_plots", "dg_tab_logs", "dg_tab_bag", "dg_tab_hw")):
            self._tabs.setTabText(i, tr(k))
        self._dev_lbl.setText(tr("dg_devices"))
        self._p_lin.set_title(tr("dg_plot_lin")); self._p_ang.set_title(tr("dg_plot_ang"))
        self._p_rpm.set_title(tr("dg_plot_rpm")); self._p_volt.set_title(tr("dg_plot_volt"))
        self._p_lin.rename(0, tr("dg_cmd")); self._p_lin.rename(1, tr("dg_actual"))
        self._p_ang.rename(0, tr("dg_cmd")); self._p_ang.rename(1, tr("dg_actual"))
        self._pause_btn.setText(tr("dg_resume") if self._pause_btn.isChecked() else tr("dg_pause"))
        self._lv_lbl.setText(tr("dg_level")); self._lsearch.setPlaceholderText(tr("dg_search"))
        self._lexp.setText(tr("dg_export")); self._lclr.setText(tr("dg_clear"))
        self._bag_auto.setText(tr("bag_auto")); self._update_bag_ui()
        self._hw_profile_lbl.setText(tr("hw_profile")); self._hw_plc_lbl.setText(tr("hw_plc"))
        self._hw_hint.setText(tr("hw_hint")); self._update_hw_ui()
        self._refresh_topics()
