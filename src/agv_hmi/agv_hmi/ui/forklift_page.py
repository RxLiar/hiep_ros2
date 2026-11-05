"""forklift_page.py - manual control and test of the fork lift (height, insert, pick, drop)."""
from __future__ import annotations

import json
import uuid

from PyQt6.QtCore import Qt, pyqtSignal, QRectF
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QPushButton, QProgressBar,
                             QDoubleSpinBox, QFormLayout, QGridLayout, QTabWidget, QScrollArea, QTextBrowser)

from agv_hmi.core import robot_profile as RP
from agv_hmi.ui.hold_button import HoldButton
from agv_hmi.ui.i18n import tr

# (i18n label, scope, key, min, max, step, decimals)   scope: "top" = profile key, "fork" = profile["fork"][key]
PARAM_GROUPS = [
    ("fk_grp_geom", [("fk_p_flen", "top", "fork_length_m", 0.1, 5.0, 0.05, 2), ("fk_p_fwid", "top", "fork_width_m", 0.1, 3.0, 0.05, 2),
                     ("fk_p_lwid", "top", "load_width_m", 0.1, 3.0, 0.05, 2), ("fk_p_overhang", "fork", "load_overhang_m", 0.0, 1.0, 0.05, 2),
                     ("fk_p_maxh", "top", "max_height_mm", 100, 8000, 10, 0)]),
    ("fk_grp_heights", [("fk_approach_h", "fork", "approach_height_mm", 0, 8000, 10, 0), ("fk_lift_h", "fork", "lift_height_mm", 0, 8000, 10, 0),
                        ("fk_travel_h", "fork", "travel_height_mm", 0, 8000, 10, 0), ("fk_release", "fork", "release_clearance_mm", 0, 500, 5, 0)]),
    ("fk_grp_motion", [("fk_insert", "fork", "insert_depth_m", 0.05, 3.0, 0.05, 2), ("fk_retract", "fork", "retract_m", 0.05, 4.0, 0.05, 2),
                       ("fk_p_insspeed", "fork", "insert_speed_mps", 0.02, 0.30, 0.01, 2), ("fk_p_retspeed", "fork", "retract_speed_mps", 0.02, 0.50, 0.01, 2)]),
    ("fk_grp_bypass", [("fk_p_bahead", "fork", "bypass_ahead_m", 0.2, 4.0, 0.1, 2), ("fk_p_bmargin", "fork", "bypass_margin_m", 0.0, 0.5, 0.05, 2)]),
    ("fk_grp_adv", [("fk_p_tol", "fork", "height_tol_mm", 2, 100, 1, 0), ("fk_p_maxspd", "fork", "max_fork_speed_pct", 10, 100, 5, 0),
                    ("fk_p_minspd", "fork", "min_fork_speed_pct", 5, 100, 5, 0), ("fk_p_slowzone", "fork", "slow_zone_mm", 10, 500, 10, 0),
                    ("fk_p_stall", "fork", "stall_s", 0.5, 10, 0.5, 1), ("fk_p_blocked", "fork", "blocked_s", 1, 20, 0.5, 1),
                    ("fk_p_hto", "fork", "height_step_timeout_s", 5, 300, 5, 0), ("fk_p_dto", "fork", "drive_step_timeout_s", 5, 300, 5, 0)]),
]

_STATE_COLOR = {"idle": "#8B949E", "running": "#58A6FF", "done": "#3FB950", "error": "#F85149"}


class HeightGauge(QWidget):
    def __init__(self):
        super().__init__()
        self.height_mm, self.max_mm, self.top, self.bottom = 0.0, 1600.0, False, False
        self.setMinimumSize(110, 260)

    def set_values(self, h, max_mm, top, bottom):
        self.height_mm, self.max_mm, self.top, self.bottom = h or 0.0, max_mm, top, bottom
        self.update()

    def paintEvent(self, _):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(36, 12, 34, self.height() - 40)
        p.setPen(QPen(QColor(48, 54, 61), 2)); p.setBrush(QBrush(QColor(13, 17, 23))); p.drawRoundedRect(r, 5, 5)
        frac = max(0.0, min(1.0, self.height_mm / max(1.0, self.max_mm)))
        fill = QRectF(r.left() + 3, r.bottom() - 3 - frac * (r.height() - 6), r.width() - 6, frac * (r.height() - 6))
        p.setPen(Qt.PenStyle.NoPen); p.setBrush(QBrush(QColor(88, 166, 255))); p.drawRoundedRect(fill, 4, 4)
        f = QFont(); f.setPixelSize(11); p.setFont(f)
        for k in range(5):
            y = r.bottom() - k * r.height() / 4
            p.setPen(QColor(139, 148, 158)); p.drawText(QRectF(0, y - 7, 33, 14), Qt.AlignmentFlag.AlignRight, f"{int(self.max_mm * k / 4)}")
        p.setPen(QColor("#F85149") if self.top else QColor(72, 79, 88)); p.drawText(QRectF(r.right() + 6, r.top() - 2, 40, 14), Qt.AlignmentFlag.AlignLeft, "TOP")
        p.setPen(QColor("#F85149") if self.bottom else QColor(72, 79, 88)); p.drawText(QRectF(r.right() + 6, r.bottom() - 12, 50, 14), Qt.AlignmentFlag.AlignLeft, "BOTTOM")
        p.setPen(QColor(255, 255, 255)); p.drawText(QRectF(0, self.height() - 22, self.width(), 16), Qt.AlignmentFlag.AlignCenter, f"{self.height_mm:.0f} mm")
        p.end()


class ForkliftPage(QWidget):
    command_requested = pyqtSignal(str)          # JSON for /forklift/command
    params_save_requested = pyqtSignal(dict)     # fork-lift parameters to store in the robot profile

    def __init__(self):
        super().__init__()
        self._profile = RP.normalize(None)
        self._status: dict = {}
        self._alive = False
        self._build()
        self.set_profile(RP.load_store()["profiles"][RP.load_store()["active"]])

    def _spin(self, lo, hi, step, dec, val):
        sp = QDoubleSpinBox(); sp.setRange(lo, hi); sp.setSingleStep(step); sp.setDecimals(dec); sp.setValue(val); return sp

    def _hold(self, key, handler):
        b = HoldButton("", hold_ms=800); b.confirmed.connect(handler); self._btn[key] = b; return b

    def _build(self):
        self._btn = {}
        outer = QVBoxLayout(self); outer.setContentsMargins(8, 8, 8, 8)
        self._tabs = QTabWidget(); outer.addWidget(self._tabs)
        control = QWidget(); self._tabs.addTab(control, "")
        root = QHBoxLayout(control); root.setContentsMargins(6, 8, 6, 6)

        # status
        self._status_box = QGroupBox(); sv = QHBoxLayout(self._status_box)
        self.gauge = HeightGauge(); sv.addWidget(self.gauge)
        col = QVBoxLayout(); sv.addLayout(col, 1)
        self._state = QLabel("—"); self._state.setStyleSheet("font-size:20px;font-weight:700;")
        self._step = QLabel(""); self._carry = QLabel(); self._bypass = QLabel(); self._msg = QLabel(); self._msg.setWordWrap(True)
        self._progress = QProgressBar(); self._progress.setRange(0, 100)
        self._node = QLabel(); self._node.setWordWrap(True)
        for w in (self._state, self._step, self._progress, self._carry, self._bypass, self._msg, self._node):
            col.addWidget(w)
        col.addStretch()
        root.addWidget(self._status_box, 2)

        right = QVBoxLayout(); root.addLayout(right, 3)
        self._manual_box = QGroupBox(); mg = QGridLayout(self._manual_box)
        self._h_lbl = QLabel(); self._h_spin = self._spin(0, 8000, 10, 0, 200)
        mg.addWidget(self._h_lbl, 0, 0); mg.addWidget(self._h_spin, 0, 1)
        mg.addWidget(self._hold("goto", self._goto_height), 0, 2)
        mg.addWidget(self._hold("lower_all", lambda: self._send({"cmd": "lower", "height_mm": 0})), 1, 0)
        mg.addWidget(self._hold("insert", lambda: self._send({"cmd": "insert", **self._insert_params()})), 1, 1)
        mg.addWidget(self._hold("retract", lambda: self._send({"cmd": "retract", "retract_m": self._p["retract"].value()})), 1, 2)
        self._b_carry, self._b_nocarry = QPushButton(), QPushButton()
        self._b_carry.clicked.connect(lambda: self._send({"cmd": "set_carrying", "value": True}))
        self._b_nocarry.clicked.connect(lambda: self._send({"cmd": "set_carrying", "value": False}))
        mg.addWidget(self._b_carry, 2, 0, 1, 1); mg.addWidget(self._b_nocarry, 2, 1, 1, 1)
        right.addWidget(self._manual_box)

        self._seq_box = QGroupBox(); sg = QFormLayout(self._seq_box)
        self._p = {}; self._p_lbl = {}
        for key, lbl, lo, hi, step, dec in (("approach", "fk_approach_h", 0, 8000, 10, 0), ("lift", "fk_lift_h", 0, 8000, 10, 0),
                                            ("drop", "fk_drop_h", 0, 8000, 10, 0), ("release", "fk_release", 0, 500, 5, 0),
                                            ("travel", "fk_travel_h", 0, 8000, 10, 0), ("depth", "fk_insert", 0.05, 3.0, 0.05, 2),
                                            ("retract", "fk_retract", 0.05, 4.0, 0.05, 2)):
            self._p[key] = self._spin(lo, hi, step, dec, 0); self._p_lbl[key] = (QLabel(), lbl); sg.addRow(self._p_lbl[key][0], self._p[key])
        row = QHBoxLayout()
        row.addWidget(self._hold("pick", self._pick)); row.addWidget(self._hold("drop", self._drop))
        sg.addRow(row)
        right.addWidget(self._seq_box)

        self._stop = QPushButton(); self._stop.setStyleSheet("background:#DA3633;color:white;font-weight:800;padding:10px;border-radius:6px;")
        self._stop.clicked.connect(lambda: self._send({"cmd": "stop"}))
        self._reset = QPushButton(); self._reset.clicked.connect(lambda: self._send({"cmd": "reset"}))
        r2 = QHBoxLayout(); r2.addWidget(self._stop, 2); r2.addWidget(self._reset, 1); right.addLayout(r2)
        self._warn = QLabel(); self._warn.setWordWrap(True); self._warn.setStyleSheet("color:#E3B341;")
        self._not_fl = QLabel(); self._not_fl.setWordWrap(True); self._not_fl.setStyleSheet("color:#F85149;font-weight:700;")
        right.addWidget(self._warn); right.addWidget(self._not_fl); right.addStretch()
        self._test_hint = QLabel(); self._test_hint.setWordWrap(True); self._test_hint.setStyleSheet("color:#8B949E;")
        self._seq_box.layout().insertRow(0, self._test_hint)

        self._build_params_tab()
        self._guide = QTextBrowser(); self._guide.setOpenExternalLinks(False)
        self._tabs.addTab(self._guide, "")
        self.retranslate()

    def _build_params_tab(self):
        page = QScrollArea(); page.setWidgetResizable(True)
        host = QWidget(); page.setWidget(host); v = QVBoxLayout(host)
        self._prof_lbl = QLabel(); self._prof_lbl.setStyleSheet("font-weight:700;"); v.addWidget(self._prof_lbl)
        self._pw = {}; self._pw_lbl = {}; self._grp_boxes = {}
        grid = QGridLayout(); v.addLayout(grid)
        for gi, (gkey, fields) in enumerate(PARAM_GROUPS):
            box = QGroupBox(); form = QFormLayout(box); self._grp_boxes[gkey] = box
            for lbl_key, scope, key, lo, hi, step, dec in fields:
                sp = QDoubleSpinBox(); sp.setRange(lo, hi); sp.setSingleStep(step); sp.setDecimals(dec)
                sp.setKeyboardTracking(False)               # commit on Enter / Tab, never mid-typing
                sp.valueChanged.connect(self._mark_dirty)
                lbl = QLabel(); form.addRow(lbl, sp)
                self._pw[(scope, key)] = sp; self._pw_lbl[(scope, key)] = (lbl, lbl_key)
            grid.addWidget(box, gi // 2, gi % 2)
        row = QHBoxLayout()
        self._save_btn = QPushButton(); self._save_btn.setObjectName("BtnSuccess"); self._save_btn.clicked.connect(self._save_params)
        self._defaults_btn = QPushButton(); self._defaults_btn.clicked.connect(self._restore_defaults)
        row.addWidget(self._save_btn, 2); row.addWidget(self._defaults_btn, 1); v.addLayout(row)
        self._params_status = QLabel(); self._params_status.setWordWrap(True); v.addWidget(self._params_status)
        self._sync_lbl = QLabel(); self._sync_lbl.setWordWrap(True); v.addWidget(self._sync_lbl)
        self._where_lbl = QLabel(); self._where_lbl.setWordWrap(True); self._where_lbl.setStyleSheet("color:#8B949E;font-size:11px;")
        v.addWidget(self._where_lbl); v.addStretch()
        self._tabs.addTab(page, "")

    # -- parameters tab ---------------------------------------------------------
    def _fill_params(self, p: dict):
        for (scope, key), sp in self._pw.items():
            sp.blockSignals(True)
            sp.setValue(float(p[key] if scope == "top" else p["fork"][key]))
            sp.blockSignals(False)
        self._params_dirty = False
        self._params_status.setText("")
        self._prof_lbl.setText(tr("fk_profile_is", p["name"], tr("rt_" + p["robot_type"])))

    def _mark_dirty(self, *_):
        self._params_dirty = True
        self._params_status.setStyleSheet("color:#E3B341;"); self._params_status.setText(tr("fk_unsaved"))

    def _collect_patch(self) -> dict:
        patch = {"fork": {}}
        for (scope, key), sp in self._pw.items():
            (patch if scope == "top" else patch["fork"])[key] = sp.value()
        return patch

    def _save_params(self):
        self.params_save_requested.emit(self._collect_patch())
        self._params_status.setStyleSheet("color:#3FB950;"); self._params_status.setText("✅ " + tr("fk_saved"))

    def _restore_defaults(self):
        d = RP.normalize(None)
        for (scope, key), sp in self._pw.items():
            sp.blockSignals(True); sp.setValue(float(d[key] if scope == "top" else d["fork"][key])); sp.blockSignals(False)
        self._mark_dirty()

    def _check_sync(self, cfg):
        if not cfg:
            self._sync_lbl.setStyleSheet("color:#8B949E;"); self._sync_lbl.setText(tr("fk_sync_none")); return
        want = RP.fork_effective_config(self._profile)
        diff = [k for k, v in want.items() if k in cfg and abs(float(cfg[k]) - float(v)) > 1e-3]
        if diff:
            self._sync_lbl.setStyleSheet("color:#E3B341;"); self._sync_lbl.setText(tr("fk_sync_bad", ", ".join(diff)))
        else:
            self._sync_lbl.setStyleSheet("color:#3FB950;"); self._sync_lbl.setText(tr("fk_sync_ok"))

    # -- commands ---------------------------------------------------------------
    def _send(self, cmd: dict):
        cmd = dict(cmd); cmd.setdefault("id", uuid.uuid4().hex[:10])
        self.command_requested.emit(json.dumps(cmd))

    def _insert_params(self):
        return {"insert_depth_m": self._p["depth"].value()}

    def _goto_height(self):
        self._send({"cmd": "to_height", "height_mm": self._h_spin.value()})

    def _pick(self):
        self._send({"cmd": "pick", "approach_height_mm": self._p["approach"].value(), "insert_depth_m": self._p["depth"].value(),
                    "lift_height_mm": self._p["lift"].value(), "retract_m": self._p["retract"].value()})

    def _drop(self):
        self._send({"cmd": "drop", "drop_height_mm": self._p["drop"].value(), "insert_depth_m": self._p["depth"].value(),
                    "release_clearance_mm": self._p["release"].value(), "retract_m": self._p["retract"].value(),
                    "travel_height_mm": self._p["travel"].value()})

    # -- data in ---------------------------------------------------------------
    def set_profile(self, p: dict):
        self._profile = RP.normalize(p)
        self._fill_params(self._profile)
        self._check_sync(self._status.get("cfg"))
        f = self._profile["fork"]
        for key, val in (("approach", f["approach_height_mm"]), ("lift", f["lift_height_mm"]), ("drop", 900.0),
                         ("release", f["release_clearance_mm"]), ("travel", f["travel_height_mm"]),
                         ("depth", f["insert_depth_m"]), ("retract", f["retract_m"])):
            self._p[key].setValue(val)
        fl = RP.is_forklift(self._profile)
        for w in (self._manual_box, self._seq_box):
            w.setEnabled(fl)
        self._not_fl.setText("" if fl else tr("fk_not_forklift"))

    def feed_status(self, st: dict):
        self._status = st
        state = st.get("state", "idle")
        self._state.setText(tr(f"fk_state_{state}")); self._state.setStyleSheet(f"font-size:20px;font-weight:700;color:{_STATE_COLOR.get(state, '#8B949E')};")
        self._step.setText(str(st.get("step", "")) or "—")
        self._progress.setValue(int(100 * float(st.get("progress", 0))))
        self._carry.setText("📦 " + tr("fk_carry_yes" if st.get("carrying") else "fk_carry_no"))
        self._msg.setText(str(st.get("message", "")))
        self.gauge.set_values(st.get("height_mm"), self._profile["max_height_mm"], bool(st.get("top")), bool(st.get("bottom")))
        self._check_sync(st.get("cfg"))

    def feed_bypass(self, ids: list):
        self._bypass.setText(tr("fk_bypass_on", ", ".join(ids)) if ids else "")
        self._bypass.setStyleSheet("color:#E3B341;font-weight:700;")

    def set_node_alive(self, alive: bool):
        self._alive = alive
        self._node.setText("" if alive else "⚠ " + tr("fk_no_node"))
        self._node.setStyleSheet("color:#E3B341;")

    def is_running(self) -> bool:
        return self._status.get("state") == "running"

    def retranslate(self):
        for i, k in enumerate(("fk_tab_control", "fk_tab_params", "fk_tab_guide")):
            self._tabs.setTabText(i, tr(k))
        self._guide.setHtml(tr("fk_guide"))
        for gkey, box in self._grp_boxes.items():
            box.setTitle(tr(gkey))
        for (_s, _k), (lbl, lk) in self._pw_lbl.items():
            lbl.setText(tr(lk))
        self._save_btn.setText(tr("fk_save")); self._defaults_btn.setText(tr("fk_defaults")); self._test_hint.setText(tr("fk_test_hint"))
        import os as _os
        from agv_hmi.ui.route_manager import ROUTES_DIR
        self._where_lbl.setText(tr("fk_where", RP.PROFILE_FILE, _os.path.join(ROUTES_DIR, "routes.json")))
        self._fill_params(self._profile) if not getattr(self, "_params_dirty", False) else None
        self._status_box.setTitle(tr("fk_status_box")); self._manual_box.setTitle(tr("fk_manual_box")); self._seq_box.setTitle(tr("fk_seq_box"))
        self._h_lbl.setText(tr("fk_height"))
        for key, text in (("goto", "fk_goto_h"), ("lower_all", "fk_lower_all"), ("insert", "fk_insert_btn"),
                          ("retract", "fk_retract_btn"), ("pick", "fk_pick"), ("drop", "fk_drop")):
            self._btn[key].setText(tr(text))
        for key, (lbl, k) in self._p_lbl.items():
            lbl.setText(tr(k))
        self._b_carry.setText(tr("fk_set_carry")); self._b_nocarry.setText(tr("fk_clear_carry"))
        self._stop.setText(tr("fk_stop")); self._reset.setText(tr("fk_reset")); self._warn.setText("⚠ " + tr("fk_warn"))
        self.set_node_alive(self._alive)
