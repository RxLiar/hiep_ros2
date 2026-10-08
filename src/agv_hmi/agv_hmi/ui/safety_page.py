"""safety_page.py - robot size + draggable SAFETY (slow) and STOP rectangles around the robot."""
from __future__ import annotations

import copy
import json
import math

import numpy as np
from PyQt6.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QFormLayout, QLabel, QDoubleSpinBox, QSpinBox,
    QCheckBox, QPushButton, QComboBox, QLineEdit, QScrollArea, QInputDialog, QMessageBox, QSplitter, QGridLayout)

from agv_hmi.core import robot_profile as RP
from agv_hmi.ui.i18n import tr

_STATE_COLOR = {"ok": "#3FB950", "slow": "#E3B341", "stop": "#F85149", "no_scan": "#F85149",
                "disabled": "#8B949E", "unknown": "#8B949E"}
HANDLE_R = 9
SNAP = 0.05


class SafetyEditor(QWidget):
    """Top view of the robot (front = up). Edges of the two rectangles are draggable."""
    changed = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.p = RP.normalize(None)
        self.pts = np.zeros((0, 2))
        self.state: dict = {}
        self._zoom = 1.0
        self._drag = None                  # (kind, side)
        self._fit = None
        self._hover = None
        self.setMouseTracking(True)
        self.setMinimumSize(420, 420)

    # -- data ---------------------------------------------------------------
    def set_profile(self, p: dict):
        self.p = RP.normalize(p)
        if self._drag is None:
            self._recompute_fit()
        self.update()

    def set_scan_points(self, pts: np.ndarray):
        self.pts = pts; self.update()

    def set_state(self, st: dict):
        self.state = st or {}; self.update()

    # -- transform: footprint centre at the widget centre, front = up ---------
    def _recompute_fit(self):
        """World half-extents shown: the largest zone + a margin. Frozen while dragging."""
        p = self.p
        mx = max(max(p["slow"]["front"], p["slow"]["rear"]), 0.8)
        my = max(max(p["slow"]["left"], p["slow"]["right"]), 0.8)
        self._fit = (p["length_m"] / 2 + mx + 0.6, p["width_m"] / 2 + my + 0.6)

    def _scale(self) -> float:
        if self._fit is None:
            self._recompute_fit()
        half_x, half_y = self._fit
        return min(self.height() / (2 * half_x), self.width() / (2 * half_y)) * self._zoom

    def _to_px(self, x: float, y: float) -> QPointF:
        s = self._scale(); p = self.p
        return QPointF(self.width() / 2 - (y - p["center_y_m"]) * s, self.height() / 2 - (x - p["center_x_m"]) * s)

    def _to_world(self, px: float, py: float):
        s = self._scale(); p = self.p
        return (p["center_x_m"] - (py - self.height() / 2) / s, p["center_y_m"] - (px - self.width() / 2) / s)

    def _rect_px(self, rect):
        x0, x1, y0, y1 = rect
        a, b = self._to_px(x1, y1), self._to_px(x0, y0)          # front-left , rear-right
        return QRectF(min(a.x(), b.x()), min(a.y(), b.y()), abs(a.x() - b.x()), abs(a.y() - b.y()))

    def _handles(self):
        out = {}
        for kind in ("slow", "stop"):
            x0, x1, y0, y1 = RP.zone_rect(self.p, kind)
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            out[(kind, "front")] = self._to_px(x1, self.p["center_y_m"])
            out[(kind, "rear")] = self._to_px(x0, self.p["center_y_m"])
            out[(kind, "left")] = self._to_px(self.p["center_x_m"], y1)
            out[(kind, "right")] = self._to_px(self.p["center_x_m"], y0)
        return out

    def _hit(self, pos: QPointF):
        best, bd = None, HANDLE_R * 1.6
        for key, pt in self._handles().items():
            d = math.hypot(pt.x() - pos.x(), pt.y() - pos.y())
            if d < bd or (d == bd and key[0] == "stop"):
                best, bd = key, d
        return best

    # -- mouse ----------------------------------------------------------------
    def mousePressEvent(self, ev):
        if ev.button() == Qt.MouseButton.LeftButton:
            self._drag = self._hit(ev.position())

    def mouseReleaseEvent(self, ev):
        self._drag = None
        self._recompute_fit(); self.update()

    def mouseMoveEvent(self, ev):
        if self._drag is None:
            self._hover = self._hit(ev.position())
            self.setCursor(Qt.CursorShape.SizeVerCursor if self._hover and self._hover[1] in ("front", "rear")
                           else Qt.CursorShape.SizeHorCursor if self._hover else Qt.CursorShape.ArrowCursor)
            self.update(); return
        kind, side = self._drag
        x, y = self._to_world(ev.position().x(), ev.position().y())
        fx0, fx1, fy0, fy1 = RP.footprint_rect(self.p)
        raw = {"front": x - fx1, "rear": fx0 - x, "left": y - fy1, "right": fy0 - y}[side]
        m = round(max(0.0, min(RP.MAX_MARGIN, raw)) / SNAP) * SNAP
        hx, hy = self._fit
        m = min(m, {"front": hx, "rear": hx, "left": hy, "right": hy}[side] - 0.15 - {"front": self.p["length_m"] / 2, "rear": self.p["length_m"] / 2, "left": self.p["width_m"] / 2, "right": self.p["width_m"] / 2}[side])
        if kind == "stop":
            m = min(m, self.p["slow"][side])
        else:
            m = max(m, self.p["stop"][side])
        if abs(m - self.p[kind][side]) > 1e-9:
            self.p[kind][side] = round(m, 3)
            self.changed.emit(); self.update()

    def wheelEvent(self, ev):
        self._zoom = max(0.5, min(3.0, self._zoom * (1.1 if ev.angleDelta().y() > 0 else 1 / 1.1)))
        self.update()

    # -- paint ------------------------------------------------------------------
    def paintEvent(self, _):
        pa = QPainter(self); pa.setRenderHint(QPainter.RenderHint.Antialiasing)
        pa.fillRect(self.rect(), QColor(13, 17, 23))
        s = self._scale(); p = self.p
        # 1 m grid
        pa.setPen(QPen(QColor(33, 38, 45), 1))
        for k in range(-6, 7):
            a = self._to_px(p["center_x_m"] + k, p["center_y_m"] - 6); b = self._to_px(p["center_x_m"] + k, p["center_y_m"] + 6)
            pa.drawLine(a, b)
            a = self._to_px(p["center_x_m"] - 6, p["center_y_m"] + k); b = self._to_px(p["center_x_m"] + 6, p["center_y_m"] + k)
            pa.drawLine(a, b)
        # zones
        slow = self._rect_px(RP.zone_rect(p, "slow")); stop = self._rect_px(RP.zone_rect(p, "stop"))
        pa.setPen(QPen(QColor(240, 136, 62), 2, Qt.PenStyle.DashLine)); pa.setBrush(QBrush(QColor(240, 136, 62, 40)))
        pa.drawRect(slow)
        pa.setPen(QPen(QColor(248, 81, 73), 2)); pa.setBrush(QBrush(QColor(248, 81, 73, 60)))
        pa.drawRect(stop)
        # lidar points coloured by zone
        if len(self.pts):
            cls = RP.classify_points(self.pts, p)
            for c, col in ((1, QColor(201, 209, 217, 200)), (2, QColor(240, 136, 62)), (3, QColor(248, 81, 73))):
                pa.setPen(Qt.PenStyle.NoPen); pa.setBrush(QBrush(col))
                for x, y in self.pts[cls == c][:1500]:
                    pt = self._to_px(x, y)
                    if -10 < pt.x() < self.width() + 10 and -10 < pt.y() < self.height() + 10:
                        pa.drawEllipse(pt, 2.2, 2.2)
        # robot body
        body = self._rect_px(RP.footprint_rect(p))
        pa.setPen(QPen(QColor(255, 255, 255, 220), 1.5)); pa.setBrush(QBrush(QColor(55, 138, 221, 200)))
        pa.drawRoundedRect(body, 8, 8)
        arrow = QPolygonF([QPointF(body.center().x(), body.top() + 6), QPointF(body.center().x() - 9, body.top() + 24),
                           QPointF(body.center().x() + 9, body.top() + 24)])
        pa.setPen(Qt.PenStyle.NoPen); pa.setBrush(QBrush(QColor(255, 255, 255, 230))); pa.drawPolygon(arrow)
        # base_link + lidar
        o = self._to_px(0.0, 0.0)
        pa.setPen(QPen(QColor(255, 255, 255, 220), 1.5)); pa.drawLine(QPointF(o.x() - 6, o.y()), QPointF(o.x() + 6, o.y())); pa.drawLine(QPointF(o.x(), o.y() - 6), QPointF(o.x(), o.y() + 6))
        li = self._to_px(p["lidar_x_m"], p["lidar_y_m"])
        pa.setBrush(QBrush(QColor(63, 185, 80))); pa.setPen(QPen(QColor(255, 255, 255), 1.2)); pa.drawEllipse(li, 6, 6)
        f = QFont(); f.setPixelSize(11); pa.setFont(f); pa.setPen(QColor(201, 209, 217))
        pa.drawText(QPointF(li.x() + 10, li.y() + 4), "LiDAR"); pa.drawText(QPointF(o.x() + 8, o.y() + 16), "base_link")
        # handles + margin labels
        for (kind, side), pt in self._handles().items():
            col = QColor(248, 81, 73) if kind == "stop" else QColor(240, 136, 62)
            active = self._drag == (kind, side) or self._hover == (kind, side)
            pa.setPen(QPen(QColor(255, 255, 255), 2 if active else 1)); pa.setBrush(QBrush(col))
            pa.drawEllipse(pt, HANDLE_R if active else HANDLE_R - 2, HANDLE_R if active else HANDLE_R - 2)
            text = f"{p[kind][side]:.2f}"
            fm = pa.fontMetrics(); tw = fm.horizontalAdvance(text) + 8
            if side == "front":
                lx, ly = pt.x() + 12, pt.y() - 10 if kind == "slow" else pt.y() + 2
            elif side == "rear":
                lx, ly = pt.x() + 12, pt.y() - 2 if kind == "slow" else pt.y() - 14
            elif side == "left":
                lx, ly = pt.x() - tw - 10, pt.y() - 8 + (0 if kind == "slow" else 14)
            else:
                lx, ly = pt.x() + 10, pt.y() - 8 + (0 if kind == "slow" else 14)
            pa.setPen(Qt.PenStyle.NoPen); pa.setBrush(QBrush(QColor(13, 17, 23, 210)))
            pa.drawRoundedRect(QRectF(lx, ly, tw, 16), 4, 4)
            pa.setPen(col.lighter(160)); pa.drawText(QRectF(lx, ly, tw, 16), Qt.AlignmentFlag.AlignCenter, text)
        # state badge
        st = self.state.get("state", "unknown")
        col = QColor(_STATE_COLOR.get(st, "#8B949E"))
        pa.setPen(Qt.PenStyle.NoPen); pa.setBrush(QBrush(col)); pa.drawRoundedRect(QRectF(10, 10, 200, 30), 8, 8)
        f2 = QFont(); f2.setPixelSize(14); f2.setBold(True); pa.setFont(f2); pa.setPen(QColor(13, 17, 23) if st in ("ok", "slow") else QColor(255, 255, 255))
        pa.drawText(QRectF(10, 10, 200, 30), Qt.AlignmentFlag.AlignCenter, tr(f"sf_state_{st}"))
        if self.state.get("nearest_m") is not None and st != "unknown":
            pa.setFont(f); pa.setPen(QColor(201, 209, 217))
            pa.drawText(QPointF(14, 56), tr("sf_nearest", self.state["nearest_m"]))
        pa.end()


class SafetyPage(QWidget):
    profile_applied = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._store = RP.load_store()
        self._loading = False
        self._dirty = False
        self._build()
        self._load_active()

    # -- UI --------------------------------------------------------------------
    def _spin(self, lo, hi, step, dec=2):
        sp = QDoubleSpinBox(); sp.setRange(lo, hi); sp.setSingleStep(step); sp.setDecimals(dec)
        sp.valueChanged.connect(self._on_form_changed); return sp

    def _build(self):
        root = QHBoxLayout(self); root.setContentsMargins(10, 10, 10, 10)
        left = QVBoxLayout(); root.addLayout(left, 3)
        self.editor = SafetyEditor(); self.editor.changed.connect(self._on_editor_changed)
        left.addWidget(self.editor, 1)
        self._legend = QLabel(); self._hint = QLabel()
        left.addWidget(self._legend); left.addWidget(self._hint)

        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setMinimumWidth(360); scroll.setMaximumWidth(460)
        panel = QWidget(); v = QVBoxLayout(panel); scroll.setWidget(panel); root.addWidget(scroll, 2)

        # profile row
        self._prof_box = QGroupBox(); pr = QHBoxLayout(self._prof_box)
        self._combo = QComboBox(); self._combo.currentTextChanged.connect(self._on_profile_switch)
        self._b_new, self._b_dup, self._b_del = QPushButton(), QPushButton(), QPushButton()
        self._b_new.clicked.connect(lambda: self._new_profile(False)); self._b_dup.clicked.connect(lambda: self._new_profile(True))
        self._b_del.clicked.connect(self._delete_profile)
        pr.addWidget(self._combo, 1)
        for b in (self._b_new, self._b_dup, self._b_del):
            pr.addWidget(b)
        v.addWidget(self._prof_box)

        # robot size
        self._robot_box = QGroupBox(); f = QFormLayout(self._robot_box)
        self._name = QLineEdit(); self._name.textChanged.connect(self._on_form_changed)
        self._len, self._wid = self._spin(0.1, 10, 0.05), self._spin(0.1, 5, 0.05)
        self._cx, self._cy = self._spin(-5, 5, 0.05), self._spin(-2, 2, 0.05)
        self._lbl = {k: QLabel() for k in ("name", "len", "wid", "cx", "cy")}
        for k, w in (("name", self._name), ("len", self._len), ("wid", self._wid), ("cx", self._cx), ("cy", self._cy)):
            f.addRow(self._lbl[k], w)
        v.addWidget(self._robot_box)

        # lidar
        self._lidar_box = QGroupBox(); f = QFormLayout(self._lidar_box)
        self._lx, self._ly, self._lyaw = self._spin(-5, 5, 0.05), self._spin(-2, 2, 0.05), self._spin(-180, 180, 1, 0)
        self._llbl = {k: QLabel() for k in ("lx", "ly", "lyaw")}
        for k, w in (("lx", self._lx), ("ly", self._ly), ("lyaw", self._lyaw)):
            f.addRow(self._llbl[k], w)
        v.addWidget(self._lidar_box)

        # zones
        self._zone_box = QGroupBox(); g = QGridLayout(self._zone_box)
        self._zs = {}
        self._zone_titles = {}
        for r, kind in enumerate(("stop", "slow")):
            t = QLabel(); t.setStyleSheet("font-weight:700;"); self._zone_titles[kind] = t
            g.addWidget(t, r * 3, 0, 1, 4)
            for c, side in enumerate(("front", "rear", "left", "right")):
                lbl = QLabel(); lbl.setObjectName(f"zl_{side}"); g.addWidget(lbl, r * 3 + 1, c)
                sp = self._spin(0.0, RP.MAX_MARGIN, 0.05); self._zs[(kind, side)] = sp; g.addWidget(sp, r * 3 + 2, c)
        v.addWidget(self._zone_box)

        # behaviour
        self._beh_box = QGroupBox(); f = QFormLayout(self._beh_box)
        self._slin, self._sang = self._spin(0.02, 2.0, 0.05), self._spin(0.05, 3.0, 0.05)
        self._minpts = QSpinBox(); self._minpts.setRange(1, 50); self._minpts.valueChanged.connect(self._on_form_changed)
        self._enabled, self._require = QCheckBox(), QCheckBox()
        for c in (self._enabled, self._require):
            c.toggled.connect(self._on_form_changed)
        self._blbl = {k: QLabel() for k in ("slin", "sang", "minpts")}
        for k, w in (("slin", self._slin), ("sang", self._sang), ("minpts", self._minpts)):
            f.addRow(self._blbl[k], w)
        f.addRow(self._enabled); f.addRow(self._require)
        v.addWidget(self._beh_box)

        self._fp = QLabel(); self._fp.setWordWrap(True); self._fp.setStyleSheet("color:#8B949E;font-size:11px;")
        self._warn = QLabel(); self._warn.setWordWrap(True); self._warn.setStyleSheet("color:#E3B341;")
        self._status = QLabel(); self._status.setWordWrap(True)
        row = QHBoxLayout(); self._b_apply, self._b_revert = QPushButton(), QPushButton(); self._b_apply.setObjectName("BtnSuccess")
        self._b_apply.clicked.connect(self.apply); self._b_revert.clicked.connect(self._load_active)
        row.addWidget(self._b_apply, 2); row.addWidget(self._b_revert, 1)
        for w in (self._fp, self._warn, self._status):
            v.addWidget(w)
        v.addLayout(row); v.addStretch()
        self.retranslate()

    # -- model <-> form ------------------------------------------------------------
    def _current(self) -> dict:
        p = self.editor.p
        return RP.normalize({
            "name": self._name.text(), "length_m": self._len.value(), "width_m": self._wid.value(),
            "center_x_m": self._cx.value(), "center_y_m": self._cy.value(),
            "lidar_x_m": self._lx.value(), "lidar_y_m": self._ly.value(), "lidar_yaw_deg": self._lyaw.value(),
            "stop": {s: self._zs[("stop", s)].value() for s in RP.SIDES},
            "slow": {s: self._zs[("slow", s)].value() for s in RP.SIDES},
            "slow_linear_mps": self._slin.value(), "slow_angular_rps": self._sang.value(),
            "safety_enabled": self._enabled.isChecked(), "require_scan": self._require.isChecked(),
            "min_points": self._minpts.value()})

    def _fill_form(self, p: dict):
        self._loading = True
        self._name.setText(p["name"]); self._len.setValue(p["length_m"]); self._wid.setValue(p["width_m"])
        self._cx.setValue(p["center_x_m"]); self._cy.setValue(p["center_y_m"])
        self._lx.setValue(p["lidar_x_m"]); self._ly.setValue(p["lidar_y_m"]); self._lyaw.setValue(p["lidar_yaw_deg"])
        for kind in ("stop", "slow"):
            for s in RP.SIDES:
                self._zs[(kind, s)].setValue(p[kind][s])
        self._slin.setValue(p["slow_linear_mps"]); self._sang.setValue(p["slow_angular_rps"])
        self._minpts.setValue(p["min_points"]); self._enabled.setChecked(p["safety_enabled"]); self._require.setChecked(p["require_scan"])
        self._loading = False
        self.editor.set_profile(p); self._update_fp()

    def _load_active(self):
        self._store = RP.load_store()
        self._loading = True
        self._combo.clear(); self._combo.addItems(list(self._store["profiles"]))
        self._combo.setCurrentText(self._store["active"]); self._loading = False
        self._fill_form(RP.active_profile(self._store))
        self._set_dirty(False); self._status.setText("")

    def _on_form_changed(self, *_):
        if self._loading:
            return
        self.editor.set_profile(self._current())
        self._fill_zone_spins_from_editor()
        self._update_fp(); self._set_dirty(True)

    def _fill_zone_spins_from_editor(self):
        self._loading = True
        for kind in ("stop", "slow"):
            for s in RP.SIDES:
                self._zs[(kind, s)].setValue(self.editor.p[kind][s])
        self._loading = False

    def _on_editor_changed(self):
        self._fill_zone_spins_from_editor(); self._update_fp(); self._set_dirty(True)

    def _update_fp(self):
        self._fp.setText(tr("sf_footprint") + " " + RP.nav2_footprint(self.editor.p))

    def _set_dirty(self, d: bool):
        self._dirty = d
        if d:
            self._status.setStyleSheet("color:#E3B341;"); self._status.setText(tr("sf_dirty"))

    # -- profiles ------------------------------------------------------------------
    def _on_profile_switch(self, name: str):
        if self._loading or not name or name not in self._store["profiles"]:
            return
        self._fill_form(self._store["profiles"][name]); self._set_dirty(True)

    def _new_profile(self, duplicate: bool):
        name, ok = QInputDialog.getText(self, tr("page_safety"), tr("sf_prompt_name"))
        name = name.strip()
        if not ok or not name:
            return
        base = self._current() if duplicate else RP.normalize(None)
        base["name"] = name
        self._store["profiles"][name] = RP.normalize(base); self._store["active"] = name
        self._loading = True; self._combo.clear(); self._combo.addItems(list(self._store["profiles"]))
        self._combo.setCurrentText(name); self._loading = False
        self._fill_form(self._store["profiles"][name]); self._set_dirty(True)

    def _delete_profile(self):
        if len(self._store["profiles"]) <= 1:
            return
        name = self._combo.currentText()
        self._store["profiles"].pop(name, None)
        self._store["active"] = next(iter(self._store["profiles"]))
        RP.save_store(self._store); self._load_active()

    def apply(self):
        p = self._current()
        old = self._combo.currentText()
        if old in self._store["profiles"] and old != p["name"]:
            self._store["profiles"].pop(old)                  # renamed
        self._store["profiles"][p["name"]] = p; self._store["active"] = p["name"]
        RP.save_store(self._store)
        self._loading = True; self._combo.clear(); self._combo.addItems(list(self._store["profiles"]))
        self._combo.setCurrentText(p["name"]); self._loading = False
        self.editor.set_profile(p)
        self._dirty = False
        self._status.setStyleSheet("color:#3FB950;"); self._status.setText("✅ " + tr("sf_applied"))
        self.profile_applied.emit(p)

    def active_profile(self) -> dict:
        return RP.active_profile(self._store)

    # -- live data ---------------------------------------------------------------------
    def feed_scan(self, angle_min, angle_inc, range_min, range_max, ranges):
        self.editor.set_scan_points(RP.scan_to_base(ranges, angle_min, angle_inc, range_min, range_max, self.editor.p))

    def feed_state(self, payload: str):
        try:
            d = json.loads(payload)
            if isinstance(d, dict):
                self.editor.set_state(d)
        except ValueError:
            pass

    def mark_state_stale(self):
        self.editor.set_state({"state": "unknown"})

    # -- i18n -------------------------------------------------------------------------
    def retranslate(self):
        self._prof_box.setTitle(tr("sf_profile")); self._robot_box.setTitle(tr("sf_robot_box"))
        self._lidar_box.setTitle(tr("sf_lidar_box")); self._zone_box.setTitle(tr("sf_zone_box")); self._beh_box.setTitle(tr("sf_beh_box"))
        for b, k in ((self._b_new, "sf_new"), (self._b_dup, "sf_dup"), (self._b_del, "sf_del"),
                     (self._b_apply, "sf_apply"), (self._b_revert, "sf_revert")):
            b.setText(tr(k))
        for k, key in (("name", "sf_name"), ("len", "sf_length"), ("wid", "sf_width"), ("cx", "sf_cx"), ("cy", "sf_cy")):
            self._lbl[k].setText(tr(key))
        for k, key in (("lx", "sf_lx"), ("ly", "sf_ly"), ("lyaw", "sf_lyaw")):
            self._llbl[k].setText(tr(key))
        for k, key in (("slin", "sf_slow_lin"), ("sang", "sf_slow_ang"), ("minpts", "sf_minpts")):
            self._blbl[k].setText(tr(key))
        self._zone_titles["stop"].setText("🟥 " + tr("sf_stop")); self._zone_titles["slow"].setText("🟧 " + tr("sf_slow"))
        for side in RP.SIDES:
            for lbl in self._zone_box.findChildren(QLabel, f"zl_{side}"):
                lbl.setText(tr(f"sf_{side}"))
        self._enabled.setText(tr("sf_enabled")); self._require.setText(tr("sf_require"))
        self._warn.setText("⚠ " + tr("sf_warn")); self._legend.setText(tr("sf_legend")); self._hint.setText(tr("sf_view_hint"))
        self._update_fp(); self.editor.update()
