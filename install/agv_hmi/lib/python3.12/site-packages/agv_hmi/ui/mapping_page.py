import os
import math

import threading

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QGroupBox, QLabel,
    QPushButton, QScrollArea, QFileDialog, QDoubleSpinBox, QCheckBox,
    QFormLayout, QMessageBox, QGridLayout, QButtonGroup, QSpinBox
)
from PyQt6.QtCore import Qt, pyqtSignal

from agv_hmi.ui.collapsible import CollapsiblePanel, make_groups_collapsible
from agv_hmi.ui.map_widget import MapWidget
from agv_hmi.ui.error_header import ErrorHeader
from agv_hmi.ui.joystick_widget import JoystickWidget
from agv_hmi.ui.velocity_input import VelocityInputPanel
from agv_hmi.ui.i18n import tr
from agv_hmi.ui.process_manager import ManagedLaunch
from agv_hmi.core.map_cleaner import clean_map, CleanParams
from agv_hmi.core import map_io
from agv_hmi.ui.map_widget import MODE_GOAL, MODE_MEASURE, MODE_EDIT, EDIT_TOOLS
from agv_hmi.core import zones as zones_io


MAPPING_CMD = ["ros2", "launch", "mec_mobile_navigation", "mapping.launch.py"]

MAPPING_KILL_PATTERNS = [
    "mapping.launch.py",
    "slam_toolbox",
    "async_slam_toolbox_node",
    "sync_slam_toolbox_node",
    "map_saver",
]


def _mono(t):
    w = QLabel(t)
    w.setObjectName("MonoVal")
    return w


def _btn(t, obj="", h=32, enabled=True):
    b = QPushButton(t)
    if obj:
        b.setObjectName(obj)
    b.setFixedHeight(h)
    b.setEnabled(enabled)
    return b


class MappingPage(QWidget):
    velocity_signal = pyqtSignal(float, float)
    save_map_requested = pyqtSignal(str)
    mapping_started = pyqtSignal()
    mapping_stopped = pyqtSignal()
    _clean_finished = pyqtSignal(object, object)   # (result | None, error str | None)

    def __init__(self):
        super().__init__()
        self._raw_view = None          # (grid, res, ox, oy) of the live map while a clean map is shown
        self._clean_result = None      # (CleanResult, params, res, ox, oy)
        self._cleaned_view = False
        self._opened_path = ""         # saved map opened for editing (empty = the live / cleaned map)
        self._clean_finished.connect(self._on_clean_finished)
        self._launcher = ManagedLaunch("Mapping", MAPPING_CMD, kill_patterns=MAPPING_KILL_PATTERNS)
        self._is_mapping = False
        self._build()

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self.error_header = ErrorHeader()
        root.addWidget(self.error_header)

        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        body.addWidget(self._build_map_area(), 1)
        panel = self._build_panel()
        make_groups_collapsible(panel, "mapping")
        self._panel = CollapsiblePanel(panel, 288, "right", "mapping.panel")
        body.addWidget(self._panel)
        root.addLayout(body, 1)

    def _build_map_area(self):
        container = QWidget()
        container.setMinimumWidth(400)

        self.map_widget = MapWidget()

        lay = QVBoxLayout(container)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.map_widget)

        self._joystick = JoystickWidget(self.map_widget)
        self._joystick.velocity_signal.connect(self.velocity_signal)
        self._joystick.show()
        self._joystick.raise_()

        return container

    def _build_panel(self):
        sa = QScrollArea()
        sa.setWidgetResizable(True)
        sa.setFixedWidth(288)
        sa.setObjectName("Panel")

        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(12, 14, 12, 14)
        lay.setSpacing(12)

        pose_box = QGroupBox(tr("pose_robot"))
        pl = QVBoxLayout(pose_box)
        self._x_lbl = _mono("X:    0.000 m")
        self._y_lbl = _mono("Y:    0.000 m")
        self._yaw_lbl = _mono("Yaw:  0.0 °")
        pl.addWidget(self._x_lbl)
        pl.addWidget(self._y_lbl)
        pl.addWidget(self._yaw_lbl)
        lay.addWidget(pose_box)

        self._vel_panel = VelocityInputPanel()
        self._vel_panel.max_velocity_changed.connect(self._joystick.set_max_velocity)
        lay.addWidget(self._vel_panel)

        ctrl_box = QGroupBox("MAPPING")
        cl = QVBoxLayout(ctrl_box)

        self._start_stop_btn = _btn(tr("mapping_start"), "BtnSuccess", h=36)
        self._start_stop_btn.clicked.connect(self.toggle_mapping)

        self._status_lbl = QLabel("—")
        self._status_lbl.setObjectName("StatusLabel")
        self._status_lbl.setWordWrap(True)

        cl.addWidget(self._start_stop_btn)
        cl.addWidget(self._status_lbl)
        lay.addWidget(ctrl_box)

        act_box = QGroupBox(tr("mapping_actions"))
        al = QVBoxLayout(act_box)

        self._save_btn = _btn(tr("mapping_save"), "BtnPrimary")
        self._save_btn.clicked.connect(self._save_map)

        self._reset_btn = _btn(tr("mapping_reset"))
        self._reset_btn.clicked.connect(self.map_widget._reset_view)

        self._clear_btn = _btn("🧹 Xoá map hiện tại", "BtnDanger")
        self._clear_btn.clicked.connect(self.clear_current_map)

        al.addWidget(self._save_btn)
        al.addWidget(self._reset_btn)
        al.addWidget(self._clear_btn)
        lay.addWidget(act_box)

        lay.addWidget(self._build_clean_box())
        lay.addWidget(self._build_edit_box())
        lay.addWidget(self._build_zone_box())
        lay.addWidget(self._build_view_box())

        lay.addStretch()
        sa.setWidget(inner)
        return sa

    # ── Map cleaning (HMI's own map) ────────────────────────────────

    def _spin(self, value, lo, hi, step, decimals=2):
        sp = QDoubleSpinBox()
        sp.setRange(lo, hi)
        sp.setSingleStep(step)
        sp.setDecimals(decimals)
        sp.setValue(value)
        return sp

    def _build_clean_box(self):
        d = CleanParams()
        self._mc_box = QGroupBox(tr("mc_title"))
        v = QVBoxLayout(self._mc_box)
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignLeft)
        self._mc_noise = self._spin(d.min_blob_m, 0.0, 2.0, 0.05)
        self._mc_minwall = self._spin(d.min_wall_m, 0.2, 5.0, 0.1)
        self._mc_gap = self._spin(d.max_gap_m, 0.0, 2.0, 0.05)
        self._mc_snap = self._spin(d.snap_deg, 0.0, 20.0, 1.0, 0)
        self._mc_thick = self._spin(d.wall_thickness_m * 100, 5.0, 40.0, 1.0, 0)
        self._mc_labels = {}
        for key, w in (("mc_noise", self._mc_noise), ("mc_minwall", self._mc_minwall),
                       ("mc_gap", self._mc_gap), ("mc_snap", self._mc_snap),
                       ("mc_thick", self._mc_thick)):
            lbl = QLabel(tr(key))
            self._mc_labels[key] = lbl
            form.addRow(lbl, w)
        v.addLayout(form)

        self._mc_run = _btn(tr("mc_run"), "BtnPrimary", h=34)
        self._mc_run.clicked.connect(self.run_clean)
        self._mc_raw = _btn(tr("mc_raw"), h=30, enabled=False)
        self._mc_raw.clicked.connect(self.show_raw_map)
        self._mc_save = _btn(tr("mc_save"), "BtnSuccess", h=34, enabled=False)
        self._mc_save.clicked.connect(self.save_clean_map)
        self._mc_status = QLabel("")
        self._mc_status.setWordWrap(True)
        self._mc_status.setObjectName("StatusLabel")
        for w in (self._mc_run, self._mc_raw, self._mc_save, self._mc_status):
            v.addWidget(w)
        return self._mc_box

    def _build_zone_box(self):
        self._zn_box = QGroupBox(tr("zn_title"))
        v = QVBoxLayout(self._zn_box)
        self._zn_keep = _btn(tr("zn_keepout"), h=30)
        self._zn_keep.clicked.connect(lambda: self.map_widget.begin_zone("keepout"))
        self._zn_slow = _btn(tr("zn_slow"), h=30)
        self._zn_slow.clicked.connect(lambda: self.map_widget.begin_zone("slow"))
        self._zn_undo = _btn(tr("zn_undo"), h=30)
        self._zn_undo.clicked.connect(self.map_widget.undo_zone)
        self._zn_hint = QLabel(tr("zn_hint")); self._zn_hint.setWordWrap(True)
        self._zn_hint.setStyleSheet("color:#8B949E;font-size:11px;")
        for w in (self._zn_keep, self._zn_slow, self._zn_undo, self._zn_hint):
            v.addWidget(w)
        return self._zn_box

    def _build_view_box(self):
        self._mv_box = QGroupBox("🗺")
        v = QVBoxLayout(self._mv_box)
        self._mv_follow = QCheckBox(tr("mv_follow"))
        self._mv_follow.toggled.connect(self.map_widget.set_follow)
        self._mv_grid = QCheckBox(tr("mv_grid"))
        self._mv_grid.setChecked(True)
        self._mv_grid.toggled.connect(self.map_widget.set_show_grid)
        self._mv_mini = QCheckBox(tr("mv_minimap"))
        self._mv_mini.setChecked(True)
        self._mv_mini.toggled.connect(self.map_widget.set_show_minimap)
        self._mv_measure = QPushButton(tr("mv_measure"))
        self._mv_measure.setCheckable(True)
        self._mv_measure.toggled.connect(self._on_measure_toggled)
        self.map_widget.set_measure_multi(True)            # several clicks = polyline with a running total
        self._mv_mclear = QPushButton(tr("mv_measure_clear"))
        self._mv_mclear.clicked.connect(self.map_widget.clear_measure)
        self._mv_mhint = QLabel(tr("mv_measure_hint")); self._mv_mhint.setWordWrap(True)
        self._mv_mhint.setStyleSheet("color:#8B949E;font-size:11px;")
        for w in (self._mv_follow, self._mv_grid, self._mv_mini, self._mv_measure, self._mv_mclear, self._mv_mhint):
            v.addWidget(w)
        return self._mv_box

    def _on_measure_toggled(self, on: bool):
        if on:
            self.map_widget.set_mode(MODE_MEASURE)
        else:                                  # back to what we were doing before measuring
            self.map_widget.set_mode(MODE_EDIT if self.map_widget.is_editing() else MODE_GOAL)

    # ── Manual map editing (4.9.0) ───────────────────────────────────

    def _build_edit_box(self):
        self._ed_box = QGroupBox(tr("ed_title"))
        v = QVBoxLayout(self._ed_box)
        self._ed_open = QPushButton(tr("ed_open"))
        self._ed_open.setFixedHeight(32)
        self._ed_open.clicked.connect(self.open_saved_map)
        v.addWidget(self._ed_open)
        self._ed_toggle = QPushButton(tr("ed_start"))
        self._ed_toggle.setCheckable(True)
        self._ed_toggle.setFixedHeight(34)
        self._ed_toggle.toggled.connect(self._on_edit_toggled)
        v.addWidget(self._ed_toggle)

        grid = QGridLayout()
        self._ed_group = QButtonGroup(self)
        self._ed_group.setExclusive(True)
        self._ed_tool_btns = {}
        for i, tool in enumerate(EDIT_TOOLS):
            b = QPushButton(tr(f"ed_t_{tool}"))
            b.setCheckable(True)
            b.setMinimumHeight(30)
            b.setChecked(tool == "wall")
            b.clicked.connect(lambda _c, t=tool: self.map_widget.set_edit_tool(t))
            self._ed_group.addButton(b)
            self._ed_tool_btns[tool] = b
            grid.addWidget(b, i // 2, i % 2)
        v.addLayout(grid)

        form = QFormLayout()
        self._ed_brush = QSpinBox()
        self._ed_brush.setRange(2, 100); self._ed_brush.setSingleStep(1); self._ed_brush.setValue(10)
        self._ed_brush.setSuffix(" cm"); self._ed_brush.setKeyboardTracking(False)
        self._ed_brush.valueChanged.connect(lambda cm: self.map_widget.set_edit_brush_m(cm / 100.0))
        self._ed_brush_lbl = QLabel(tr("ed_brush")); form.addRow(self._ed_brush_lbl, self._ed_brush)
        self._ed_cells = QSpinBox(); self._ed_cells.setRange(1, 6); self._ed_cells.setValue(1)
        self._ed_cells_lbl = QLabel(tr("ed_cells")); form.addRow(self._ed_cells_lbl, self._ed_cells)
        v.addLayout(form)

        self._ed_snap = QCheckBox(tr("ed_snap"))
        self._ed_snap.toggled.connect(self.map_widget.set_edit_snap)
        v.addWidget(self._ed_snap)

        row = QHBoxLayout()
        self._ed_undo, self._ed_redo = QPushButton(tr("ed_undo")), QPushButton(tr("ed_redo"))
        self._ed_undo.clicked.connect(self.map_widget.edit_undo); self._ed_redo.clicked.connect(self.map_widget.edit_redo)
        row.addWidget(self._ed_undo); row.addWidget(self._ed_redo)
        v.addLayout(row)

        row2 = QHBoxLayout()
        self._ed_thicken, self._ed_thin = QPushButton(tr("ed_thicken")), QPushButton(tr("ed_thin"))
        self._ed_thicken.clicked.connect(lambda: self._edit_batch(self.map_widget.edit_thicken, self._ed_cells.value()))
        self._ed_thin.clicked.connect(lambda: self._edit_batch(self.map_widget.edit_thin, self._ed_cells.value()))
        row2.addWidget(self._ed_thicken); row2.addWidget(self._ed_thin)
        v.addLayout(row2)
        self._ed_specks = QPushButton(tr("ed_specks"))
        self._ed_specks.clicked.connect(lambda: self._edit_batch(self.map_widget.edit_clear_specks, 4, "ed_specks_done"))
        v.addWidget(self._ed_specks)

        self._ed_save = QPushButton(tr("ed_save")); self._ed_save.setObjectName("BtnSuccess"); self._ed_save.setFixedHeight(34)
        self._ed_save.clicked.connect(self.save_edited_map)
        v.addWidget(self._ed_save)
        self._ed_status = QLabel(""); self._ed_status.setWordWrap(True); self._ed_status.setObjectName("StatusLabel")
        self._ed_hint = QLabel(tr("ed_hint")); self._ed_hint.setWordWrap(True)
        self._ed_hint.setStyleSheet("color:#8B949E;font-size:11px;")
        v.addWidget(self._ed_status); v.addWidget(self._ed_hint)

        self._ed_controls = (list(self._ed_tool_btns.values()) + [self._ed_brush, self._ed_cells, self._ed_snap,
                             self._ed_undo, self._ed_redo, self._ed_thicken, self._ed_thin, self._ed_specks])
        self._edited_once = False
        self._set_edit_enabled(False)
        self.map_widget.edit_state_changed.connect(self._on_edit_state)
        self.map_widget.edit_ended.connect(self._on_edit_ended)
        return self._ed_box

    def _set_edit_enabled(self, on: bool):
        for w in self._ed_controls:
            w.setEnabled(on)
        if not on:
            self._ed_undo.setEnabled(False); self._ed_redo.setEnabled(False)
        self._ed_save.setEnabled(on and self._edited_once)

    def _on_edit_toggled(self, on: bool):
        if on:
            if self.map_widget.get_grid() is None:
                self._ed_status.setText(tr("ed_no_map"))
                self._ed_toggle.blockSignals(True); self._ed_toggle.setChecked(False); self._ed_toggle.blockSignals(False)
                return
            # freeze the live SLAM map (otherwise the next /map message would overwrite the edits) and
            # remember it so "show raw map" can bring it back
            if not self._cleaned_view:
                self._raw_view = self.map_widget.get_grid()
                self._cleaned_view = True
                self._mc_raw.setEnabled(True)
            self._mv_measure.blockSignals(True); self._mv_measure.setChecked(False); self._mv_measure.blockSignals(False)
            if self.map_widget.begin_edit():
                self._edited_once = False
                self._set_edit_enabled(True)
                self.map_widget.set_edit_brush_m(self._ed_brush.value() / 100.0)
                self.map_widget.set_edit_snap(self._ed_snap.isChecked())
                for t, b in self._ed_tool_btns.items():
                    if b.isChecked():
                        self.map_widget.set_edit_tool(t)
                self._ed_toggle.setText(tr("ed_stop"))
                self._ed_status.setText(tr("ed_on"))
        else:
            self.map_widget.end_edit()
            self._ed_toggle.setText(tr("ed_start"))
            self._set_edit_enabled(False)
            self._ed_save.setEnabled(self._edited_once)       # the edited map can still be saved
            self._ed_status.setText(tr("ed_off") if self._edited_once else "")

    def _on_edit_state(self, can_undo: bool, can_redo: bool):
        self._ed_undo.setEnabled(can_undo); self._ed_redo.setEnabled(can_redo)
        if can_undo:
            self._edited_once = True
        self._ed_save.setEnabled(self._edited_once)

    def _on_edit_ended(self):
        """The grid was replaced from outside (raw map restored, map cleaned, map cleared): edits are gone."""
        self._ed_toggle.blockSignals(True); self._ed_toggle.setChecked(False); self._ed_toggle.blockSignals(False)
        self._ed_toggle.setText(tr("ed_start"))
        self._edited_once = False
        self._set_edit_enabled(False)
        self._ed_status.setText("")

    def _edit_batch(self, fn, arg, done_key: str = "ed_thick_done"):
        n = fn(arg)
        self._ed_status.setText(tr(done_key, n))

    def _export_zones(self, path_no_ext: str, shape, res, ox, oy) -> str:
        zones = self.map_widget.get_zones()
        zones_io.save_zones(path_no_ext, zones)
        if zones:
            mask = zones_io.export_keepout_mask(path_no_ext, zones, shape, res, ox, oy)
            if mask:
                return "\n" + tr("zn_exported", mask)
        return ""

    def open_saved_map(self):
        """Load a map saved earlier (.yaml + .pgm) and start editing it."""
        if not self._confirm_discard_edits():
            return
        path, _ = QFileDialog.getOpenFileName(
            self, tr("ed_open"), os.path.expanduser("~/maps"), "Map (*.yaml)")
        if not path:
            return
        if self._launcher.is_running():
            ans = QMessageBox.question(self, tr("ed_open"), tr("ed_open_mapping"),
                                       QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                       QMessageBox.StandardButton.No)
            if ans != QMessageBox.StandardButton.Yes:
                return
            self.stop_mapping()
        self.map_widget.load_map_file(path)          # grid + saved vector walls + zones
        if self.map_widget.get_grid() is None:
            self._ed_status.setText(tr("ed_open_fail", path))
            return
        self._opened_path = path
        self._raw_view = None                        # nothing to go "back" to except the live map
        self._clean_result = None
        self._cleaned_view = True                    # freeze: live SLAM must not replace the opened map
        self._mc_raw.setEnabled(False); self._mc_save.setEnabled(False)
        self._ed_toggle.setChecked(False)
        self._ed_toggle.setChecked(True)             # -> _on_edit_toggled(True): starts editing this map
        self._ed_status.setText(tr("ed_opened", os.path.basename(path)))

    def save_edited_map(self):
        """Save the grid exactly as shown (what the user drew), as a normal Nav2 map (.pgm + .yaml)."""
        src = self.map_widget.get_grid()
        if src is None:
            return
        grid, res, ox, oy = src
        path, _ = QFileDialog.getSaveFileName(
            self, tr("ed_save"),
            (os.path.splitext(self._opened_path)[0] + "_edit") if self._opened_path
            else os.path.expanduser("~/maps/map_edit"), "Map (*.yaml)")
        if not path:
            return
        if path.endswith(".yaml"):
            path = path[:-5]
        try:
            yaml_path = map_io.save_map(path, grid, res, ox, oy)       # no vector walls: the grid is the truth
        except Exception as exc:
            QMessageBox.warning(self, tr("ed_save"), str(exc))
            return
        self._ed_status.setText("✅ " + tr("ed_saved", yaml_path) + self._export_zones(path, grid.shape, res, ox, oy))

    def _current_params(self) -> CleanParams:
        return CleanParams(
            min_blob_m=self._mc_noise.value(),
            min_wall_m=self._mc_minwall.value(),
            max_gap_m=self._mc_gap.value(),
            snap_deg=self._mc_snap.value(),
            wall_thickness_m=self._mc_thick.value() / 100.0,
        )

    def _confirm_discard_edits(self) -> bool:
        """Hand edits live only in the picture: replacing the grid would lose them."""
        if not getattr(self, "_edited_once", False):
            return True
        ans = QMessageBox.question(self, tr("ed_title"), tr("ed_discard"),
                                   QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                   QMessageBox.StandardButton.No)
        return ans == QMessageBox.StandardButton.Yes

    def run_clean(self):
        if not self._confirm_discard_edits():
            return
        src = self._raw_view if self._cleaned_view else self.map_widget.get_grid()
        if src is None:
            self._mc_status.setText(tr("mc_empty"))
            return
        self._raw_view = src
        grid, res, ox, oy = src
        params = self._current_params()
        self._mc_run.setEnabled(False)
        self._mc_status.setText(tr("mc_working"))

        def work():
            try:
                r = clean_map(grid, res, params)
                self._clean_finished.emit((r, params, res, ox, oy), None)
            except Exception as exc:   # never kill the UI because of a bad map
                self._clean_finished.emit(None, str(exc))

        threading.Thread(target=work, daemon=True).start()

    def _on_clean_finished(self, payload, error):
        self._mc_run.setEnabled(True)
        if payload is None:
            self._mc_status.setText("❌ " + str(error))
            return
        r, params, res, ox, oy = payload
        self._clean_result = payload
        self._cleaned_view = True
        self.map_widget.set_grid(r.grid, res, ox, oy, segments=r.segments,
                                 wall_thickness_m=params.wall_thickness_m)
        self._mc_raw.setEnabled(True)
        self._mc_save.setEnabled(True)
        self._mc_status.setText(
            tr("mc_done", r.stats.get("lines", 0), r.stats.get("noise_removed_cells", 0))
            + "\n" + tr("mc_hint_frozen"))

    def show_raw_map(self):
        if self._raw_view is None or not self._confirm_discard_edits():
            return
        grid, res, ox, oy = self._raw_view
        self.map_widget.set_grid(grid, res, ox, oy)
        self._cleaned_view = False
        self._raw_view = None
        self._mc_raw.setEnabled(False)
        self._mc_status.setText("")

    def save_clean_map(self):
        if not self._clean_result:
            return
        r, params, res, ox, oy = self._clean_result
        path, _ = QFileDialog.getSaveFileName(
            self, tr("mc_save"), os.path.expanduser("~/maps/map_clean"), "Map (*.yaml)")
        if not path:
            return
        if path.endswith(".yaml"):
            path = path[:-5]
        try:
            yaml_path = map_io.save_map(path, r.grid, res, ox, oy, segments=r.segments,
                                        wall_thickness_m=params.wall_thickness_m)
        except Exception as exc:
            QMessageBox.warning(self, tr("mc_save"), str(exc))
            return
        zones = self.map_widget.get_zones()
        zones_io.save_zones(path, zones)
        extra = ""
        if zones:
            mask = zones_io.export_keepout_mask(path, zones, r.grid.shape, res, ox, oy)
            if mask:
                extra = "\n" + tr("zn_exported", mask)
        self._mc_status.setText("✅ " + tr("mc_saved", yaml_path) + extra)

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._reposition_joystick()

    def showEvent(self, ev):
        super().showEvent(ev)
        self._reposition_joystick()

    def _reposition_joystick(self):
        if hasattr(self, "_joystick"):
            ph = self.map_widget.height()
            self._joystick.move(16, max(16, ph - self._joystick.height() - 16))
            self._joystick.raise_()
            self._joystick.show()

    def update_pose(self, x: float, y: float, yaw: float):
        self._x_lbl.setText(f"X:    {x:.3f} m")
        self._y_lbl.setText(f"Y:    {y:.3f} m")
        self._yaw_lbl.setText(f"Yaw:  {math.degrees(yaw):.1f} °")

    def update_pose_on_map(self, x: float, y: float, yaw: float):
        self.map_widget.update_pose(x, y, yaw)

    def update_map(self, msg):
        if self._cleaned_view:
            return          # keep showing the cleaned map until "raw map" is pressed
        self.map_widget.update_map(msg, reset_view=False)

    def update_scan(self, world_pts):
        self.map_widget.update_scan(world_pts)

    def toggle_mapping(self):
        if self._launcher.is_running():
            self.stop_mapping()
        else:
            self.start_mapping()

    def start_mapping(self):
        if self._opened_path:                        # an opened old map is frozen on screen: leaving it = new map
            if not self._confirm_discard_edits():
                return
            self._opened_path = ""
            self._cleaned_view = False
            self.map_widget.end_edit()
            self._on_edit_ended()
        ok, msg = self._launcher.start()
        self._status_lbl.setText(("🟢 " if ok else "❌ ") + msg)
        self._is_mapping = ok

        if ok:
            self._start_stop_btn.setText(tr("mapping_stop"))
            self._start_stop_btn.setObjectName("BtnDanger")
            self.mapping_started.emit()
        else:
            self._start_stop_btn.setText(tr("mapping_start"))
            self._start_stop_btn.setObjectName("BtnSuccess")

        self._refresh_btn_style()

    def stop_mapping(self):
        ok, msg = self._launcher.stop()
        self._status_lbl.setText(("⬛ " if ok else "❌ ") + msg)
        self._is_mapping = False
        self._start_stop_btn.setText(tr("mapping_start"))
        self._start_stop_btn.setObjectName("BtnSuccess")
        self._refresh_btn_style()
        self.velocity_signal.emit(0.0, 0.0)
        self.mapping_stopped.emit()

    def cleanup(self):
        self.stop_mapping()

    def clear_current_map(self):
        if not self._confirm_discard_edits():
            return
        self._cleaned_view = False
        self._opened_path = ""
        self._raw_view = None
        self._clean_result = None
        self._mc_raw.setEnabled(False)
        self._mc_save.setEnabled(False)
        self.map_widget.clear_map()
        self._status_lbl.setText("🧹 Đã xoá map hiển thị. Có thể mapping lại.")

    def _save_map(self):
        path, _ = QFileDialog.getSaveFileName(
            self,
            tr("mapping_save"),
            os.path.expanduser("~/maps/map"),
            "Map (*.yaml)",
        )
        if not path:
            return
        if path.endswith(".yaml"):
            path = path[:-5]
        self.save_map_requested.emit(path)

    def _refresh_btn_style(self):
        self._start_stop_btn.style().unpolish(self._start_stop_btn)
        self._start_stop_btn.style().polish(self._start_stop_btn)

    def retranslate(self):
        self._mc_box.setTitle(tr("mc_title"))
        for k, lbl in self._mc_labels.items():
            lbl.setText(tr(k))
        self._mc_run.setText(tr("mc_run"))
        self._mc_raw.setText(tr("mc_raw"))
        self._mc_save.setText(tr("mc_save"))
        self._zn_box.setTitle(tr("zn_title"))
        self._zn_keep.setText(tr("zn_keepout")); self._zn_slow.setText(tr("zn_slow"))
        self._zn_undo.setText(tr("zn_undo")); self._zn_hint.setText(tr("zn_hint"))
        self._mv_follow.setText(tr("mv_follow"))
        self._mv_grid.setText(tr("mv_grid"))
        self._mv_mini.setText(tr("mv_minimap"))
        self._mv_measure.setText(tr("mv_measure"))
        self._mv_mclear.setText(tr("mv_measure_clear")); self._mv_mhint.setText(tr("mv_measure_hint"))
        self._ed_box.setTitle(tr("ed_title")); self._ed_open.setText(tr("ed_open"))
        self._ed_toggle.setText(tr("ed_stop") if self.map_widget.is_editing() else tr("ed_start"))
        for t, b in self._ed_tool_btns.items():
            b.setText(tr(f"ed_t_{t}"))
        self._ed_brush_lbl.setText(tr("ed_brush")); self._ed_cells_lbl.setText(tr("ed_cells"))
        self._ed_snap.setText(tr("ed_snap")); self._ed_undo.setText(tr("ed_undo")); self._ed_redo.setText(tr("ed_redo"))
        self._ed_thicken.setText(tr("ed_thicken")); self._ed_thin.setText(tr("ed_thin"))
        self._ed_specks.setText(tr("ed_specks")); self._ed_save.setText(tr("ed_save")); self._ed_hint.setText(tr("ed_hint"))
        self.error_header.retranslate()
        self._vel_panel.retranslate()
        self._start_stop_btn.setText(tr("mapping_stop") if self._launcher.is_running() else tr("mapping_start"))
        self._save_btn.setText(tr("mapping_save"))
        self._reset_btn.setText(tr("mapping_reset"))
        self._clear_btn.setText("🧹 Xoá map hiện tại")