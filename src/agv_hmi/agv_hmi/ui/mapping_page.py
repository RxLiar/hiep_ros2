import os
import math

import threading

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QGroupBox, QLabel,
    QPushButton, QScrollArea, QFileDialog, QDoubleSpinBox, QCheckBox,
    QFormLayout, QMessageBox
)
from PyQt6.QtCore import Qt, pyqtSignal

from agv_hmi.ui.map_widget import MapWidget
from agv_hmi.ui.error_header import ErrorHeader
from agv_hmi.ui.joystick_widget import JoystickWidget
from agv_hmi.ui.velocity_input import VelocityInputPanel
from agv_hmi.ui.i18n import tr
from agv_hmi.ui.process_manager import ManagedLaunch
from agv_hmi.core.map_cleaner import clean_map, CleanParams
from agv_hmi.core import map_io
from agv_hmi.ui.map_widget import MODE_GOAL, MODE_MEASURE
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

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self._build_map_area())
        splitter.addWidget(self._build_panel())
        splitter.setStretchFactor(0, 1)
        splitter.setSizes([1000, 260])
        root.addWidget(splitter)

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
        sa.setFixedWidth(270)
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
        self._mv_measure.toggled.connect(
            lambda on: self.map_widget.set_mode(MODE_MEASURE if on else MODE_GOAL))
        for w in (self._mv_follow, self._mv_grid, self._mv_mini, self._mv_measure):
            v.addWidget(w)
        return self._mv_box

    def _current_params(self) -> CleanParams:
        return CleanParams(
            min_blob_m=self._mc_noise.value(),
            min_wall_m=self._mc_minwall.value(),
            max_gap_m=self._mc_gap.value(),
            snap_deg=self._mc_snap.value(),
            wall_thickness_m=self._mc_thick.value() / 100.0,
        )

    def run_clean(self):
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
        if self._raw_view is None:
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
        self._cleaned_view = False
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
        self.error_header.retranslate()
        self._vel_panel.retranslate()
        self._start_stop_btn.setText(tr("mapping_stop") if self._launcher.is_running() else tr("mapping_start"))
        self._save_btn.setText(tr("mapping_save"))
        self._reset_btn.setText(tr("mapping_reset"))
        self._clear_btn.setText("🧹 Xoá map hiện tại")