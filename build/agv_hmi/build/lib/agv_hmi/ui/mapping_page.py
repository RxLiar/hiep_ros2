import os
import math

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QSplitter, QGroupBox, QLabel,
    QPushButton, QScrollArea, QFileDialog
)
from PyQt6.QtCore import Qt, pyqtSignal

from agv_hmi.ui.map_widget import MapWidget
from agv_hmi.ui.error_header import ErrorHeader
from agv_hmi.ui.joystick_widget import JoystickWidget
from agv_hmi.ui.velocity_input import VelocityInputPanel
from agv_hmi.ui.i18n import tr
from agv_hmi.ui.process_manager import ManagedLaunch


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

    def __init__(self):
        super().__init__()
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

        lay.addStretch()
        sa.setWidget(inner)
        return sa

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
        self.error_header.retranslate()
        self._vel_panel.retranslate()
        self._start_stop_btn.setText(tr("mapping_stop") if self._launcher.is_running() else tr("mapping_start"))
        self._save_btn.setText(tr("mapping_save"))
        self._reset_btn.setText(tr("mapping_reset"))
        self._clear_btn.setText("🧹 Xoá map hiện tại")