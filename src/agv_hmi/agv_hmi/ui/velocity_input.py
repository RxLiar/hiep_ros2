"""
velocity_input.py — Panel nhập vận tốc tối đa (ô số, không slider)
- Gửi qua ros2 param set (subprocess) thay vì /cmd_vel trực tiếp
- Emit signal để joystick biết giới hạn tối đa
"""
import subprocess
from PyQt6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout,
    QLabel, QDoubleSpinBox, QPushButton
)
from PyQt6.QtCore import pyqtSignal
from agv_hmi.ui.i18n import tr


class VelocityInputPanel(QGroupBox):
    """
    Nhập vận tốc tối đa → emit max_velocity_changed(linear, angular)
    để joystick và Nav2 biết giới hạn.
    Nút "Áp dụng" gọi ros2 param set (non-blocking subprocess).
    """
    max_velocity_changed = pyqtSignal(float, float)  # linear_max, angular_max

    def __init__(self):
        super().__init__()
        self._lin_val = 0.5
        self._ang_val = 0.8
        self._build()

    def _build(self):
        self.setTitle(tr("vel_title"))
        lay = QVBoxLayout(self)
        lay.setSpacing(10)

        # Info
        info = QLabel(tr("vel_info"))
        info.setWordWrap(True)
        info.setStyleSheet("color:#8B949E;font-size:11px;")
        lay.addWidget(info)

        # Linear
        lin_row = QHBoxLayout()
        self._lin_lbl = QLabel(tr("vel_linear"))
        self._lin_lbl.setStyleSheet("color:#E6EDF3;font-size:12px;")
        self._lin_spin = QDoubleSpinBox()
        self._lin_spin.setRange(0.05, 3.0)
        self._lin_spin.setValue(0.5)
        self._lin_spin.setSingleStep(0.05)
        self._lin_spin.setDecimals(2)
        self._lin_spin.setSuffix(" m/s")
        self._lin_spin.setFixedWidth(100)
        lin_row.addWidget(self._lin_lbl)
        lin_row.addStretch()
        lin_row.addWidget(self._lin_spin)
        lay.addLayout(lin_row)

        # Angular
        ang_row = QHBoxLayout()
        self._ang_lbl = QLabel(tr("vel_angular"))
        self._ang_lbl.setStyleSheet("color:#E6EDF3;font-size:12px;")
        self._ang_spin = QDoubleSpinBox()
        self._ang_spin.setRange(0.05, 3.14)
        self._ang_spin.setValue(0.8)
        self._ang_spin.setSingleStep(0.05)
        self._ang_spin.setDecimals(2)
        self._ang_spin.setSuffix(" rad/s")
        self._ang_spin.setFixedWidth(100)
        ang_row.addWidget(self._ang_lbl)
        ang_row.addStretch()
        ang_row.addWidget(self._ang_spin)
        lay.addLayout(ang_row)

        # Apply button
        self._apply_btn = QPushButton(tr("vel_apply"))
        self._apply_btn.setObjectName("BtnPrimary")
        self._apply_btn.setFixedHeight(30)
        self._apply_btn.clicked.connect(self._apply)
        lay.addWidget(self._apply_btn)

        # Status
        self._status_lbl = QLabel("")
        self._status_lbl.setStyleSheet("color:#8B949E;font-size:10px;")
        lay.addWidget(self._status_lbl)

    def _apply(self):
        self._lin_val = self._lin_spin.value()
        self._ang_val = self._ang_spin.value()
        self.max_velocity_changed.emit(self._lin_val, self._ang_val)
        self._send_param()

    def _send_param(self):
        """
        Gửi ros2 param set non-blocking.
        Node name và param name sẽ được xác nhận sau khi tích hợp Nav2.
        Hiện tại target controller_server của Nav2.
        """
        cmds = [
            ["ros2", "param", "set",
             "/controller_server",
             "FollowPath.max_vel_x",
             str(self._lin_val)],
            ["ros2", "param", "set",
             "/controller_server",
             "FollowPath.max_vel_theta",
             str(self._ang_val)],
        ]
        try:
            for cmd in cmds:
                subprocess.Popen(
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            self._status_lbl.setText(
                f"✅ Đã áp dụng: {self._lin_val:.2f} m/s / {self._ang_val:.2f} rad/s")
            self._status_lbl.setStyleSheet("color:#3FB950;font-size:10px;")
        except Exception as e:
            self._status_lbl.setText(f"❌ {e}")
            self._status_lbl.setStyleSheet("color:#F85149;font-size:10px;")

    def get_values(self):
        return self._lin_val, self._ang_val

    def retranslate(self):
        self.setTitle(tr("vel_title"))
        self._lin_lbl.setText(tr("vel_linear"))
        self._ang_lbl.setText(tr("vel_angular"))
        self._apply_btn.setText(tr("vel_apply"))