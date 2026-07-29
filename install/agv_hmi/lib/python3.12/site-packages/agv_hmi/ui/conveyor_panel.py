from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QPushButton, QSlider, QDoubleSpinBox,
    QScrollArea
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
from agv_hmi.ui.i18n import tr


class ConveyorCard(QGroupBox):
    cmd_requested = pyqtSignal(dict)

    def __init__(self, conv_id: int):
        super().__init__()
        self._id = conv_id
        self._timer = None
        self._build()
        self.setTitle(tr("conv_belt", self._id))

    def _build(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(12, 14, 12, 12)

        row = QHBoxLayout()
        self._status_dot = QLabel("●")
        self._status_dot.setStyleSheet("color:#484F58;font-size:10px;")
        self._status_lbl = QLabel(tr("conv_status_idle"))
        self._status_lbl.setStyleSheet("color:#8B949E;font-size:11px;")
        row.addWidget(self._status_dot)
        row.addWidget(self._status_lbl)
        row.addStretch()
        layout.addLayout(row)

        sp_row = QVBoxLayout()
        sh = QHBoxLayout()
        sh.addWidget(QLabel(tr("conv_speed")))
        self._speed_val = QLabel("75 %")
        self._speed_val.setStyleSheet("color:#58A6FF;font-family:monospace;")
        sh.addStretch()
        sh.addWidget(self._speed_val)

        self._speed_sl = QSlider(Qt.Orientation.Horizontal)
        self._speed_sl.setRange(0, 100)
        self._speed_sl.setValue(75)
        self._speed_sl.valueChanged.connect(lambda v: self._speed_val.setText(f"{v} %"))

        sp_row.addLayout(sh)
        sp_row.addWidget(self._speed_sl)
        layout.addLayout(sp_row)

        dur = QHBoxLayout()
        dur.addWidget(QLabel(tr("conv_duration")))
        self._duration_spin = QDoubleSpinBox()
        self._duration_spin.setRange(0, 999)
        self._duration_spin.setValue(5.0)
        self._duration_spin.setSingleStep(0.5)
        self._duration_spin.setDecimals(1)
        self._duration_spin.setSpecialValueText("∞")
        self._duration_spin.setFixedWidth(90)
        dur.addStretch()
        dur.addWidget(self._duration_spin)
        layout.addLayout(dur)

        btns = QHBoxLayout()
        self._recv_btn = QPushButton(tr("conv_receive"))
        self._recv_btn.setObjectName("BtnSuccess")
        self._recv_btn.clicked.connect(lambda: self._send_cmd("receive"))

        self._send_btn = QPushButton(tr("conv_send"))
        self._send_btn.setObjectName("BtnPrimary")
        self._send_btn.clicked.connect(lambda: self._send_cmd("send"))

        self._stop_btn = QPushButton(tr("conv_stop"))
        self._stop_btn.setObjectName("BtnDanger")
        self._stop_btn.clicked.connect(self._send_stop)

        for b in (self._recv_btn, self._send_btn, self._stop_btn):
            b.setFixedHeight(30)
            btns.addWidget(b)
        layout.addLayout(btns)

    def _send_cmd(self, mode: str):
        dur = self._duration_spin.value()
        payload = {
            "conveyor_id": self._id,
            "mode": mode,
            "speed": self._speed_sl.value(),
            "duration": dur,
        }
        self.cmd_requested.emit(payload)
        self._set_running(True, mode)

        if dur > 0:
            if self._timer:
                self._timer.stop()
            self._timer = QTimer()
            self._timer.setSingleShot(True)
            self._timer.timeout.connect(lambda: self._set_running(False, "stop"))
            self._timer.start(int(dur * 1000))

    def _send_stop(self):
        if self._timer:
            self._timer.stop()
            self._timer = None
        self.cmd_requested.emit({
            "conveyor_id": self._id,
            "mode": "stop",
            "speed": 0,
            "duration": 0,
        })
        self._set_running(False, "stop")

    def _set_running(self, running: bool, mode: str):
        if running:
            color = "#3FB950" if mode == "receive" else "#58A6FF"
            label = tr("conv_receive") if mode == "receive" else tr("conv_send")
            self._status_dot.setStyleSheet(f"color:{color};font-size:10px;")
            self._status_lbl.setText(f"{tr('conv_status_run')} — {label}")
        else:
            self._status_dot.setStyleSheet("color:#484F58;font-size:10px;")
            self._status_lbl.setText(tr("conv_status_idle"))

    def force_stop(self):
        self._send_stop()

    def retranslate(self):
        self.setTitle(tr("conv_belt", self._id))
        self._recv_btn.setText(tr("conv_receive"))
        self._send_btn.setText(tr("conv_send"))
        self._stop_btn.setText(tr("conv_stop"))


class SensorCell(QGroupBox):
    def __init__(self, sensor_id: int):
        super().__init__(tr("sensor_n", sensor_id))
        self._id = sensor_id
        self._state = False
        self._build()
        self.set_state(False)

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 14, 10, 10)
        self._dot = QLabel("●")
        self._dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._dot.setStyleSheet("font-size:20px;color:#F85149;background:transparent;")
        self._text = QLabel("OFF")
        self._text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._text.setStyleSheet("font-size:12px;font-weight:700;color:#F85149;background:transparent;")
        lay.addWidget(self._dot)
        lay.addWidget(self._text)

    def set_state(self, on: bool):
        self._state = on
        if on:
            self._dot.setStyleSheet("font-size:20px;color:#3FB950;background:transparent;")
            self._text.setText("ON")
            self._text.setStyleSheet("font-size:12px;font-weight:700;color:#3FB950;background:transparent;")
        else:
            self._dot.setStyleSheet("font-size:20px;color:#F85149;background:transparent;")
            self._text.setText("OFF")
            self._text.setStyleSheet("font-size:12px;font-weight:700;color:#F85149;background:transparent;")

    def retranslate(self):
        self.setTitle(tr("sensor_n", self._id))


class ConveyorPage(QWidget):
    conveyor_cmd = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._cards = []
        self._sensors = []
        self._build()

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(16)

        hdr = QHBoxLayout()
        self._title = QLabel(tr("conv_title"))
        self._title.setStyleSheet("font-size:17px;font-weight:700;color:#E6EDF3;")
        hdr.addWidget(self._title)
        hdr.addStretch()

        self._stop_all_btn = QPushButton(tr("conv_all_stop"))
        self._stop_all_btn.setObjectName("BtnDanger")
        self._stop_all_btn.setFixedHeight(34)
        self._stop_all_btn.clicked.connect(self.stop_all)
        hdr.addWidget(self._stop_all_btn)
        root.addLayout(hdr)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setObjectName("Panel")
        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setSpacing(18)

        conv_grid = QGridLayout()
        conv_grid.setSpacing(16)
        for i in range(4):
            card = ConveyorCard(i + 1)
            card.cmd_requested.connect(self.conveyor_cmd)
            self._cards.append(card)
            conv_grid.addWidget(card, i // 2, i % 2)
        lay.addLayout(conv_grid)

        sensor_box = QGroupBox(tr("sensor_title"))
        sg = QGridLayout(sensor_box)
        sg.setSpacing(10)
        for i in range(12):
            cell = SensorCell(i + 1)
            self._sensors.append(cell)
            sg.addWidget(cell, i // 4, i % 4)
        lay.addWidget(sensor_box)

        lay.addStretch()
        scroll.setWidget(inner)
        root.addWidget(scroll)

    def update_sensor(self, sensor_id: int, on: bool):
        # chấp nhận cả 0-index và 1-index
        idx = sensor_id - 1 if 1 <= sensor_id <= 12 else sensor_id
        if 0 <= idx < len(self._sensors):
            self._sensors[idx].set_state(on)

    def stop_all(self):
        for card in self._cards:
            card.force_stop()

    def retranslate(self):
        self._title.setText(tr("conv_title"))
        self._stop_all_btn.setText(tr("conv_all_stop"))
        for c in self._cards:
            c.retranslate()
        for s in self._sensors:
            s.retranslate()
