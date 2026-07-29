"""
task_editor.py — Editor nhiều task cho 1 waypoint

Mỗi task là 1 dict:
{
  "type": "conveyor" | "wait" | "confirm" | "io" | "buzzer",
  "order": "parallel" | "sequential",
  # conveyor only:
  "conveyor_ids": [1, 2],          # multi-select 1-4
  "conveyor_mode": "receive"|"send",
  "speed": 75,
  "duration": 5.0,
  # wait only:
  "delay": 3,
}

UI: list tasks ở trên, form edit ở dưới.
"""
import copy
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QListWidget, QListWidgetItem,
    QComboBox, QDoubleSpinBox, QSpinBox, QCheckBox,
    QFrame, QButtonGroup
)
from PyQt6.QtCore import Qt, pyqtSignal
from agv_hmi.ui.i18n import tr


def _btn(t, obj="", h=28, enabled=True):
    b = QPushButton(t)
    if obj: b.setObjectName(obj)
    b.setFixedHeight(h); b.setEnabled(enabled)
    return b


TASK_TYPES = ["conveyor", "wait", "confirm", "io", "buzzer"]

def _task_label(task: dict) -> str:
    t = task.get("type", "")
    order = "‖" if task.get("order") == "parallel" else "→"
    if t == "conveyor":
        ids   = task.get("conveyor_ids", [])
        mode  = task.get("conveyor_mode", "receive")
        mode_str = tr("conv_receive") if mode == "receive" else tr("conv_send")
        return f"{order} 🏭 BT{ids} {mode_str} {task.get('speed',75)}% {task.get('duration',5)}s"
    elif t == "wait":
        return f"{order} ⏱ Dừng {task.get('delay', 3)}s"
    elif t == "confirm":
        return f"{order} ✋ Chờ xác nhận"
    elif t == "io":
        return f"{order} ⚡ Tín hiệu IO"
    elif t == "buzzer":
        return f"{order} 🔔 Phát còi"
    return f"{order} {t}"


class TaskEditor(QWidget):
    """
    Widget nhúng vào NavigationPage panel phải.
    tasks_changed emit mỗi khi list task thay đổi.
    """
    tasks_changed = pyqtSignal(list)   # list[dict]

    def __init__(self):
        super().__init__()
        self._tasks: list[dict] = []
        self._editing_idx: int = -1
        self._build()

    # ── Build ────────────────────────────────────────────────────────

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(6)

        # ── Task list ────────────────────────────────────────────
        list_row = QHBoxLayout()
        self._task_list = QListWidget()
        self._task_list.setFixedHeight(100)
        self._task_list.currentRowChanged.connect(self._on_task_select)

        btn_col = QVBoxLayout()
        btn_col.setSpacing(4)
        self._add_btn = _btn(tr("task_add"), h=26)
        self._del_btn = _btn(tr("task_del"), "BtnDanger", h=26, enabled=False)
        self._up_btn  = _btn("▲", h=26, enabled=False)
        self._dn_btn  = _btn("▼", h=26, enabled=False)
        self._add_btn.clicked.connect(self._add_task)
        self._del_btn.clicked.connect(self._del_task)
        self._up_btn.clicked.connect(lambda: self._move_task(-1))
        self._dn_btn.clicked.connect(lambda: self._move_task(1))
        for b in (self._add_btn, self._del_btn, self._up_btn, self._dn_btn):
            btn_col.addWidget(b)
        btn_col.addStretch()

        list_row.addWidget(self._task_list)
        list_row.addLayout(btn_col)
        root.addLayout(list_row)

        # ── Task form ─────────────────────────────────────────────
        self._form = QGroupBox("TASK")
        fl = QVBoxLayout(self._form)
        fl.setSpacing(8)

        # Type selector
        type_row = QHBoxLayout()
        type_row.addWidget(QLabel("Loại:"))
        self._type_combo = QComboBox()
        self._type_combo.addItems([
            tr("task_conveyor"), tr("task_wait"),
            tr("task_confirm"), tr("task_io"), tr("task_buzzer")
        ])
        self._type_combo.currentIndexChanged.connect(self._on_type_change)
        type_row.addWidget(self._type_combo)
        fl.addLayout(type_row)

        # Order selector
        order_row = QHBoxLayout()
        order_row.addWidget(QLabel(tr("task_order")))
        self._order_combo = QComboBox()
        self._order_combo.addItems([tr("task_sequential"), tr("task_parallel")])
        order_row.addWidget(self._order_combo)
        fl.addLayout(order_row)

        # ── Conveyor options ─────────────────────────────────────
        self._conv_frame = QFrame()
        cfl = QVBoxLayout(self._conv_frame)
        cfl.setContentsMargins(0, 0, 0, 0)
        cfl.setSpacing(6)

        # Multi-select băng tải
        bt_row = QHBoxLayout()
        bt_row.addWidget(QLabel("Băng tải:"))
        self._conv_checks: list[QCheckBox] = []
        for i in range(1, 5):
            cb = QCheckBox(str(i))
            cb.setStyleSheet("QCheckBox{color:#E6EDF3;font-size:12px;}")
            bt_row.addWidget(cb)
            self._conv_checks.append(cb)
        bt_row.addStretch()
        cfl.addLayout(bt_row)

        # Mode
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        self._conv_mode = QComboBox()
        self._conv_mode.addItems([tr("conv_receive"), tr("conv_send")])
        mode_row.addWidget(self._conv_mode)
        cfl.addLayout(mode_row)

        # Speed
        spd_row = QHBoxLayout()
        spd_row.addWidget(QLabel(tr("conv_speed")))
        self._conv_speed = QSpinBox()
        self._conv_speed.setRange(0, 100)
        self._conv_speed.setValue(75)
        self._conv_speed.setSuffix(" %")
        self._conv_speed.setFixedWidth(80)
        spd_row.addStretch()
        spd_row.addWidget(self._conv_speed)
        cfl.addLayout(spd_row)

        # Duration
        dur_row = QHBoxLayout()
        dur_row.addWidget(QLabel(tr("conv_duration")))
        self._conv_dur = QDoubleSpinBox()
        self._conv_dur.setRange(0, 999)
        self._conv_dur.setValue(5.0)
        self._conv_dur.setSingleStep(0.5)
        self._conv_dur.setSuffix(" s")
        self._conv_dur.setSpecialValueText("∞")
        self._conv_dur.setFixedWidth(80)
        dur_row.addStretch()
        dur_row.addWidget(self._conv_dur)
        cfl.addLayout(dur_row)
        fl.addWidget(self._conv_frame)

        # ── Wait options ──────────────────────────────────────────
        self._wait_frame = QFrame()
        wfl = QHBoxLayout(self._wait_frame)
        wfl.setContentsMargins(0, 0, 0, 0)
        wfl.addWidget(QLabel("Dừng (giây):"))
        self._wait_spin = QSpinBox()
        self._wait_spin.setRange(1, 999)
        self._wait_spin.setValue(3)
        self._wait_spin.setSuffix(" s")
        self._wait_spin.setFixedWidth(80)
        wfl.addStretch()
        wfl.addWidget(self._wait_spin)
        fl.addWidget(self._wait_frame)

        # Apply button
        self._apply_btn = _btn("✔ Cập nhật task", "BtnSuccess", h=30)
        self._apply_btn.clicked.connect(self._apply_form)
        fl.addWidget(self._apply_btn)

        root.addWidget(self._form)

        # Init
        self._on_type_change(0)
        self._form.setVisible(False)

    # ── Logic ────────────────────────────────────────────────────────

    def _refresh_list(self):
        self._task_list.clear()
        for t in self._tasks:
            self._task_list.addItem(f"  {_task_label(t)}")
        self.tasks_changed.emit(copy.deepcopy(self._tasks))

    def _add_task(self):
        task = {
            "type": "conveyor",
            "order": "sequential",
            "conveyor_ids": [1],
            "conveyor_mode": "receive",
            "speed": 75,
            "duration": 5.0,
            "delay": 3,
        }
        self._tasks.append(task)
        self._refresh_list()
        self._task_list.setCurrentRow(len(self._tasks) - 1)
        self._form.setVisible(True)

    def _del_task(self):
        row = self._task_list.currentRow()
        if 0 <= row < len(self._tasks):
            self._tasks.pop(row)
            self._editing_idx = -1
            self._refresh_list()
            self._form.setVisible(bool(self._tasks))
            self._del_btn.setEnabled(False)
            self._up_btn.setEnabled(False)
            self._dn_btn.setEnabled(False)

    def _move_task(self, direction: int):
        row = self._task_list.currentRow()
        new_row = row + direction
        if 0 <= new_row < len(self._tasks):
            self._tasks[row], self._tasks[new_row] = \
                self._tasks[new_row], self._tasks[row]
            self._refresh_list()
            self._task_list.setCurrentRow(new_row)

    def _on_task_select(self, row: int):
        ok = 0 <= row < len(self._tasks)
        self._del_btn.setEnabled(ok)
        self._up_btn.setEnabled(ok and row > 0)
        self._dn_btn.setEnabled(ok and row < len(self._tasks) - 1)
        if ok:
            self._editing_idx = row
            self._load_form(self._tasks[row])
            self._form.setVisible(True)

    def _on_type_change(self, idx: int):
        types = ["conveyor", "wait", "confirm", "io", "buzzer"]
        t = types[idx] if idx < len(types) else "conveyor"
        self._conv_frame.setVisible(t == "conveyor")
        self._wait_frame.setVisible(t == "wait")

    def _load_form(self, task: dict):
        """Fill form từ task dict."""
        types   = ["conveyor", "wait", "confirm", "io", "buzzer"]
        t_type  = task.get("type", "conveyor")
        t_idx   = types.index(t_type) if t_type in types else 0
        self._type_combo.blockSignals(True)
        self._type_combo.setCurrentIndex(t_idx)
        self._type_combo.blockSignals(False)
        self._on_type_change(t_idx)

        order = task.get("order", "sequential")
        self._order_combo.setCurrentIndex(
            0 if order == "sequential" else 1)

        if t_type == "conveyor":
            ids = task.get("conveyor_ids", [1])
            for i, cb in enumerate(self._conv_checks):
                cb.setChecked((i + 1) in ids)
            mode = task.get("conveyor_mode", "receive")
            self._conv_mode.setCurrentIndex(0 if mode == "receive" else 1)
            self._conv_speed.setValue(int(task.get("speed", 75)))
            self._conv_dur.setValue(float(task.get("duration", 5.0)))
        elif t_type == "wait":
            self._wait_spin.setValue(int(task.get("delay", 3)))

    def _apply_form(self):
        """Lưu form vào task đang edit."""
        if self._editing_idx < 0 or self._editing_idx >= len(self._tasks):
            return
        types = ["conveyor", "wait", "confirm", "io", "buzzer"]
        t_idx = self._type_combo.currentIndex()
        t_type = types[t_idx] if t_idx < len(types) else "conveyor"
        order = "sequential" if self._order_combo.currentIndex() == 0 else "parallel"

        task: dict = {"type": t_type, "order": order}

        if t_type == "conveyor":
            ids = [i + 1 for i, cb in enumerate(self._conv_checks) if cb.isChecked()]
            if not ids:
                ids = [1]
            mode = "receive" if self._conv_mode.currentIndex() == 0 else "send"
            task.update({
                "conveyor_ids":   ids,
                "conveyor_mode":  mode,
                "speed":          self._conv_speed.value(),
                "duration":       self._conv_dur.value(),
            })
        elif t_type == "wait":
            task["delay"] = self._wait_spin.value()

        self._tasks[self._editing_idx] = task
        self._refresh_list()
        self._task_list.setCurrentRow(self._editing_idx)

    # ── Public API ───────────────────────────────────────────────────

    def load_tasks(self, tasks: list[dict]):
        """Load tasks từ waypoint đã lưu."""
        self._tasks = copy.deepcopy(tasks)
        self._editing_idx = -1
        self._refresh_list()
        self._form.setVisible(False)

    def get_tasks(self) -> list[dict]:
        return copy.deepcopy(self._tasks)

    def clear(self):
        self._tasks.clear()
        self._task_list.clear()
        self._editing_idx = -1
        self._form.setVisible(False)