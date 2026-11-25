"""safety_override.py - shared "Engineer teleop: safety zones OFF" switch.

One instance for the whole app, so the three joysticks (Mapping, Navigation, Routes) and the gamepad always agree.
The switch is only a request: MainWindow republishes it to /safety_override at 5 Hz and the ROS safety_monitor
lets it expire after at most 2 s, so a crash / closed window / lost link puts the zones back by itself.
"""
from PyQt6.QtCore import QObject, pyqtSignal


class _SafetyOverride(QObject):
    changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self._on = False
        self.confirmed = False          # the warning dialog is shown once per session

    def is_on(self) -> bool:
        return self._on

    def set(self, on: bool) -> None:
        on = bool(on)
        if on != self._on:
            self._on = on
            self.changed.emit(on)


OVERRIDE = _SafetyOverride()
