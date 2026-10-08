"""alarm_manager.py - central alarm list for the HMI.

An alarm has a stable `key` (e.g. "plc.offline"). Raising an active key again
only refreshes it; clearing marks it inactive. History is kept in
~/.agv_hmi/alarms.jsonl so incidents survive restarts.
"""
from __future__ import annotations

import json
import os
import time
from datetime import datetime

from PyQt6.QtCore import QObject, pyqtSignal

LEVELS = ("info", "warn", "error", "critical")
_RANK = {lv: i for i, lv in enumerate(LEVELS)}
ALARM_FILE = os.path.expanduser("~/.agv_hmi/alarms.jsonl")


class AlarmManager(QObject):
    changed = pyqtSignal()
    raised = pyqtSignal(dict)      # emitted once when a NEW alarm becomes active

    def __init__(self, persist: bool = True):
        super().__init__()
        self._persist = persist
        self._seq = 0
        self._items: list[dict] = []          # newest last

    # -- API ----------------------------------------------------------------
    def raise_alarm(self, key: str, level: str, message: str, source: str = "") -> None:
        level = level if level in LEVELS else "warn"
        cur = self._active_by_key(key)
        if cur is not None:
            if cur["message"] != message or cur["level"] != level:
                cur["message"], cur["level"] = message, level
                self.changed.emit()
            return
        self._seq += 1
        item = {"id": self._seq, "key": key, "level": level, "message": message,
                "source": source, "t_raised": time.time(), "t_cleared": None,
                "acked": False}
        self._items.append(item)
        self._write("raised", item)
        self.raised.emit(dict(item))
        self.changed.emit()

    def clear_alarm(self, key: str) -> None:
        cur = self._active_by_key(key)
        if cur is None:
            return
        cur["t_cleared"] = time.time()
        self._write("cleared", cur)
        self.changed.emit()

    def set_condition(self, key: str, bad: bool, level: str, message: str, source: str = ""):
        """Convenience: raise while `bad`, clear when not."""
        if bad:
            self.raise_alarm(key, level, message, source)
        else:
            self.clear_alarm(key)

    def ack(self, alarm_id: int) -> None:
        for it in self._items:
            if it["id"] == alarm_id and not it["acked"]:
                it["acked"] = True
                self._write("acked", it)
        self.changed.emit()

    def ack_all(self) -> None:
        for it in self._items:
            if not it["acked"]:
                it["acked"] = True
                self._write("acked", it)
        self.changed.emit()

    def clear_history(self) -> None:
        """Forget inactive + acknowledged alarms (the log file is kept)."""
        self._items = [i for i in self._items if i["t_cleared"] is None]
        self.changed.emit()

    # -- queries ------------------------------------------------------------
    def items(self) -> list[dict]:
        return list(reversed(self._items))

    def active(self) -> list[dict]:
        return [i for i in self._items if i["t_cleared"] is None]

    def unacked_count(self) -> int:
        return sum(1 for i in self._items if not i["acked"])

    def worst_active_level(self) -> str | None:
        act = self.active()
        if not act:
            return None
        return max((i["level"] for i in act), key=lambda lv: _RANK[lv])

    def has_active(self, key: str) -> bool:
        return self._active_by_key(key) is not None

    # -- internals ----------------------------------------------------------
    def _active_by_key(self, key: str):
        for it in reversed(self._items):
            if it["key"] == key and it["t_cleared"] is None:
                return it
        return None

    def _write(self, event: str, item: dict) -> None:
        if not self._persist:
            return
        try:
            os.makedirs(os.path.dirname(ALARM_FILE), exist_ok=True)
            rec = dict(item, event=event, at=datetime.now().isoformat(timespec="seconds"))
            with open(ALARM_FILE, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except OSError:
            pass
