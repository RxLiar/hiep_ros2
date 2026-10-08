"""prefs.py - small JSON store for HMI preferences (~/.agv_hmi/hmi_prefs.json)."""
from __future__ import annotations

import json
import os

PREFS_FILE = os.path.expanduser("~/.agv_hmi/hmi_prefs.json")

DEFAULTS = {
    "preflight_enabled": True,
    "sound_enabled": False,
    "tower_enabled": False,          # drives the PLC lights; off until wired/configured
    "tower_mask_idle": 0b0001,
    "tower_mask_running": 0b0011,
    "tower_mask_paused": 0b0101,
    "tower_mask_error": 0b1000,
    "joy_enabled": True,
    "joy_deadman_button": 4,         # LB on an Xbox-style pad
    "joy_lin_axis": 1,
    "joy_ang_axis": 0,
    "joy_max_lin": 0.4,
    "joy_max_ang": 0.6,
    "operator_kiosk": False,
    "schedule_enabled": False,      # allow routes to start by themselves from the schedule
}
_cache: dict | None = None


def _load() -> dict:
    global _cache
    if _cache is None:
        data = {}
        try:
            with open(PREFS_FILE, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            pass
        _cache = {**DEFAULTS, **(data if isinstance(data, dict) else {})}
    return _cache


def get(key: str):
    return _load().get(key, DEFAULTS.get(key))


def set(key: str, value) -> None:  # noqa: A001 - mirrors dict-like API
    _load()[key] = value
    try:
        os.makedirs(os.path.dirname(PREFS_FILE), exist_ok=True)
        with open(PREFS_FILE, "w", encoding="utf-8") as f:
            json.dump(_load(), f, indent=2)
    except OSError:
        pass
