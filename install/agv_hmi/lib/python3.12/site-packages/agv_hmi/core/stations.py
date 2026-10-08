"""stations.py - named places (stations) the AGV can be sent to."""
from __future__ import annotations

import json
import os
import uuid

STATIONS_FILE = os.path.expanduser("~/.agv_hmi/stations.json")
KINDS = ("normal", "charge", "home")


def load() -> list[dict]:
    try:
        with open(STATIONS_FILE, encoding="utf-8") as f:
            data = json.load(f)
        return [s for s in data if isinstance(s, dict) and "x" in s and "y" in s] if isinstance(data, list) else []
    except (OSError, ValueError):
        return []


def save(items: list[dict]) -> None:
    os.makedirs(os.path.dirname(STATIONS_FILE), exist_ok=True)
    with open(STATIONS_FILE, "w", encoding="utf-8") as f:
        json.dump(items, f, ensure_ascii=False, indent=2)


def add(name: str, x: float, y: float, yaw: float = 0.0, kind: str = "normal", map_path: str = "") -> dict:
    items = load()
    st = {"id": uuid.uuid4().hex[:8], "name": name.strip() or "Station", "kind": kind if kind in KINDS else "normal",
          "x": float(x), "y": float(y), "yaw": float(yaw), "map": map_path}
    items.append(st)
    save(items)
    return st


def remove(station_id: str) -> None:
    save([s for s in load() if s.get("id") != station_id])


def update(station_id: str, **fields) -> None:
    items = load()
    for s in items:
        if s.get("id") == station_id:
            s.update(fields)
    save(items)


def first_of_kind(kind: str):
    for s in load():
        if s.get("kind") == kind:
            return s
    return None
