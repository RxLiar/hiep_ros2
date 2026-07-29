"""Shared JSON protocol helpers for the AGV fleet prototype."""
from __future__ import annotations

import json
import re
import time
from dataclasses import asdict, dataclass, field
from typing import Any

MAP_ID_RE = re.compile(r"^M\d{5}$")
ROBOT_ID_RE = re.compile(r"^[A-Z][A-Z0-9_-]{2,31}$")


def validate_map_id(value: str) -> str:
    map_id = str(value or "").strip().upper()
    if not MAP_ID_RE.fullmatch(map_id):
        raise ValueError("map_id must use format M + 5 digits, for example M02101")
    return map_id


def validate_robot_id(value: str) -> str:
    robot_id = str(value or "").strip().upper()
    if not ROBOT_ID_RE.fullmatch(robot_id):
        raise ValueError("robot_id must be 3-32 uppercase letters, digits, '_' or '-'")
    return robot_id


@dataclass(slots=True)
class RobotState:
    robot_id: str
    robot_name: str
    map_id: str
    map_version: int = 1
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    battery_percent: int = -1
    connection: str = "online"
    moving: bool = False
    nav_state: str = "idle"
    mission_id: str = ""
    route_id: str = ""
    route_name: str = ""
    waypoint_current: int = 0
    waypoint_total: int = 0
    error_level: str = "ok"
    error_message: str = ""
    emergency_stop: bool = False
    cargo_count: int = 0
    sequence: int = 0
    stamp: float = field(default_factory=time.time)

    def normalize(self) -> "RobotState":
        self.robot_id = validate_robot_id(self.robot_id)
        self.map_id = validate_map_id(self.map_id)
        self.robot_name = str(self.robot_name or self.robot_id).strip()
        self.map_version = max(1, int(self.map_version))
        self.battery_percent = max(-1, min(100, int(self.battery_percent)))
        self.connection = str(self.connection or "online").strip().lower()
        self.nav_state = str(self.nav_state or "idle").strip().lower()
        self.error_level = str(self.error_level or "ok").strip().lower()
        if self.error_level not in {"ok", "warn", "error"}:
            self.error_level = "error"
        self.stamp = float(self.stamp or time.time())
        return self

    def to_dict(self) -> dict[str, Any]:
        self.normalize()
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, separators=(",", ":"))

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RobotState":
        allowed = cls.__dataclass_fields__.keys()
        values = {key: data[key] for key in allowed if key in data}
        state = cls(**values)
        return state.normalize()

    @classmethod
    def from_json(cls, payload: str) -> "RobotState":
        data = json.loads(payload)
        if not isinstance(data, dict):
            raise ValueError("robot state payload must be a JSON object")
        return cls.from_dict(data)


def make_event(event_type: str, robot_id: str, message: str, **extra: Any) -> str:
    payload = {
        "event_type": str(event_type),
        "robot_id": validate_robot_id(robot_id),
        "message": str(message),
        "stamp": time.time(),
        **extra,
    }
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
