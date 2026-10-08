"""robot_profile.py - robot size, lidar pose and safety zones (several named profiles).

Stored in ~/.agv_hmi/robot_profile.json:
    {"active": "AGV-1800", "profiles": {"AGV-1800": {...}, "Model-ESP32": {...}}}
The ROS node `safety_monitor` (package hiep_robot2) reads the same file at start-up
and receives live changes on /safety_config.  Geometry is in the robot base frame
(x forward, y left, metres); the body is a rectangle that may be offset from base_link.
"""
from __future__ import annotations

import copy
import json
import math
import os

import numpy as np

PROFILE_FILE = os.path.expanduser("~/.agv_hmi/robot_profile.json")
SIDES = ("front", "rear", "left", "right")
MAX_MARGIN = 3.0

DEFAULT_PROFILE = {
    "name": "AGV-1800x700",
    "length_m": 1.8, "width_m": 0.7, "center_x_m": 0.0, "center_y_m": 0.0,
    "lidar_x_m": 0.9, "lidar_y_m": 0.0, "lidar_yaw_deg": 0.0,
    "stop": {"front": 0.25, "rear": 0.25, "left": 0.10, "right": 0.10},
    "slow": {"front": 1.00, "rear": 0.60, "left": 0.40, "right": 0.40},
    "slow_linear_mps": 0.15, "slow_angular_rps": 0.30,
    "safety_enabled": True, "require_scan": True, "min_points": 3,
}


def _num(d, key, lo, hi, default):
    try:
        v = float(d[key])
        return default if not math.isfinite(v) else max(lo, min(hi, v))
    except (KeyError, TypeError, ValueError):
        return default


def normalize(d: dict | None) -> dict:
    """Clamp every value to a sane range (same rules as the ROS node)."""
    base = copy.deepcopy(DEFAULT_PROFILE)
    if not isinstance(d, dict):
        return base
    out = base
    out["name"] = (str(d.get("name", base["name"])).strip() or base["name"])[:60]
    for key, lo, hi in (("length_m", 0.1, 10.0), ("width_m", 0.1, 5.0), ("center_x_m", -5.0, 5.0),
                        ("center_y_m", -2.0, 2.0), ("lidar_x_m", -5.0, 5.0), ("lidar_y_m", -2.0, 2.0),
                        ("lidar_yaw_deg", -180.0, 180.0), ("slow_linear_mps", 0.02, 2.0),
                        ("slow_angular_rps", 0.05, 3.0)):
        out[key] = _num(d, key, lo, hi, base[key])
    for zone in ("stop", "slow"):
        src = d.get(zone) if isinstance(d.get(zone), dict) else {}
        for s in SIDES:
            out[zone][s] = _num(src, s, 0.0, 5.0, base[zone][s])
    for s in SIDES:                                    # the slow zone always contains the stop zone
        out["slow"][s] = max(out["slow"][s], out["stop"][s])
    out["safety_enabled"] = bool(d.get("safety_enabled", base["safety_enabled"]))
    out["require_scan"] = bool(d.get("require_scan", base["require_scan"]))
    try:
        out["min_points"] = max(1, min(50, int(d.get("min_points", base["min_points"]))))
    except (TypeError, ValueError):
        pass
    return out


# -- geometry ------------------------------------------------------------------
def footprint_rect(p: dict):
    return (p["center_x_m"] - p["length_m"] / 2, p["center_x_m"] + p["length_m"] / 2,
            p["center_y_m"] - p["width_m"] / 2, p["center_y_m"] + p["width_m"] / 2)


def zone_rect(p: dict, kind: str):
    m = p["stop"] if kind == "stop" else p["slow"]
    x0, x1, y0, y1 = footprint_rect(p)
    return (x0 - m["rear"], x1 + m["front"], y0 - m["right"], y1 + m["left"])


def nav2_footprint(p: dict) -> str:
    x0, x1, y0, y1 = footprint_rect(p)
    return "[" + ", ".join(f"[{x:.3f}, {y:.3f}]" for x, y in ((x1, y1), (x1, y0), (x0, y0), (x0, y1))) + "]"


def scan_to_base(ranges, angle_min, angle_inc, range_min, range_max, p: dict) -> np.ndarray:
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + angle_inc * np.arange(r.size)
    ok = np.isfinite(r) & (r >= max(range_min, 0.02)) & (r <= range_max)
    r, ang = r[ok], ang[ok]
    yaw = math.radians(p["lidar_yaw_deg"]); c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = r * np.cos(ang), r * np.sin(ang)
    return np.stack([p["lidar_x_m"] + c * lx - s * ly, p["lidar_y_m"] + s * lx + c * ly], axis=1)


def classify_points(pts: np.ndarray, p: dict, self_margin: float = 0.03):
    """Per-point class: 0 = own body, 1 = outside, 2 = slow zone, 3 = stop zone."""
    if len(pts) == 0:
        return np.zeros(0, dtype=int)
    x, y = pts[:, 0], pts[:, 1]
    fx0, fx1, fy0, fy1 = footprint_rect(p)
    own = (x > fx0 - self_margin) & (x < fx1 + self_margin) & (y > fy0 - self_margin) & (y < fy1 + self_margin)
    out = np.ones(len(pts), dtype=int)
    for kind, val in (("slow", 2), ("stop", 3)):
        a0, a1, b0, b1 = zone_rect(p, kind)
        out[(x >= a0) & (x <= a1) & (y >= b0) & (y <= b1)] = val
    out[own] = 0
    return out


# -- store ----------------------------------------------------------------------
def load_store(path: str = PROFILE_FILE) -> dict:
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        profiles = {k: normalize(dict(v, name=k)) for k, v in data.get("profiles", {}).items() if isinstance(v, dict)}
        if profiles:
            active = data.get("active") if data.get("active") in profiles else next(iter(profiles))
            return {"active": active, "profiles": profiles}
    except (OSError, ValueError, AttributeError):
        pass
    d = normalize(None)
    return {"active": d["name"], "profiles": {d["name"]: d}}


def save_store(store: dict, path: str = PROFILE_FILE) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(store, f, indent=2)
    os.replace(tmp, path)          # the ROS node may read it at any time


def active_profile(store: dict) -> dict:
    return store["profiles"][store["active"]]
