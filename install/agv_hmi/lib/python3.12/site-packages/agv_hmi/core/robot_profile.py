"""robot_profile.py - robot size, lidar pose and safety zones (several named profiles).

Stored in ~/.agv_hmi/robot_profile.json:
    {"active": "AGV-1800", "profiles": {"AGV-1800": {...}, "Model-ESP32": {...}}}
The ROS node `safety_monitor` (package hiep_robot2) reads the same file at start-up
and receives live changes on /safety_config.  Geometry is in the robot base frame
(x forward, y left, metres); the body is a rectangle that may be offset from base_link.
"""
from __future__ import annotations

import copy
import hashlib
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
    # Lidar set-up: "merged" = ONE scan topic already in the base frame (/scan from laser_merger);
    # "dual" = TWO lidars, each with its own topic and pose (lidar 1 = lidar_x/y/yaw_deg, lidar 2 = lidar2_*).
    "lidar_mode": "merged", "lidar1_topic": "/front_lidar/scan", "lidar2_topic": "/rear_lidar/scan",
    "lidar2_x_m": -0.9, "lidar2_y_m": 0.0, "lidar2_yaw_deg": 180.0,
    # "agv" = conveyor AGV, "forklift" = fork lift (adds the fork block below)
    "robot_type": "agv",
    "fork_length_m": 1.10,         # fork tips back to the carriage / body
    "fork_width_m": 0.90,          # overall width of the two forks
    "load_width_m": 1.20,          # widest pallet carried
    "max_height_mm": 1600.0,
    "fork": {
        "approach_height_mm": 80.0, "lift_height_mm": 200.0, "travel_height_mm": 150.0,
        "release_clearance_mm": 60.0, "insert_depth_m": 1.00, "retract_m": 1.30,
        "insert_speed_mps": 0.08, "retract_speed_mps": 0.10, "load_overhang_m": 0.10,
        "bypass_ahead_m": 1.60, "bypass_margin_m": 0.10,
        # advanced (fork motion control)
        "height_tol_mm": 15.0, "max_fork_speed_pct": 70.0, "min_fork_speed_pct": 25.0, "slow_zone_mm": 80.0,
        "stall_s": 2.5, "blocked_s": 4.0, "height_step_timeout_s": 60.0, "drive_step_timeout_s": 60.0,
    },
}

ROBOT_TYPES = ("agv", "forklift")
LIDAR_MODES = ("merged", "dual")
# starting point for a new fork-lift profile (the lidar position is a guess: set yours)
FORKLIFT_PRESET = {"robot_type": "forklift", "length_m": 2.4, "width_m": 1.0, "center_x_m": 0.0,
                   "lidar_x_m": -0.6, "lidar_y_m": 0.0,
                   "stop": {"front": 0.30, "rear": 0.40, "left": 0.10, "right": 0.10},
                   "slow": {"front": 1.00, "rear": 0.80, "left": 0.40, "right": 0.40}}


def _num(d, key, lo, hi, default):
    try:
        v = float(d[key])
        return default if not math.isfinite(v) else max(lo, min(hi, v))
    except (KeyError, TypeError, ValueError):
        return default


def _topic(v, default: str) -> str:
    """Same rule as the ROS node: a topic must look like /abc/def."""
    t = str(v).strip() if v is not None else ""
    return t if t.startswith("/") and " " not in t and 1 < len(t) <= 120 else default


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
                        ("slow_angular_rps", 0.05, 3.0), ("lidar2_x_m", -5.0, 5.0),
                        ("lidar2_y_m", -2.0, 2.0), ("lidar2_yaw_deg", -180.0, 180.0)):
        out[key] = _num(d, key, lo, hi, base[key])
    for zone in ("stop", "slow"):
        src = d.get(zone) if isinstance(d.get(zone), dict) else {}
        for s in SIDES:
            out[zone][s] = _num(src, s, 0.0, 5.0, base[zone][s])
    for s in SIDES:                                    # the slow zone always contains the stop zone
        out["slow"][s] = max(out["slow"][s], out["stop"][s])
    out["robot_type"] = d.get("robot_type") if d.get("robot_type") in ROBOT_TYPES else base["robot_type"]
    for key, lo, hi in (("fork_length_m", 0.1, 5.0), ("fork_width_m", 0.1, 3.0), ("load_width_m", 0.1, 3.0),
                        ("max_height_mm", 100.0, 8000.0)):
        out[key] = _num(d, key, lo, hi, base[key])
    fsrc = d.get("fork") if isinstance(d.get("fork"), dict) else {}
    for key, lo, hi in (("approach_height_mm", 0, 8000), ("lift_height_mm", 0, 8000), ("travel_height_mm", 0, 8000),
                        ("release_clearance_mm", 0, 500), ("insert_depth_m", 0.05, 3.0), ("retract_m", 0.05, 4.0),
                        ("insert_speed_mps", 0.02, 0.30), ("retract_speed_mps", 0.02, 0.50),
                        ("load_overhang_m", 0.0, 1.0), ("bypass_ahead_m", 0.2, 4.0), ("bypass_margin_m", 0.0, 0.5),
                        ("height_tol_mm", 2, 100), ("max_fork_speed_pct", 10, 100), ("min_fork_speed_pct", 5, 100),
                        ("slow_zone_mm", 10, 500), ("stall_s", 0.5, 10), ("blocked_s", 1, 20),
                        ("height_step_timeout_s", 5, 300), ("drive_step_timeout_s", 5, 300)):
        out["fork"][key] = _num(fsrc, key, lo, hi, base["fork"][key])
    out["lidar_mode"] = d.get("lidar_mode") if d.get("lidar_mode") in LIDAR_MODES else base["lidar_mode"]
    out["lidar1_topic"] = _topic(d.get("lidar1_topic"), base["lidar1_topic"])
    out["lidar2_topic"] = _topic(d.get("lidar2_topic"), base["lidar2_topic"])
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


def scan_to_base(ranges, angle_min, angle_inc, range_min, range_max, p: dict, pose=None) -> np.ndarray:
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + angle_inc * np.arange(r.size)
    ok = np.isfinite(r) & (r >= max(range_min, 0.02)) & (r <= range_max)
    r, ang = r[ok], ang[ok]
    px, py, yaw = pose if pose is not None else (p["lidar_x_m"], p["lidar_y_m"], math.radians(p["lidar_yaw_deg"]))
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = r * np.cos(ang), r * np.sin(ang)
    return np.stack([px + c * lx - s * ly, py + s * lx + c * ly], axis=1)


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


def new_profile(name: str, robot_type: str = "agv") -> dict:
    d = normalize(FORKLIFT_PRESET if robot_type == "forklift" else None)
    d["name"] = name
    return d


def is_forklift(p: dict) -> bool:
    return p.get("robot_type") == "forklift"


def fork_bypass_rect(p: dict, carrying: bool = False):
    """(x0, x1, y0, y1) of the area ahead of the fork tips that is bypassed during INSERT."""
    front = p["center_x_m"] + p["length_m"] / 2
    width = max(p["fork_width_m"], p["load_width_m"] if carrying else p["fork_width_m"])
    half = width / 2 + p["fork"]["bypass_margin_m"]
    return (front - p["fork_length_m"], front + p["fork"]["bypass_ahead_m"], -half, half)


def signature(p: dict) -> str:
    """Same hash as hiep_robot2 Profile.signature(): lets the HMI check what the safety node really runs."""
    r = lambda v: round(float(v), 3)
    data = [r(p["length_m"]), r(p["width_m"]), r(p["center_x_m"]), r(p["center_y_m"]),
            [r(p["stop"][k]) for k in SIDES], [r(p["slow"][k]) for k in SIDES],
            r(p["slow_linear_mps"]), r(p["slow_angular_rps"]), bool(p["safety_enabled"]),
            bool(p["require_scan"]), int(p["min_points"]),
            r(p["lidar_x_m"]), r(p["lidar_y_m"]), r(p["lidar_yaw_deg"]),
            str(p["lidar_mode"]), str(p["lidar1_topic"]), str(p["lidar2_topic"]),
            r(p["lidar2_x_m"]), r(p["lidar2_y_m"]), r(p["lidar2_yaw_deg"])]
    return hashlib.sha1(json.dumps(data).encode()).hexdigest()[:8]


def fork_effective_config(p: dict) -> dict:
    """What forklift_node should be running for this profile (same names as its ForkConfig)."""
    d = {k: float(v) for k, v in p["fork"].items()}
    d["fork_length_m"], d["fork_width_m"] = p["fork_length_m"], p["fork_width_m"]
    d["load_width_m"], d["max_height_mm"] = p["load_width_m"], p["max_height_mm"]
    d["front_x_m"] = p["center_x_m"] + p["length_m"] / 2
    return d
