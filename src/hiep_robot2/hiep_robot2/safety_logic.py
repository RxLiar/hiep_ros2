"""safety_logic.py - pure (no ROS) logic of the safety monitor.

Coordinates are in the robot base frame (x forward, y left, metres). The
footprint is a rectangle that may be offset from base_link; the stop and slow
zones are that rectangle grown by one margin per side.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple

import numpy as np

SIDES = ("front", "rear", "left", "right")


def _margins(src, default) -> Dict[str, float]:
    out = dict(default)
    if isinstance(src, dict):
        for k in SIDES:
            try:
                out[k] = max(0.0, min(5.0, float(src[k])))
            except (KeyError, TypeError, ValueError):
                pass
    return out


@dataclass
class Profile:
    name: str = "default"
    length_m: float = 1.8
    width_m: float = 0.7
    center_x_m: float = 0.0          # footprint centre relative to base_link (forward +)
    center_y_m: float = 0.0
    lidar_x_m: float = 0.9           # lidar pose in base_link
    lidar_y_m: float = 0.0
    lidar_yaw_deg: float = 0.0
    stop: Dict[str, float] = field(default_factory=lambda: {"front": 0.25, "rear": 0.25, "left": 0.10, "right": 0.10})
    slow: Dict[str, float] = field(default_factory=lambda: {"front": 1.00, "rear": 0.60, "left": 0.40, "right": 0.40})
    slow_linear_mps: float = 0.15
    slow_angular_rps: float = 0.30
    safety_enabled: bool = True
    require_scan: bool = True
    min_points: int = 3

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> "Profile":
        p = cls()
        if not isinstance(d, dict):
            return p

        def num(key, lo, hi, cur):
            try:
                v = float(d[key])
                return cur if not math.isfinite(v) else max(lo, min(hi, v))
            except (KeyError, TypeError, ValueError):
                return cur

        p.name = str(d.get("name", p.name))[:60] or p.name
        p.length_m = num("length_m", 0.1, 10.0, p.length_m)
        p.width_m = num("width_m", 0.1, 5.0, p.width_m)
        p.center_x_m = num("center_x_m", -5.0, 5.0, p.center_x_m)
        p.center_y_m = num("center_y_m", -2.0, 2.0, p.center_y_m)
        p.lidar_x_m = num("lidar_x_m", -5.0, 5.0, p.lidar_x_m)
        p.lidar_y_m = num("lidar_y_m", -2.0, 2.0, p.lidar_y_m)
        p.lidar_yaw_deg = num("lidar_yaw_deg", -180.0, 180.0, p.lidar_yaw_deg)
        p.stop = _margins(d.get("stop"), p.stop)
        p.slow = _margins(d.get("slow"), p.slow)
        # the slow zone always contains the stop zone
        for k in SIDES:
            p.slow[k] = max(p.slow[k], p.stop[k])
        p.slow_linear_mps = num("slow_linear_mps", 0.02, 2.0, p.slow_linear_mps)
        p.slow_angular_rps = num("slow_angular_rps", 0.05, 3.0, p.slow_angular_rps)
        p.safety_enabled = bool(d.get("safety_enabled", p.safety_enabled))
        p.require_scan = bool(d.get("require_scan", p.require_scan))
        try:
            p.min_points = max(1, min(50, int(d.get("min_points", p.min_points))))
        except (TypeError, ValueError):
            pass
        return p

    def to_dict(self) -> dict:
        return asdict(self)

    def signature(self) -> str:
        """Short hash of everything that changes the zones; the HMI computes the same value to check
        that the node really runs the configuration that was applied."""
        r = lambda v: round(float(v), 3)
        data = [r(self.length_m), r(self.width_m), r(self.center_x_m), r(self.center_y_m),
                [r(self.stop[k]) for k in SIDES], [r(self.slow[k]) for k in SIDES],
                r(self.slow_linear_mps), r(self.slow_angular_rps), bool(self.safety_enabled),
                bool(self.require_scan), int(self.min_points),
                r(self.lidar_x_m), r(self.lidar_y_m), r(self.lidar_yaw_deg)]
        return hashlib.sha1(json.dumps(data).encode()).hexdigest()[:8]

    # -- geometry -----------------------------------------------------------
    def footprint_rect(self) -> Tuple[float, float, float, float]:
        """(xmin, xmax, ymin, ymax) of the robot body in base_link."""
        return (self.center_x_m - self.length_m / 2, self.center_x_m + self.length_m / 2,
                self.center_y_m - self.width_m / 2, self.center_y_m + self.width_m / 2)

    def footprint_polygon(self) -> List[Tuple[float, float]]:
        x0, x1, y0, y1 = self.footprint_rect()
        return [(x1, y1), (x1, y0), (x0, y0), (x0, y1)]       # front-left, front-right, rear-right, rear-left

    def nav2_footprint(self) -> str:
        return "[" + ", ".join(f"[{x:.3f}, {y:.3f}]" for x, y in self.footprint_polygon()) + "]"

    def zone_rect(self, kind: str) -> Tuple[float, float, float, float]:
        m = self.stop if kind == "stop" else self.slow
        x0, x1, y0, y1 = self.footprint_rect()
        return (x0 - m["rear"], x1 + m["front"], y0 - m["right"], y1 + m["left"])


def load_active_profile(path: str) -> Profile:
    """Read the HMI's robot_profile.json ({"active": name, "profiles": {name: {...}}})."""
    try:
        with open(os.path.expanduser(path), encoding="utf-8") as f:
            data = json.load(f)
        profiles = data.get("profiles", {})
        d = profiles.get(data.get("active"))
        if d is None and profiles:
            d = next(iter(profiles.values()))
        prof = Profile.from_dict(d)
        if d and "name" not in d:
            prof.name = str(data.get("active", prof.name))
        return prof
    except (OSError, ValueError, AttributeError):
        return Profile()


# -- scan -> points ----------------------------------------------------------
def scan_to_base_points(ranges, angle_min: float, angle_inc: float, range_min: float,
                        range_max: float, prof: Profile, pose=None) -> np.ndarray:
    """Scan -> points in the base frame. `pose` = (x, y, yaw_rad) of the scan frame in the base frame
    (from TF); without it the manual lidar pose of the profile is used."""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + angle_inc * np.arange(r.size)
    ok = np.isfinite(r) & (r >= max(range_min, 0.02)) & (r <= range_max)
    r, ang = r[ok], ang[ok]
    px, py, yaw = pose if pose is not None else (prof.lidar_x_m, prof.lidar_y_m, math.radians(prof.lidar_yaw_deg))
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = r * np.cos(ang), r * np.sin(ang)
    return np.stack([px + c * lx - s * ly, py + s * lx + c * ly], axis=1)


@dataclass
class ZoneResult:
    stop_front: int = 0
    stop_rear: int = 0
    stop_side: int = 0
    slow_front: int = 0
    slow_rear: int = 0
    slow_side: int = 0
    nearest_m: Optional[float] = None
    nearest_xy: Optional[Tuple[float, float]] = None

    @property
    def stop_any(self): return self.stop_front + self.stop_rear + self.stop_side
    @property
    def slow_any(self): return self.slow_front + self.slow_rear + self.slow_side


def classify(points: np.ndarray, prof: Profile, self_margin: float = 0.03,
             ignore_rects: Optional[List[Tuple[float, float, float, float]]] = None) -> ZoneResult:
    res = ZoneResult()
    if points is None or len(points) == 0:
        return res
    fx0, fx1, fy0, fy1 = prof.footprint_rect()
    x, y = points[:, 0], points[:, 1]
    # the lidar sees the robot's own body: drop returns inside the footprint
    own = (x > fx0 - self_margin) & (x < fx1 + self_margin) & (y > fy0 - self_margin) & (y < fy1 + self_margin)
    # bypass areas (e.g. the pallet in front of the fork tips): only these points are ignored,
    # everything else around the robot still counts
    for (bx0, bx1, by0, by1) in (ignore_rects or []):
        own |= (x >= bx0) & (x <= bx1) & (y >= by0) & (y <= by1)
    pts = points[~own]
    if len(pts) == 0:
        return res
    x, y = pts[:, 0], pts[:, 1]
    sx0, sx1, sy0, sy1 = prof.zone_rect("stop")
    wx0, wx1, wy0, wy1 = prof.zone_rect("slow")
    in_stop = (x >= sx0) & (x <= sx1) & (y >= sy0) & (y <= sy1)
    in_slow = (x >= wx0) & (x <= wx1) & (y >= wy0) & (y <= wy1)
    # Sectors by where the point is: beyond the front edge, beyond the rear edge, or BESIDE the body.
    # Beside-the-body points count for both directions (a person walking along the truck is a hazard
    # whether it drives forwards or backwards).
    front = x > fx1
    rear = x < fx0
    side = ~front & ~rear
    res.stop_front = int((in_stop & front).sum())
    res.stop_rear = int((in_stop & rear).sum())
    res.stop_side = int((in_stop & side).sum())
    res.slow_front = int((in_slow & front).sum())
    res.slow_rear = int((in_slow & rear).sum())
    res.slow_side = int((in_slow & side).sum())
    d = np.hypot(x - prof.center_x_m, y - prof.center_y_m)
    k = int(d.argmin())
    # nearest distance to the footprint edge (not to its centre)
    dx = np.maximum(np.maximum(fx0 - x, x - fx1), 0.0)
    dy = np.maximum(np.maximum(fy0 - y, y - fy1), 0.0)
    edge = np.hypot(dx, dy)
    j = int(edge.argmin())
    res.nearest_m = float(edge[j])
    res.nearest_xy = (float(x[j]), float(y[j]))
    return res


class Debouncer:
    """A zone flag turns on at `min_points` and stays on `hold_s` after the last hit."""

    def __init__(self, hold_s: float = 0.3):
        self.hold_s = hold_s
        self._last_on: Dict[str, float] = {}

    def update(self, now: float, counts: Dict[str, int], min_points: int) -> Dict[str, bool]:
        out = {}
        for k, n in counts.items():
            if n >= min_points:
                self._last_on[k] = now
            out[k] = (now - self._last_on.get(k, -1e9)) <= self.hold_s
        return out


EPS = 1e-3


def gate(lin: float, ang: float, flags: Dict[str, bool], prof: Profile,
         block_rotation_in_stop: bool = True) -> Tuple[float, float]:
    """Apply the stop/slow zones to a velocity command.

    Zones beside the body (`*_side`) are hazards for both directions of travel; zones beyond the front
    or rear edge only matter when driving towards them (reversing away from them stays possible).
    """
    sf, sr, ss = (flags.get("stop_front", False), flags.get("stop_rear", False), flags.get("stop_side", False))
    wf, wr, ws = (flags.get("slow_front", False), flags.get("slow_rear", False), flags.get("slow_side", False))
    stop_fwd, stop_rev = sf or ss, sr or ss
    slow_fwd, slow_rev = wf or ws, wr or ws
    out_lin, out_ang = lin, ang
    if lin > EPS and stop_fwd:
        out_lin = 0.0
    if lin < -EPS and stop_rev:
        out_lin = 0.0
    if (sf or sr or ss) and block_rotation_in_stop:
        out_ang = 0.0
    # slow zone: limit speed when moving towards it, or when turning with something close
    moving_to_slow = (out_lin > EPS and slow_fwd) or (out_lin < -EPS and slow_rev)
    turning_near = abs(out_lin) <= EPS and (wf or wr or ws)
    if moving_to_slow or turning_near:
        out_lin = max(-prof.slow_linear_mps, min(prof.slow_linear_mps, out_lin))
        out_ang = max(-prof.slow_angular_rps, min(prof.slow_angular_rps, out_ang))
    return out_lin, out_ang


def state_name(flags: Dict[str, bool]) -> str:
    if flags.get("stop_front") or flags.get("stop_rear") or flags.get("stop_side"):
        return "stop"
    if flags.get("slow_front") or flags.get("slow_rear") or flags.get("slow_side"):
        return "slow"
    return "ok"


# -- bypass requests ---------------------------------------------------------
MAX_BYPASS_TTL_S = 3.0
MAX_BYPASS_SPEED = 0.30


class BypassManager:
    """Time-limited, scoped exceptions to the safety zones.

    A request is {"id": str, "ttl": s, "rects": [[x0, x1, y0, y1], ...],
    "max_linear": m/s, "max_angular": rad/s}. It must be refreshed (the forklift node
    publishes it at 5 Hz) or it expires after `ttl` seconds (capped at 3 s). While any
    bypass is active the speed is capped (default 0.10 m/s), so a bypass can never mean
    "drive at full speed through the zone".
    """

    def __init__(self):
        self._items: Dict[str, dict] = {}

    def update(self, now: float, req: dict) -> None:
        try:
            rid = str(req["id"])[:40]
            ttl = max(0.1, min(MAX_BYPASS_TTL_S, float(req.get("ttl", 1.0))))
            rects = []
            for r in req.get("rects", []):
                x0, x1, y0, y1 = (float(v) for v in r)
                rects.append((min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)))
            if not req.get("active", True) or not rects:
                self._items.pop(rid, None)
                return
            self._items[rid] = {
                "until": now + ttl, "rects": rects,
                # cap=false is for "this is my own load" rectangles that must not slow the robot down
                "max_linear": (max(0.02, min(MAX_BYPASS_SPEED, float(req.get("max_linear", 0.10))))
                               if req.get("cap", True) else None),
                "max_angular": (max(0.05, min(1.0, float(req.get("max_angular", 0.20))))
                                if req.get("cap", True) else None),
            }
        except (KeyError, TypeError, ValueError):
            return

    def active(self, now: float) -> Dict[str, dict]:
        self._items = {k: v for k, v in self._items.items() if v["until"] > now}
        return self._items

    def rects(self, now: float):
        return [r for v in self.active(now).values() for r in v["rects"]]

    def speed_caps(self, now: float):
        act = [v for v in self.active(now).values() if v["max_linear"] is not None]
        if not act:
            return None
        return (min(v["max_linear"] for v in act), min(v["max_angular"] for v in act))


def cap_speed(lin: float, ang: float, caps) -> Tuple[float, float]:
    if caps is None:
        return lin, ang
    return (max(-caps[0], min(caps[0], lin)), max(-caps[1], min(caps[1], ang)))
