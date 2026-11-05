"""forklift_logic.py - pure (no ROS) logic of the fork-lift node.

* HeightController: drive the fork to a target height with the PLC's U/D/S command,
  slow near the target, respect limit switches, detect a stalled fork.
* ForkliftCore: task state machine (pick, drop, lift, lower, insert, retract). Only the
  INSERT step asks the safety monitor for a bypass (a rectangle ahead of the fork tips,
  speed capped); the rear zone is never bypassed. While a load is carried, an "own load"
  rectangle is published so the load itself does not stop the robot.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple


@dataclass
class ForkConfig:
    fork_length_m: float = 1.10        # fork tips back to the carriage / body
    fork_width_m: float = 0.90         # overall width of the two forks
    load_width_m: float = 1.20         # widest load carried (pallet)
    load_overhang_m: float = 0.10      # load beyond the fork tips
    front_x_m: float = 1.20            # x of the fork tips in base_link (footprint front edge)
    max_height_mm: float = 1600.0
    height_tol_mm: float = 15.0
    approach_height_mm: float = 80.0   # height for entering a pallet pocket
    lift_height_mm: float = 200.0      # pick: lift the load to this height for transport
    travel_height_mm: float = 150.0    # height while driving with a load
    release_clearance_mm: float = 60.0 # drop: lower this far below the pallet bottom to release
    insert_depth_m: float = 1.00
    retract_m: float = 1.30
    insert_speed_mps: float = 0.08
    retract_speed_mps: float = 0.10
    bypass_ahead_m: float = 1.60       # bypass region reaches this far beyond the fork tips
    bypass_margin_m: float = 0.10
    max_fork_speed_pct: float = 70.0
    min_fork_speed_pct: float = 25.0
    slow_zone_mm: float = 80.0
    stall_s: float = 2.5
    height_step_timeout_s: float = 60.0
    drive_step_timeout_s: float = 60.0
    blocked_s: float = 4.0

    def to_dict(self) -> dict:
        return {k: round(float(v), 3) for k, v in asdict(self).items()}

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> "ForkConfig":
        c = cls()
        if isinstance(d, dict):
            for k in asdict(c):
                if k in d:
                    try:
                        v = float(d[k])
                        if math.isfinite(v):
                            setattr(c, k, v)
                    except (TypeError, ValueError):
                        pass
        return c


@dataclass
class HwState:
    height_mm: float = 0.0
    top: bool = False
    bottom: bool = False
    fault: int = 0
    stamp: float = 0.0


class HeightController:
    """Returns (direction 'U'|'D'|'S', speed %, reached) for a target height."""

    def __init__(self, cfg: ForkConfig):
        self.cfg = cfg

    def command(self, target_mm: float, hw: HwState) -> Tuple[str, float, bool]:
        c = self.cfg
        target = max(0.0, min(c.max_height_mm, target_mm))
        err = target - hw.height_mm
        if abs(err) <= c.height_tol_mm:
            return "S", 0.0, True
        # the limit switches are the final word, whatever the height sensor says
        if err > 0 and hw.top:
            return "S", 0.0, target >= c.max_height_mm - c.height_tol_mm
        if err < 0 and hw.bottom:
            return "S", 0.0, target <= c.height_tol_mm
        frac = min(1.0, abs(err) / max(1.0, c.slow_zone_mm))
        speed = c.min_fork_speed_pct + (c.max_fork_speed_pct - c.min_fork_speed_pct) * frac
        return ("U" if err > 0 else "D"), speed, False


@dataclass
class Outputs:
    lin: float = 0.0
    ang: float = 0.0
    drive: bool = False              # publish a cmd_vel this tick
    fork_dir: str = "S"
    fork_speed: float = 0.0
    bypass: List[dict] = field(default_factory=list)


class ForkliftCore:
    def __init__(self, cfg: ForkConfig):
        self.cfg = cfg
        self.hc = HeightController(cfg)
        self.carrying = False
        self.state = "idle"          # idle | running | done | error
        self.task_id = ""
        self.task_name = ""
        self.message = ""
        self._steps: List[dict] = []
        self._i = 0
        self._t0 = 0.0
        self._p0: Optional[Tuple[float, float]] = None
        self._h_ref = (0.0, 0.0)     # (time, height) for stall detection
        self._move_ref = (0.0, 0.0, 0.0)   # (time, x, y) for blocked detection

    # -- task creation -------------------------------------------------------
    def build_steps(self, cmd: dict) -> List[dict]:
        c = self.cfg
        g = lambda k, d: float(cmd.get(k, d))
        name = cmd.get("cmd")
        if name == "pick":
            return [
                {"type": "height", "target": g("approach_height_mm", c.approach_height_mm), "name": "approach_height"},
                {"type": "drive", "dist": g("insert_depth_m", c.insert_depth_m), "dir": 1, "bypass": True, "name": "insert"},
                {"type": "height", "target": g("lift_height_mm", c.lift_height_mm), "name": "lift"},
                {"type": "carry", "value": True},
                {"type": "drive", "dist": g("retract_m", c.retract_m), "dir": -1, "bypass": False, "name": "retract"},
            ]
        if name == "drop":
            rel = g("drop_height_mm", c.lift_height_mm) - g("release_clearance_mm", c.release_clearance_mm)
            return [
                {"type": "height", "target": g("drop_height_mm", c.lift_height_mm), "name": "raise_to_level"},
                {"type": "drive", "dist": g("insert_depth_m", c.insert_depth_m), "dir": 1, "bypass": True, "name": "insert"},
                {"type": "height", "target": max(0.0, rel), "name": "release"},
                {"type": "carry", "value": False},
                {"type": "drive", "dist": g("retract_m", c.retract_m), "dir": -1, "bypass": False, "name": "retract"},
                {"type": "height", "target": g("travel_height_mm", c.travel_height_mm), "name": "travel_height"},
            ]
        if name in ("lift", "lower", "to_height"):
            return [{"type": "height", "target": g("height_mm", c.lift_height_mm if name == "lift" else c.approach_height_mm), "name": name}]
        if name == "insert":
            return [{"type": "drive", "dist": g("insert_depth_m", c.insert_depth_m), "dir": 1, "bypass": True, "name": "insert"}]
        if name == "retract":
            return [{"type": "drive", "dist": g("retract_m", c.retract_m), "dir": -1, "bypass": False, "name": "retract"}]
        return []

    def start(self, now: float, cmd: dict) -> Tuple[bool, str]:
        if cmd.get("cmd") == "set_carrying":
            self.carrying = bool(cmd.get("value"))
            return True, "carrying=%s" % self.carrying
        steps = self.build_steps(cmd)
        if not steps:
            return False, "unknown command"
        if self.state == "running":
            return False, "busy"
        self._steps, self._i = steps, 0
        self.state, self.message = "running", ""
        self.task_id = str(cmd.get("id", ""))
        self.task_name = str(cmd.get("cmd"))
        self._enter_step(now, None)
        return True, "started"

    def abort(self, message: str = "aborted") -> None:
        if self.state == "running":
            self.state, self.message = "error", message
        self._steps, self._i = [], 0

    def reset(self) -> None:
        self.state, self.message, self._steps, self._i = "idle", "", [], 0

    # -- step handling ---------------------------------------------------------
    def _enter_step(self, now: float, pose) -> None:
        self._t0 = now
        self._p0 = (pose[0], pose[1]) if pose else None
        self._h_ref = (now, -1e9)
        self._move_ref = (now, pose[0] if pose else 0.0, pose[1] if pose else 0.0)

    def _fail(self, msg: str) -> Outputs:
        self.state, self.message = "error", msg
        return Outputs()

    def _advance(self, now: float, pose) -> None:
        self._i += 1
        if self._i >= len(self._steps):
            self.state, self.message = "done", ""
        else:
            self._enter_step(now, pose)

    def progress(self) -> float:
        if not self._steps:
            return 1.0 if self.state == "done" else 0.0
        return min(1.0, self._i / len(self._steps))

    def step_name(self) -> str:
        if self.state == "running" and 0 <= self._i < len(self._steps):
            st = self._steps[self._i]
            return st.get("name", st["type"])
        return ""

    def _load_rect(self):
        c = self.cfg
        half = max(c.fork_width_m, c.load_width_m) / 2 + 0.03
        return [c.front_x_m - c.fork_length_m, c.front_x_m + c.load_overhang_m, -half, half]

    def _insert_rect(self):
        c = self.cfg
        width = max(c.fork_width_m, c.load_width_m if self.carrying else c.fork_width_m)
        half = width / 2 + c.bypass_margin_m
        return [c.front_x_m - c.fork_length_m, c.front_x_m + c.bypass_ahead_m, -half, half]

    # -- main tick ---------------------------------------------------------------
    def tick(self, now: float, hw: Optional[HwState], pose: Optional[Tuple[float, float, float]]) -> Outputs:
        out = Outputs()
        if self.carrying:
            out.bypass.append({"id": "fork_load", "ttl": 0.6, "rects": [self._load_rect()], "cap": False})
        if self.state != "running":
            return out
        if hw is None or now - hw.stamp > 1.0:
            return self._fail("no fork state (/fork_hw_state)")
        if hw.fault:
            return self._fail(f"fork fault code {hw.fault}")
        st = self._steps[self._i]
        if st["type"] == "carry":
            self.carrying = bool(st["value"])
            self._advance(now, pose)
            return out
        if st["type"] == "height":
            if now - self._t0 > self.cfg.height_step_timeout_s:
                return self._fail(f"timeout: {st.get('name')}")
            d, speed, reached = self.hc.command(st["target"], hw)
            if reached:
                self._advance(now, pose)
                return out
            if d == "S":                                   # limit switch before the target
                return self._fail(f"fork limit reached before target ({st.get('name')})")
            # stall detection: commanded to move but the height does not change
            t_ref, h_ref = self._h_ref
            if abs(hw.height_mm - h_ref) > 3.0:
                self._h_ref = (now, hw.height_mm)
            elif now - t_ref > self.cfg.stall_s:
                return self._fail("fork stalled (no height change)")
            out.fork_dir, out.fork_speed = d, speed
            return out
        if st["type"] == "drive":
            if pose is None:
                return self._fail("no odometry")
            if self._p0 is None:
                self._enter_step(now, pose)
            if now - self._t0 > self.cfg.drive_step_timeout_s:
                return self._fail(f"timeout: {st.get('name')}")
            travelled = math.hypot(pose[0] - self._p0[0], pose[1] - self._p0[1])
            if travelled >= st["dist"] - 0.01:
                self._advance(now, pose)
                return out
            speed = self.cfg.insert_speed_mps if st["dir"] > 0 else self.cfg.retract_speed_mps
            if st["dist"] - travelled < 0.05:
                speed = min(speed, 0.04)                  # creep the last 5 cm
            out.lin, out.drive = st["dir"] * speed, True
            # blocked: no real movement for a while (safety zone / obstacle / driver off)
            t_ref, x_ref, y_ref = self._move_ref
            if math.hypot(pose[0] - x_ref, pose[1] - y_ref) > 0.01:
                self._move_ref = (now, pose[0], pose[1])
            elif now - t_ref > self.cfg.blocked_s:
                return self._fail(f"blocked during {st.get('name')} (safety zone or obstacle?)")
            if st.get("bypass") and st["dir"] > 0:
                out.bypass.append({"id": "fork_insert", "ttl": 0.6, "rects": [self._insert_rect()],
                                   "max_linear": self.cfg.insert_speed_mps + 0.02, "max_angular": 0.10})
            return out
        return self._fail("bad step")

    def status(self, hw: Optional[HwState]) -> dict:
        return {"id": self.task_id, "task": self.task_name, "state": self.state, "step": self.step_name(),
                "progress": round(self.progress(), 2), "message": self.message, "carrying": self.carrying,
                "height_mm": None if hw is None else round(hw.height_mm, 1),
                "top": bool(hw and hw.top), "bottom": bool(hw and hw.bottom),
                "cfg": self.cfg.to_dict()}
