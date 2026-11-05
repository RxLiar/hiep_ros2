#!/usr/bin/env python3
"""forklift_node - fork mechanism control + pallet pick / drop sequences.

Command  : /forklift/command  (std_msgs/String JSON)   {"id": "...", "cmd": "pick|drop|lift|lower|insert|retract|stop|reset|set_carrying", ...params}
Status   : /forklift/status   (std_msgs/String JSON, 5 Hz)
Hardware : /fork_hw_cmd (JSON {"dir": "U|D|S", "speed": 0-100}) and /fork_hw_state (JSON {"height_mm", "top", "bottom", "fault"})
           provided by the Mega2560 PLC bridge, or by the built-in simulator (backend: sim).
Motion   : publishes /cmd_vel only while an insert / retract step runs (so it passes through the safety monitor).
Safety   : asks /safety_bypass for a time-limited exception ONLY during the insert step (rectangle ahead of the fork
           tips, speed capped) and while a load is carried (own-load rectangle). The rear zone is never bypassed.
           Emergency stop aborts the task; if this node dies the bypass expires within 0.6 s.
"""
import json
import os
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from hiep_robot2.forklift_logic import ForkConfig, ForkliftCore, HwState


def _yaw(q) -> float:
    import math
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ForkliftNode(Node):
    def __init__(self) -> None:
        super().__init__("forklift_node")
        for name, default in (
            ("backend", "sim"), ("rate_hz", 20.0), ("profile_file", "~/.agv_hmi/robot_profile.json"),
            ("state_file", "~/.agv_hmi/forklift_state.json"),
            ("command_topic", "/forklift/command"), ("status_topic", "/forklift/status"),
            ("hw_cmd_topic", "/fork_hw_cmd"), ("hw_state_topic", "/fork_hw_state"),
            ("odom_topic", "/odom"), ("cmd_vel_topic", "/cmd_vel"), ("bypass_topic", "/safety_bypass"),
            ("estop_topic", "/emergency_stop"), ("config_topic", "/safety_config"),
            ("sim_speed_mm_s", 60.0), ("sim_start_height_mm", 300.0), ("odom_timeout_s", 1.0)):
            self.declare_parameter(name, default)
        gp = lambda n: self.get_parameter(n).value
        self.backend = str(gp("backend")).lower()
        self.state_file = os.path.expanduser(str(gp("state_file")))
        self.odom_timeout = float(gp("odom_timeout_s"))
        self.sim_speed = float(gp("sim_speed_mm_s"))

        self.core = ForkliftCore(self._load_config(str(gp("profile_file"))))
        self.core.carrying = self._load_carrying()
        self._hw: Optional[HwState] = None
        self._pose = None
        self._pose_t = 0.0
        self._last_dir = "S"
        self._last_hw_send = 0.0
        self._last_bypass_send = 0.0
        self._last_status_send = 0.0
        self._last_status_json = ""
        self._sim_h = float(gp("sim_start_height_mm"))
        self._sim_dir, self._sim_speed_pct = "S", 0.0
        self._last_tick = time.monotonic()
        self._withdrawn = True

        self.create_subscription(String, str(gp("command_topic")), self._command_cb, 10)
        self.create_subscription(String, str(gp("hw_state_topic")), self._hw_state_cb, 10)
        self.create_subscription(Odometry, str(gp("odom_topic")), self._odom_cb, 10)
        self.create_subscription(Bool, str(gp("estop_topic")), self._estop_cb, 10)
        self.create_subscription(String, str(gp("config_topic")), self._profile_cb, 10)
        self.vel_pub = self.create_publisher(Twist, str(gp("cmd_vel_topic")), 10)
        self.hw_cmd_pub = self.create_publisher(String, str(gp("hw_cmd_topic")), 10)
        self.hw_state_pub = self.create_publisher(String, str(gp("hw_state_topic")), 10) if self.backend == "sim" else None
        self.bypass_pub = self.create_publisher(String, str(gp("bypass_topic")), 10)
        self.status_pub = self.create_publisher(String, str(gp("status_topic")), 10)
        self.create_timer(1.0 / max(5.0, float(gp("rate_hz"))), self._tick)
        self.get_logger().info(f"Forklift node ready (backend={self.backend}), carrying={self.core.carrying}")

    # -- config ---------------------------------------------------------------
    @staticmethod
    def _config_from_profile(prof: dict) -> ForkConfig:
        d = dict(prof.get("fork", {})) if isinstance(prof, dict) else {}
        if isinstance(prof, dict):
            d.setdefault("front_x_m", float(prof.get("center_x_m", 0.0)) + float(prof.get("length_m", 2.4)) / 2)
            for k in ("fork_length_m", "fork_width_m", "load_width_m", "max_height_mm"):
                if k in prof:
                    d.setdefault(k, prof[k])
        return ForkConfig.from_dict(d)

    def _load_config(self, path: str) -> ForkConfig:
        try:
            with open(os.path.expanduser(path), encoding="utf-8") as f:
                data = json.load(f)
            prof = data.get("profiles", {}).get(data.get("active"), {})
            return self._config_from_profile(prof)
        except (OSError, ValueError, AttributeError):
            return ForkConfig()

    def _profile_cb(self, msg: String) -> None:
        try:
            self.core.cfg = self._config_from_profile(json.loads(msg.data))
            self.core.hc.cfg = self.core.cfg
        except ValueError:
            pass

    def _load_carrying(self) -> bool:
        try:
            with open(self.state_file, encoding="utf-8") as f:
                return bool(json.load(f).get("carrying"))
        except (OSError, ValueError):
            return False

    def _save_carrying(self) -> None:
        try:
            os.makedirs(os.path.dirname(self.state_file), exist_ok=True)
            with open(self.state_file, "w", encoding="utf-8") as f:
                json.dump({"carrying": bool(self.core.carrying)}, f)
        except OSError:
            pass

    # -- inputs -------------------------------------------------------------------
    def _command_cb(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            return
        name = cmd.get("cmd")
        before = self.core.carrying
        if name == "stop":
            self.core.abort("stopped by operator")
            self._stop_everything()
        elif name == "reset":
            self.core.reset()
        else:
            ok, text = self.core.start(time.monotonic(), cmd)
            if not ok:
                self.get_logger().warning(f"Command '{name}' rejected: {text}")
                self.core.state, self.core.message, self.core.task_id = "error", text, str(cmd.get("id", ""))
        if self.core.carrying != before:
            self._save_carrying()
        self._publish_status(force=True)

    def _hw_state_cb(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            self._hw = HwState(float(d.get("height_mm", 0.0)), bool(d.get("top")), bool(d.get("bottom")),
                               int(d.get("fault", 0)), time.monotonic())
        except (ValueError, TypeError):
            pass

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (float(p.position.x), float(p.position.y), _yaw(p.orientation))
        self._pose_t = time.monotonic()

    def _estop_cb(self, msg: Bool) -> None:
        if msg.data and self.core.state == "running":
            self.core.abort("emergency stop")
            self._stop_everything()
            self._publish_status(force=True)

    # -- outputs ------------------------------------------------------------------
    def _send_hw(self, direction: str, speed: float) -> None:
        self.hw_cmd_pub.publish(String(data=json.dumps({"dir": direction, "speed": int(round(speed))})))
        self._last_hw_send = time.monotonic()
        self._sim_dir, self._sim_speed_pct = direction, speed

    def _stop_everything(self) -> None:
        self._send_hw("S", 0)
        self._last_dir = "S"
        for _ in range(3):
            self.vel_pub.publish(Twist())
        self._withdraw_bypass()

    def _withdraw_bypass(self) -> None:
        for rid in ("fork_insert", "fork_load"):
            self.bypass_pub.publish(String(data=json.dumps({"id": rid, "active": False})))

    def _simulate(self, dt: float) -> None:
        if self.backend != "sim":
            return
        if self._sim_dir != "S":
            sign = 1 if self._sim_dir == "U" else -1
            self._sim_h = max(0.0, min(self.core.cfg.max_height_mm,
                                       self._sim_h + sign * self.sim_speed * self._sim_speed_pct / 100.0 * dt))
        hw = {"height_mm": round(self._sim_h, 1), "top": self._sim_h >= self.core.cfg.max_height_mm - 1,
              "bottom": self._sim_h <= 1.0, "fault": 0}
        self.hw_state_pub.publish(String(data=json.dumps(hw)))
        self._hw = HwState(self._sim_h, hw["top"], hw["bottom"], 0, time.monotonic())

    def _tick(self) -> None:
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        self._simulate(dt)
        pose = self._pose if now - self._pose_t <= self.odom_timeout else None
        was_running = self.core.state == "running"
        carrying_before = self.core.carrying
        out = self.core.tick(now, self._hw, pose)
        if out.drive:
            t = Twist()
            t.linear.x = float(out.lin)
            self.vel_pub.publish(t)
        # fork command: change immediately, keep refreshing (the PLC stops the fork if it hears nothing)
        if out.fork_dir != self._last_dir or (out.fork_dir != "S" and now - self._last_hw_send > 0.15):
            self._send_hw(out.fork_dir, out.fork_speed)
            self._last_dir = out.fork_dir
        if was_running and self.core.state in ("error", "done"):
            self._stop_everything()
            if self.core.state == "error":
                self.get_logger().warning(f"Task '{self.core.task_name}' failed: {self.core.message}")
        if now - self._last_bypass_send >= 0.2:
            self._last_bypass_send = now
            for req in out.bypass:
                self.bypass_pub.publish(String(data=json.dumps(req)))
        if self.core.carrying != carrying_before:
            self._save_carrying()
        self._publish_status()

    def _publish_status(self, force: bool = False) -> None:
        now = time.monotonic()
        js = json.dumps(self.core.status(self._hw))
        if force or js != self._last_status_json or now - self._last_status_send >= 0.2:
            self._last_status_json, self._last_status_send = js, now
            self.status_pub.publish(String(data=js))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForkliftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop_everything()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
