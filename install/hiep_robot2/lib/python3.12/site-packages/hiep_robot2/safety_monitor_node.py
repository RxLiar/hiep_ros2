#!/usr/bin/env python3
"""safety_monitor_node - lidar based slow / stop gate between /cmd_vel and the motor bridge.

    /cmd_vel (Nav2, HMI joystick, ...)  ->  [ safety_monitor ]  ->  /cmd_vel_safe  ->  KEYA / ESP32 bridge

* Reads the robot profile (size, lidar pose, stop / slow zones) from the HMI's
  robot_profile.json at start-up and live from /safety_config (JSON, latched).
* Points of /scan inside the SLOW rectangle limit the speed, points inside the
  STOP rectangle block motion towards them.  Returns from the robot's own body
  are ignored.
* Fail safe: no /scan (when require_scan) or a stale scan => output is zero.
  If this node dies the bridge stops receiving /cmd_vel_safe and stops the robot.
* Publishes /safety_state (JSON, 10 Hz) for the HMI and /robot_profile (latched).
* Optionally pushes the footprint to the Nav2 costmaps (dynamic parameter).

This is a SOFTWARE layer. It does not replace a safety-rated lidar / PLC e-stop.
"""
import json
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from hiep_robot2.safety_logic import (
    Debouncer, Profile, classify, gate, load_active_profile, scan_to_base_points, state_name)

COSTMAP_NODES = ("/local_costmap/local_costmap", "/global_costmap/global_costmap")


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor")
        self.declare_parameter("profile_file", "~/.agv_hmi/robot_profile.json")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self.declare_parameter("config_topic", "/safety_config")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("scan_timeout_s", 0.5)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("hold_s", 0.3)
        self.declare_parameter("block_rotation_in_stop", True)
        self.declare_parameter("apply_nav2_footprint", True)

        gp = lambda n: self.get_parameter(n).value
        self.profile: Profile = load_active_profile(str(gp("profile_file")))
        self.scan_timeout = float(gp("scan_timeout_s"))
        self.cmd_timeout = float(gp("cmd_timeout_s"))
        self.block_rot = bool(gp("block_rotation_in_stop"))
        self.apply_footprint = bool(gp("apply_nav2_footprint"))
        self.debounce = Debouncer(float(gp("hold_s")))

        self._cmd = (0.0, 0.0)
        self._cmd_t = 0.0
        self._scan_t: Optional[float] = None
        self._flags = {"stop_front": False, "stop_rear": False, "slow_front": False, "slow_rear": False}
        self._nearest = None
        self._counts = {}
        self._applied_fp = {}

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, str(gp("scan_topic")), self._scan_cb, sensor)
        self.create_subscription(Twist, str(gp("input_topic")), self._cmd_cb, 10)
        self.create_subscription(String, str(gp("config_topic")), self._config_cb, latched)
        self.out_pub = self.create_publisher(Twist, str(gp("output_topic")), 10)
        self.state_pub = self.create_publisher(String, "/safety_state", 10)
        self.profile_pub = self.create_publisher(String, "/robot_profile", latched)
        self._clients = {n: self.create_client(SetParameters, f"{n}/set_parameters") for n in COSTMAP_NODES}

        self.create_timer(1.0 / max(5.0, float(gp("rate_hz"))), self._tick)
        self.create_timer(0.1, self._publish_state)
        self.create_timer(3.0, self._apply_nav2_footprint)
        self._publish_profile()
        self.get_logger().info(
            f"Safety monitor ready: {gp('input_topic')} -> {gp('output_topic')}, profile='{self.profile.name}' "
            f"{self.profile.length_m:.2f}x{self.profile.width_m:.2f} m, enabled={self.profile.safety_enabled}")

    # -- inputs ---------------------------------------------------------------
    def _cmd_cb(self, msg: Twist) -> None:
        self._cmd = (float(msg.linear.x), float(msg.angular.z))
        self._cmd_t = time.monotonic()

    def _config_cb(self, msg: String) -> None:
        try:
            new = Profile.from_dict(json.loads(msg.data))
        except ValueError:
            self.get_logger().warning("Ignoring invalid /safety_config JSON")
            return
        self.profile = new
        self._applied_fp = {}
        self._publish_profile()
        self.get_logger().info(
            f"Profile updated: '{new.name}' {new.length_m:.2f}x{new.width_m:.2f} m, "
            f"stop={new.stop}, slow={new.slow}, enabled={new.safety_enabled}")

    def _scan_cb(self, msg: LaserScan) -> None:
        now = time.monotonic()
        self._scan_t = now
        pts = scan_to_base_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                  msg.range_min, msg.range_max, self.profile)
        res = classify(pts, self.profile)
        counts = {"stop_front": res.stop_front, "stop_rear": res.stop_rear,
                  "slow_front": res.slow_front, "slow_rear": res.slow_rear}
        self._counts = counts
        self._flags = self.debounce.update(now, counts, self.profile.min_points)
        self._nearest = res

    # -- output ---------------------------------------------------------------
    def _scan_age(self) -> float:
        return float("inf") if self._scan_t is None else time.monotonic() - self._scan_t

    def _mode(self) -> str:
        if not self.profile.safety_enabled:
            return "disabled"
        if self.profile.require_scan and self._scan_age() > self.scan_timeout:
            return "no_scan"
        return state_name(self._flags)

    def _tick(self) -> None:
        lin, ang = self._cmd
        if time.monotonic() - self._cmd_t > self.cmd_timeout:
            lin, ang = 0.0, 0.0
        mode = self._mode()
        if mode == "no_scan":
            lin, ang = 0.0, 0.0
        elif mode != "disabled":
            flags = self._flags if self._scan_age() <= self.scan_timeout else {}
            lin, ang = gate(lin, ang, flags, self.profile, self.block_rot)
        out = Twist()
        out.linear.x, out.angular.z = float(lin), float(ang)
        self.out_pub.publish(out)

    def _publish_state(self) -> None:
        near = self._nearest
        age = self._scan_age()
        state = {
            "state": self._mode(),
            "profile": self.profile.name,
            "nearest_m": None if near is None or near.nearest_m is None else round(near.nearest_m, 3),
            "nearest_xy": None if near is None or near.nearest_xy is None else [round(v, 3) for v in near.nearest_xy],
            "counts": self._counts,
            "scan_age_s": None if math.isinf(age) else round(age, 2),
            "enabled": self.profile.safety_enabled,
        }
        self.state_pub.publish(String(data=json.dumps(state)))

    def _publish_profile(self) -> None:
        self.profile_pub.publish(String(data=json.dumps(self.profile.to_dict())))

    # -- Nav2 footprint -------------------------------------------------------
    def _apply_nav2_footprint(self) -> None:
        if not self.apply_footprint:
            return
        fp = self.profile.nav2_footprint()
        for name, cli in self._clients.items():
            if self._applied_fp.get(name) == fp or not cli.service_is_ready():
                continue
            req = SetParameters.Request()
            req.parameters = [Parameter(name="footprint", value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=fp))]
            fut = cli.call_async(req)
            fut.add_done_callback(lambda f, n=name, v=fp: self._footprint_done(n, v, f))

    def _footprint_done(self, name: str, fp: str, fut) -> None:
        try:
            res = fut.result().results[0]
        except Exception as exc:                      # service vanished, etc.
            self.get_logger().warning(f"Footprint update on {name} failed: {exc}")
            return
        if res.successful:
            self._applied_fp[name] = fp
            self.get_logger().info(f"Nav2 footprint on {name} set to {fp}")
        else:
            self.get_logger().warning(f"{name} rejected footprint: {res.reason}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
