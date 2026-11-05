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
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from hiep_robot2.safety_logic import (
    BypassManager, Debouncer, Profile, cap_speed, classify, gate, load_active_profile,
    scan_to_base_points, state_name)

COSTMAP_NODES = ("/local_costmap/local_costmap", "/global_costmap/global_costmap")


class SafetyMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor")
        self.declare_parameter("profile_file", "~/.agv_hmi/robot_profile.json")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self.declare_parameter("config_topic", "/safety_config")
        self.declare_parameter("bypass_topic", "/safety_bypass")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("scan_timeout_s", 0.5)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("hold_s", 0.3)
        self.declare_parameter("block_rotation_in_stop", True)
        self.declare_parameter("apply_nav2_footprint", True)
        # Where the lidar is: by default asked from TF (scan frame -> base_frame), so a merged scan or a moved
        # lidar cannot be wrong because of a number typed in the HMI. The profile lidar pose is the fallback.
        self.declare_parameter("use_tf", True)
        self.declare_parameter("base_frame", "base_footprint")

        gp = lambda n: self.get_parameter(n).value
        self.profile: Profile = load_active_profile(str(gp("profile_file")))
        self.scan_timeout = float(gp("scan_timeout_s"))
        self.cmd_timeout = float(gp("cmd_timeout_s"))
        self.block_rot = bool(gp("block_rotation_in_stop"))
        self.apply_footprint = bool(gp("apply_nav2_footprint"))
        self.base_frame = str(gp("base_frame"))
        self.input_topic, self.output_topic = str(gp("input_topic")), str(gp("output_topic"))
        self._tf_buffer = None
        self._tf_error = None
        if bool(gp("use_tf")):
            try:
                from tf2_ros import Buffer, TransformListener
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self)
            except Exception as exc:                    # no tf2: use the manual lidar pose
                self._tf_error = f"tf2 unavailable: {exc}"
        self._scan_frame = ""
        self._pose_info = (self.profile.lidar_x_m, self.profile.lidar_y_m, self.profile.lidar_yaw_deg, "profile")
        self._cmd_in = (0.0, 0.0)
        self._cmd_out = (0.0, 0.0)
        self._out_subs = 0
        self._direct_consumers = []
        self.debounce = Debouncer(float(gp("hold_s")))
        self.bypass = BypassManager()
        self._last_scan = None
        self._bypass_seen = False

        self._cmd = (0.0, 0.0)
        self._cmd_t = 0.0
        self._scan_t: Optional[float] = None
        self._flags = {"stop_front": False, "stop_rear": False, "stop_side": False,
                       "slow_front": False, "slow_rear": False, "slow_side": False}
        self._nearest = None
        self._counts = {}
        self._applied_fp = {}

        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(LaserScan, str(gp("scan_topic")), self._scan_cb, sensor)
        self.create_subscription(Twist, str(gp("input_topic")), self._cmd_cb, 10)
        self.create_subscription(String, str(gp("config_topic")), self._config_cb, latched)
        self.create_subscription(String, str(gp("bypass_topic")), self._bypass_cb, 10)
        self.out_pub = self.create_publisher(Twist, str(gp("output_topic")), 10)
        self.state_pub = self.create_publisher(String, "/safety_state", 10)
        self.profile_pub = self.create_publisher(String, "/robot_profile", latched)
        self._clients = {n: self.create_client(SetParameters, f"{n}/set_parameters") for n in COSTMAP_NODES}

        self.create_timer(1.0 / max(5.0, float(gp("rate_hz"))), self._tick)
        self.create_timer(0.1, self._publish_state)
        self.create_timer(3.0, self._apply_nav2_footprint)
        self.create_timer(1.0, self._refresh_graph)
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

    def _bypass_cb(self, msg: String) -> None:
        try:
            self.bypass.update(time.monotonic(), json.loads(msg.data))
        except ValueError:
            self.get_logger().warning("Ignoring invalid /safety_bypass JSON")

    @staticmethod
    def _counts_of(res) -> dict:
        return {"stop_front": res.stop_front, "stop_rear": res.stop_rear, "stop_side": res.stop_side,
                "slow_front": res.slow_front, "slow_rear": res.slow_rear, "slow_side": res.slow_side}

    def _scan_pose(self, frame: str):
        """(x, y, yaw_rad, source): pose of the scan frame in the base frame."""
        if self._tf_buffer is not None:
            if frame == self.base_frame:
                return 0.0, 0.0, 0.0, "same_frame"
            try:
                tf = self._tf_buffer.lookup_transform(self.base_frame, frame, Time())
                t, q = tf.transform.translation, tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self._tf_error = None
                return float(t.x), float(t.y), yaw, "tf"
            except Exception as exc:
                self._tf_error = str(exc)[:140]
        return (self.profile.lidar_x_m, self.profile.lidar_y_m,
                math.radians(self.profile.lidar_yaw_deg), "profile")

    def _refresh_graph(self) -> None:
        """Who listens to what: is anything still driving the motors straight from /cmd_vel?"""
        try:
            infos = self.get_subscriptions_info_by_topic(self.input_topic)
            self._direct_consumers = sorted({i.node_name for i in infos if i.node_name != "safety_monitor"})
        except Exception:
            self._direct_consumers = None
        try:
            self._out_subs = int(self.out_pub.get_subscription_count())
        except Exception:
            self._out_subs = None

    def _scan_cb(self, msg: LaserScan) -> None:
        now = time.monotonic()
        self._scan_t = now
        x, y, yaw, source = self._scan_pose(str(msg.header.frame_id))
        self._scan_frame = str(msg.header.frame_id)
        self._pose_info = (round(x, 3), round(y, 3), round(math.degrees(yaw), 1), source)
        pts = scan_to_base_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                  msg.range_min, msg.range_max, self.profile, pose=(x, y, yaw))
        self._last_scan = pts
        res = classify(pts, self.profile, ignore_rects=self.bypass.rects(now))
        counts = self._counts_of(res)
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

    def _reclassify(self) -> None:
        """Zones depend on the bypass rectangles, which can change between scans."""
        if self._last_scan is None:
            return
        now = time.monotonic()
        res = classify(self._last_scan, self.profile, ignore_rects=self.bypass.rects(now))
        counts = self._counts_of(res)
        self._counts = counts
        self._nearest = res
        if self.bypass.rects(now):
            # while bypassing, do not keep a zone 'held' by the debouncer from before the bypass began
            self.debounce = Debouncer(self.debounce.hold_s) if not self._bypass_seen else self.debounce
        self._bypass_seen = bool(self.bypass.rects(now))
        self._flags = self.debounce.update(now, counts, self.profile.min_points)

    def _tick(self) -> None:
        self._reclassify()
        lin, ang = self._cmd
        if time.monotonic() - self._cmd_t > self.cmd_timeout:
            lin, ang = 0.0, 0.0
        self._cmd_in = (lin, ang)
        mode = self._mode()
        if mode == "no_scan":
            lin, ang = 0.0, 0.0
        elif mode != "disabled":
            flags = self._flags if self._scan_age() <= self.scan_timeout else {}
            lin, ang = gate(lin, ang, flags, self.profile, self.block_rot)
            lin, ang = cap_speed(lin, ang, self.bypass.speed_caps(time.monotonic()))
        out = Twist()
        out.linear.x, out.angular.z = float(lin), float(ang)
        self._cmd_out = (float(lin), float(ang))
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
            "bypass": sorted(self.bypass.active(time.monotonic()).keys()),
            # --- chain diagnostics (shown by the HMI "Robot & Safety" page) ---
            "profile_sig": self.profile.signature(),
            "cmd_in": [round(self._cmd_in[0], 3), round(self._cmd_in[1], 3)],
            "cmd_out": [round(self._cmd_out[0], 3), round(self._cmd_out[1], 3)],
            "scan_frame": self._scan_frame,
            "pose_source": self._pose_info[3],
            "lidar_pose": [self._pose_info[0], self._pose_info[1], self._pose_info[2]],
            "tf_error": self._tf_error,
            "base_frame": self.base_frame,
            "input_topic": self.input_topic,
            "output_topic": self.output_topic,
            "out_subscribers": self._out_subs,
            "direct_consumers": self._direct_consumers,
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
