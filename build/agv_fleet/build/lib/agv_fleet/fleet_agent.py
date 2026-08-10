"""Fleet Agent: collect one AGV's local ROS topics and publish compact state.

The active map is updated dynamically from ``/fleet/map_context``. Therefore
``map_id`` and ``map_version`` no longer need to be fixed in the launch command.
"""
from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String

from .protocol import RobotState, make_event, validate_map_id, validate_robot_id


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _goal_status_text(msg: GoalStatusArray) -> str:
    if not msg.status_list:
        return "idle"
    status = int(msg.status_list[-1].status)
    return {
        0: "unknown",
        1: "accepted",
        2: "executing",
        3: "canceling",
        4: "succeeded",
        5: "canceled",
        6: "aborted",
    }.get(status, "unknown")


class FleetAgent(Node):
    def __init__(self) -> None:
        super().__init__("fleet_agent")

        self.declare_parameter("robot_id", "AGV001")
        self.declare_parameter("robot_name", "Busan AGV 01")
        # Backward-compatible optional initial map. Route context overrides it.
        self.declare_parameter("map_id", "")
        self.declare_parameter("map_version", 0)
        self.declare_parameter("source_namespace", "")
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("assume_online", True)
        self.declare_parameter("persist_map_context", True)

        self.robot_id = validate_robot_id(self.get_parameter("robot_id").value)
        self.robot_name = str(self.get_parameter("robot_name").value or self.robot_id)
        initial_map_id = validate_map_id(
            self.get_parameter("map_id").value, allow_empty=True
        )
        initial_map_version = int(self.get_parameter("map_version").value or 0)
        if initial_map_id:
            initial_map_version = max(1, initial_map_version)
        else:
            initial_map_version = 0
        self.source_namespace = str(
            self.get_parameter("source_namespace").value or ""
        ).strip("/")
        self.assume_online = bool(self.get_parameter("assume_online").value)
        self.persist_map_context = bool(
            self.get_parameter("persist_map_context").value
        )

        self._context_path = (
            Path.home()
            / ".agv_hmi"
            / "fleet_agent"
            / f"{self.robot_id}_map_context.json"
        )

        self.state = RobotState(
            robot_id=self.robot_id,
            robot_name=self.robot_name,
            map_id=initial_map_id,
            map_version=initial_map_version,
            connection="online" if self.assume_online else "offline",
        )
        self._sequence = 0
        if not initial_map_id:
            self._load_persisted_context()

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        context_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._state_pub = self.create_publisher(String, "/fleet/robot_state", state_qos)
        self._event_pub = self.create_publisher(String, "/fleet/events", 20)
        self._map_context_sub = self.create_subscription(
            String,
            "/fleet/map_context",
            self._map_context_cb,
            context_qos,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            self._topic("amcl_pose"),
            self._pose_cb,
            10,
        )
        self.create_subscription(
            Odometry,
            self._topic("odometry/filtered"),
            self._odom_cb,
            10,
        )
        self.create_subscription(Int32, self._topic("battery_status"), self._battery_cb, 10)
        self.create_subscription(String, self._topic("connection_status"), self._connection_cb, 10)
        self.create_subscription(String, self._topic("robot_status"), self._robot_status_cb, 10)
        self.create_subscription(Bool, self._topic("agv_status"), self._moving_cb, 10)
        self.create_subscription(
            GoalStatusArray,
            self._topic("navigate_to_pose/_action/status"),
            self._nav_status_cb,
            10,
        )

        hz = max(0.5, min(20.0, float(self.get_parameter("publish_hz").value)))
        self.create_timer(1.0 / hz, self._publish_state)
        map_label = (
            f"{self.state.map_id} v{self.state.map_version}"
            if self.state.map_id
            else "waiting for route map context"
        )
        self.get_logger().info(
            f"Fleet Agent started: robot={self.robot_id}, map={map_label}, "
            f"source_namespace=/{self.source_namespace or '(root)'}"
        )

    def _topic(self, suffix: str) -> str:
        suffix = suffix.strip("/")
        return f"/{self.source_namespace}/{suffix}" if self.source_namespace else f"/{suffix}"

    def _load_persisted_context(self) -> None:
        if not self.persist_map_context or not self._context_path.is_file():
            return
        try:
            data = json.loads(self._context_path.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                return
            map_id = validate_map_id(data.get("map_id", ""), allow_empty=True)
            if not map_id:
                return
            self.state.map_id = map_id
            self.state.map_version = max(1, int(data.get("map_version", 1)))
            self.state.map_checksum = str(data.get("map_checksum", ""))
            self.state.map_path = str(data.get("map_path", ""))
            self.state.route_id = str(data.get("route_id", ""))
            self.state.route_name = str(data.get("route_name", ""))
        except Exception as exc:
            self.get_logger().warn(f"Cannot load persisted map context: {exc}")

    def _persist_context(self, data: dict) -> None:
        if not self.persist_map_context:
            return
        try:
            self._context_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._context_path.with_suffix(".tmp")
            tmp.write_text(
                json.dumps(data, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            os.replace(tmp, self._context_path)
        except Exception as exc:
            self.get_logger().warn(f"Cannot persist map context: {exc}")

    def _map_context_cb(self, msg: String) -> None:
        try:
            data = json.loads(str(msg.data or "{}"))
            if not isinstance(data, dict):
                raise ValueError("map context must be a JSON object")

            target_robot_id = str(
                data.get("robot_id", data.get("target_robot_id", ""))
            ).strip().upper()
            if target_robot_id and target_robot_id != self.robot_id:
                return

            map_id = validate_map_id(data.get("map_id", ""))
            map_version = max(1, int(data.get("map_version", 1)))
            old_map = (self.state.map_id, self.state.map_version)

            self.state.map_id = map_id
            self.state.map_version = map_version
            self.state.map_checksum = str(data.get("map_checksum", "")).strip().lower()
            self.state.map_path = str(data.get("map_path", "")).strip()
            self.state.route_id = str(data.get("route_id", "")).strip()
            self.state.route_name = str(data.get("route_name", "")).strip()
            self.state.mission_id = str(data.get("mission_id", "")).strip()
            self.state.waypoint_current = max(
                0, int(data.get("waypoint_current", 0) or 0)
            )
            self.state.waypoint_total = max(
                0, int(data.get("waypoint_total", 0) or 0)
            )

            persisted = {
                "robot_id": self.robot_id,
                "map_id": self.state.map_id,
                "map_version": self.state.map_version,
                "map_checksum": self.state.map_checksum,
                "map_path": self.state.map_path,
                "route_id": self.state.route_id,
                "route_name": self.state.route_name,
                "updated_at": time.time(),
            }
            self._persist_context(persisted)

            if old_map != (map_id, map_version):
                self.get_logger().info(
                    f"Map context updated: {map_id} v{map_version}, "
                    f"route={self.state.route_name or self.state.route_id or '-'}"
                )
                event = String()
                event.data = make_event(
                    "map_context_updated",
                    self.robot_id,
                    f"Map changed to {map_id} v{map_version}",
                    map_id=map_id,
                    map_version=map_version,
                    route_id=self.state.route_id,
                )
                self._event_pub.publish(event)
        except Exception as exc:
            self.get_logger().warn(f"Invalid /fleet/map_context payload: {exc}")

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self.state.x = float(pose.position.x)
        self.state.y = float(pose.position.y)
        self.state.yaw = float(_yaw_from_quaternion(pose.orientation))

    def _odom_cb(self, msg: Odometry) -> None:
        self.state.linear_velocity = float(msg.twist.twist.linear.x)
        self.state.angular_velocity = float(msg.twist.twist.angular.z)
        self.state.moving = (
            abs(self.state.linear_velocity) > 0.01
            or abs(self.state.angular_velocity) > 0.01
        )

    def _battery_cb(self, msg: Int32) -> None:
        self.state.battery_percent = max(0, min(100, int(msg.data)))

    def _connection_cb(self, msg: String) -> None:
        value = str(msg.data or "").strip().lower()
        self.state.connection = value if value in {"online", "idle", "offline"} else "offline"

    def _robot_status_cb(self, msg: String) -> None:
        text = str(msg.data or "").strip()
        if not text:
            self.state.error_level = "ok"
            self.state.error_message = ""
            return
        try:
            payload = json.loads(text)
            if isinstance(payload, dict):
                level = str(payload.get("level", payload.get("severity", "error"))).lower()
                message = str(payload.get("message", payload.get("msg", text)))
                self.state.error_level = "warn" if level in {"warn", "warning"} else (
                    "ok" if level in {"ok", "normal", "none"} else "error"
                )
                self.state.error_message = "" if self.state.error_level == "ok" else message
                return
        except Exception:
            pass
        upper = text.upper()
        if upper in {"OK", "NORMAL", "NONE", "CLEAR", "NO ERROR", "NO ERRORS"}:
            self.state.error_level = "ok"
            self.state.error_message = ""
        elif upper.startswith("WARN"):
            self.state.error_level = "warn"
            self.state.error_message = text
        else:
            self.state.error_level = "error"
            self.state.error_message = text

    def _moving_cb(self, msg: Bool) -> None:
        self.state.moving = bool(msg.data)

    def _nav_status_cb(self, msg: GoalStatusArray) -> None:
        self.state.nav_state = _goal_status_text(msg)

    def _publish_state(self) -> None:
        self._sequence += 1
        self.state.sequence = self._sequence
        self.state.stamp = time.time()
        if self.assume_online and self.state.connection == "offline":
            self.state.connection = "online"
        msg = String()
        msg.data = self.state.to_json()
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
