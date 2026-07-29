"""Fleet Server: aggregate RobotState messages and publish one fleet snapshot."""
from __future__ import annotations

import json
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .protocol import RobotState, make_event


@dataclass
class _TrackedRobot:
    state: RobotState
    received_monotonic: float
    last_health: str = "online"


class FleetServer(Node):
    def __init__(self) -> None:
        super().__init__("fleet_server")
        self.declare_parameter("snapshot_hz", 2.0)
        self.declare_parameter("warning_timeout_sec", 1.5)
        self.declare_parameter("offline_timeout_sec", 3.0)

        self.warning_timeout = max(0.2, float(self.get_parameter("warning_timeout_sec").value))
        self.offline_timeout = max(
            self.warning_timeout + 0.1,
            float(self.get_parameter("offline_timeout_sec").value),
        )
        self._robots: dict[str, _TrackedRobot] = {}

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        snapshot_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(String, "/fleet/robot_state", self._state_cb, input_qos)
        self._snapshot_pub = self.create_publisher(String, "/fleet/snapshot", snapshot_qos)
        self._event_pub = self.create_publisher(String, "/fleet/events", 20)

        hz = max(0.5, min(20.0, float(self.get_parameter("snapshot_hz").value)))
        self.create_timer(1.0 / hz, self._publish_snapshot)
        self.get_logger().info(
            f"Fleet Server started: warning={self.warning_timeout:.1f}s, "
            f"offline={self.offline_timeout:.1f}s"
        )

    def _state_cb(self, msg: String) -> None:
        try:
            state = RobotState.from_json(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid /fleet/robot_state payload: {exc}")
            return

        now = time.monotonic()
        previous = self._robots.get(state.robot_id)
        self._robots[state.robot_id] = _TrackedRobot(
            state=state,
            received_monotonic=now,
            last_health=previous.last_health if previous else "online",
        )
        if previous is None:
            self.get_logger().info(
                f"Robot discovered: {state.robot_id} on {state.map_id} v{state.map_version}"
            )
            self._publish_event("robot_discovered", state.robot_id, "Robot connected to fleet")

    def _health(self, tracked: _TrackedRobot, now: float) -> tuple[str, float]:
        age = max(0.0, now - tracked.received_monotonic)
        if age > self.offline_timeout:
            return "offline", age
        if age > self.warning_timeout:
            return "stale", age
        return "online", age

    def _publish_event(self, event_type: str, robot_id: str, message: str, **extra) -> None:
        event = String()
        event.data = make_event(event_type, robot_id, message, **extra)
        self._event_pub.publish(event)

    def _publish_snapshot(self) -> None:
        now_mono = time.monotonic()
        robots = []
        summary = {
            "total": 0,
            "online": 0,
            "stale": 0,
            "offline": 0,
            "moving": 0,
            "errors": 0,
            "warnings": 0,
        }

        for robot_id in sorted(self._robots):
            tracked = self._robots[robot_id]
            health, age = self._health(tracked, now_mono)
            if health != tracked.last_health:
                self._publish_event(
                    f"robot_{health}",
                    robot_id,
                    f"Robot state changed to {health}",
                    age_sec=round(age, 3),
                )
                tracked.last_health = health

            state = tracked.state.to_dict()
            state["fleet_health"] = health
            state["last_seen_sec"] = round(age, 3)
            if health == "offline":
                state["connection"] = "offline"
            robots.append(state)

            summary["total"] += 1
            summary[health] += 1
            summary["moving"] += int(bool(state.get("moving")) and health != "offline")
            summary["errors"] += int(state.get("error_level") == "error")
            summary["warnings"] += int(state.get("error_level") == "warn")

        snapshot = {
            "schema": "agv_fleet_snapshot_v1",
            "server_time": time.time(),
            "summary": summary,
            "maps": sorted({robot["map_id"] for robot in robots}),
            "robots": robots,
        }
        msg = String()
        msg.data = json.dumps(snapshot, ensure_ascii=False, separators=(",", ":"))
        self._snapshot_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
