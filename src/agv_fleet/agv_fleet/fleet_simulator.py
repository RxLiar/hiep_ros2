"""Simple moving RobotState publisher for testing two computers without Nav2."""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .protocol import RobotState, validate_map_id, validate_robot_id


class FleetRobotSimulator(Node):
    def __init__(self) -> None:
        super().__init__("fleet_robot_simulator")
        self.declare_parameter("robot_id", "AGV001")
        self.declare_parameter("robot_name", "Simulated AGV 01")
        self.declare_parameter("map_id", "M02101")
        self.declare_parameter("map_version", 1)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("radius", 2.0)
        self.declare_parameter("angular_speed", 0.18)
        self.declare_parameter("battery_start", 95)
        self.declare_parameter("publish_hz", 5.0)

        self.robot_id = validate_robot_id(self.get_parameter("robot_id").value)
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.map_id = validate_map_id(self.get_parameter("map_id").value)
        self.map_version = int(self.get_parameter("map_version").value)
        self.center_x = float(self.get_parameter("center_x").value)
        self.center_y = float(self.get_parameter("center_y").value)
        self.radius = max(0.1, float(self.get_parameter("radius").value))
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.battery_start = max(1, min(100, int(self.get_parameter("battery_start").value)))
        self.started = time.monotonic()
        self.sequence = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.publisher = self.create_publisher(String, "/fleet/robot_state", qos)
        hz = max(0.5, min(20.0, float(self.get_parameter("publish_hz").value)))
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f"Fleet simulator started: {self.robot_id}, shared map {self.map_id} v{self.map_version}"
        )

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.started
        angle = self.angular_speed * elapsed
        x = self.center_x + self.radius * math.cos(angle)
        y = self.center_y + self.radius * math.sin(angle)
        yaw = angle + math.pi / 2.0
        battery = max(10, self.battery_start - int(elapsed / 90.0))
        self.sequence += 1

        state = RobotState(
            robot_id=self.robot_id,
            robot_name=self.robot_name,
            map_id=self.map_id,
            map_version=self.map_version,
            x=x,
            y=y,
            yaw=yaw,
            linear_velocity=abs(self.angular_speed * self.radius),
            angular_velocity=self.angular_speed,
            battery_percent=battery,
            connection="online",
            moving=True,
            nav_state="executing",
            mission_id=f"SIM-{self.robot_id}",
            route_id="R-DEMO",
            route_name="Fleet communication test",
            waypoint_current=(int(elapsed / 8.0) % 5) + 1,
            waypoint_total=5,
            sequence=self.sequence,
            stamp=time.time(),
        )
        msg = String()
        msg.data = state.to_json()
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetRobotSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
