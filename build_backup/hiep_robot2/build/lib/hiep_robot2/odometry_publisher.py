#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
import math
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_base', 0.2)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            String,
            'encoder_data',
            self.encoder_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('OdometryPublisher started')

    def encoder_callback(self, msg):
        try:
            left_ticks, right_ticks = map(int, msg.data.split(','))

            # Giả sử mỗi vòng encoder là 360 ticks, tùy theo ESP32
            ticks_per_rev = 360
            distance_per_tick = 2 * math.pi * self.wheel_radius / ticks_per_rev

            left_distance = left_ticks * distance_per_tick
            right_distance = right_ticks * distance_per_tick

            dt = (self.get_clock().now() - self.last_time).nanoseconds * 1e-9
            self.last_time = self.get_clock().now()

            if dt == 0:
                return

            v = (right_distance + left_distance) / 2.0 / dt
            w = (right_distance - left_distance) / self.wheel_base / dt

            # Tính toán vị trí mới
            delta_x = v * math.cos(self.theta) * dt
            delta_y = v * math.sin(self.theta) * dt
            delta_theta = w * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # Quaternion từ yaw
            q = quaternion_from_yaw(self.theta)

            # Gửi TF
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

            # Gửi nav_msgs/Odometry
            odom = Odometry()
            odom.header.stamp = t.header.stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = q
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = w

            self.odom_pub.publish(odom)

        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý encoder data: {e}')


def quaternion_from_yaw(yaw):
    """Chuyển từ góc yaw sang quaternion."""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
