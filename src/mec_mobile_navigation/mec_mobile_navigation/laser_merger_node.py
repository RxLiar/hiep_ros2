#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

from message_filters import Subscriber
from message_filters import ApproximateTimeSynchronizer

import tf2_ros
import tf2_sensor_msgs

from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

import numpy as np
import struct


class LaserMerger(Node):

    def __init__(self):

        super().__init__('laser_merger')

        self.target_frame = 'base_footprint'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.projector = LaserProjection()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        self.front_sub = Subscriber(
            self,
            LaserScan,
            '/front_lidar/scan',
            qos_profile=qos
        )

        self.rear_sub = Subscriber(
            self,
            LaserScan,
            '/rear_lidar/scan',
            qos_profile=qos
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.front_sub, self.rear_sub],
            queue_size=20,
            slop=0.03
        )

        self.sync.registerCallback(
            self.synced_scan_callback
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.get_logger().info(
            'Laser merger started'
        )

    def synced_scan_callback(
        self,
        front_scan,
        rear_scan
    ):

        front_time = (
            front_scan.header.stamp.sec +
            front_scan.header.stamp.nanosec * 1e-9
        )

        rear_time = (
            rear_scan.header.stamp.sec +
            rear_scan.header.stamp.nanosec * 1e-9
        )

        dt = abs(front_time - rear_time)

        self.get_logger().debug(
            f'Sync dt = {dt:.6f}s'
        )

        try:

            merged = self.merge_scans(
                front_scan,
                rear_scan
            )

            if merged is not None:
                self.pub.publish(merged)

        except Exception as e:

            self.get_logger().error(
                f'Merge failed: {str(e)}'
            )

    def merge_scans(
        self,
        front_scan,
        rear_scan
    ):

        angle_min = -np.pi
        angle_max = np.pi

        angle_increment = front_scan.angle_increment

        num_samples = int(
            (angle_max - angle_min)
            / angle_increment
        )

        merged_ranges = np.full(
            num_samples,
            np.inf,
            dtype=np.float32
        )

        scans = [
            front_scan,
            rear_scan
        ]

        for scan in scans:

            try:

                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    scan.header.frame_id,
                    scan.header.stamp,
                    timeout=Duration(seconds=0.2)
                )

                cloud = self.projector.projectLaser(
                    scan
                )

                cloud_tf = (
                    tf2_sensor_msgs.do_transform_cloud(
                        cloud,
                        transform
                    )
                )

                points = self.cloud_to_xy(
                    cloud_tf
                )

                for x, y in points:

                    dist = np.hypot(x, y)

                    if dist < scan.range_min:
                        continue

                    if dist > scan.range_max:
                        continue

                    angle = np.arctan2(y, x)

                    idx = int(
                        (angle - angle_min)
                        / angle_increment
                    )

                    if (
                        idx < 0
                        or idx >= num_samples
                    ):
                        continue

                    if dist < merged_ranges[idx]:
                        merged_ranges[idx] = dist

            except Exception as e:

                self.get_logger().warn(
                    f'TF error {scan.header.frame_id}: {e}'
                )

        output = LaserScan()

        output.header.stamp = front_scan.header.stamp
        output.header.frame_id = self.target_frame

        output.angle_min = angle_min
        output.angle_max = angle_max
        output.angle_increment = angle_increment

        output.time_increment = 0.0
        output.scan_time = front_scan.scan_time

        output.range_min = front_scan.range_min
        output.range_max = front_scan.range_max

        output.ranges = merged_ranges.tolist()

        return output

    def cloud_to_xy(
        self,
        cloud_msg
    ):

        points = []

        point_step = cloud_msg.point_step
        data = cloud_msg.data

        for i in range(cloud_msg.width):

            offset = i * point_step

            x = struct.unpack_from(
                'f',
                data,
                offset
            )[0]

            y = struct.unpack_from(
                'f',
                data,
                offset + 4
            )[0]

            if (
                np.isnan(x)
                or np.isnan(y)
            ):
                continue

            points.append(
                (x, y)
            )

        return points


def main():

    rclpy.init()

    node = LaserMerger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()