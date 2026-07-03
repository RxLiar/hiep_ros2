#!/usr/bin/env python3
"""
debug_tf_clock.py — Chạy độc lập để tìm nguyên nhân exact=0

Chạy:
  python3 debug_tf_clock.py

In ra:
  - scan.header.stamp (giây)
  - TF map->base_footprint stamp mới nhất trong buffer
  - Độ lệch (delta) giữa hai timestamp
  - Có use_sim_time không

Nếu delta > 0.1s → clock mismatch (sim time vs wall time).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class TfClockDebugger(Node):
    def __init__(self):
        super().__init__("tf_clock_debugger")
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._count = 0

        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, 10)

        # Kiểm tra use_sim_time
        use_sim = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"use_sim_time = {use_sim}")
        self.get_logger().info("Waiting for /scan and TF...")

    def _scan_cb(self, msg: LaserScan):
        self._count += 1
        if self._count % 10 != 1:   # chỉ in mỗi 10 scan
            return

        scan_stamp_sec = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )
        now_sec = (
            self.get_clock().now().nanoseconds * 1e-9
        )

        # Thử lookup TF tại đúng timestamp scan
        exact_ok = False
        exact_err = ""
        try:
            t = Time.from_msg(msg.header.stamp)
            self._tf_buffer.lookup_transform(
                "map", "base_footprint", t,
                timeout=Duration(seconds=0.2),
            )
            exact_ok = True
        except Exception as e:
            exact_err = str(e)

        # Lookup TF mới nhất để lấy stamp của TF
        tf_stamp_sec = None
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_footprint", Time(),
                timeout=Duration(seconds=0.0),
            )
            tf_stamp_sec = (
                tf.header.stamp.sec
                + tf.header.stamp.nanosec * 1e-9
            )
        except Exception as e:
            self.get_logger().warn(f"Cannot get latest TF: {e}")
            return

        delta = scan_stamp_sec - tf_stamp_sec

        self.get_logger().info(
            f"\n"
            f"  scan stamp  : {scan_stamp_sec:.6f} s\n"
            f"  TF latest   : {tf_stamp_sec:.6f} s\n"
            f"  node now    : {now_sec:.6f} s\n"
            f"  delta(scan-TF): {delta:+.4f} s  "
            f"{'← scan FUTURE vs TF!' if delta > 0.05 else ''}"
            f"{'← scan OLD vs TF!'    if delta < -0.1 else ''}\n"
            f"  exact lookup: {'OK' if exact_ok else 'FAIL: ' + exact_err[:120]}\n"
        )


def main():
    rclpy.init()
    node = TfClockDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()