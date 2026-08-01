"""
ros_node.py — AGV Pacific Autonomous Robot HMI v4.3.3

Chức năng chính:
- ROS2 spin bằng SingleThreadedExecutor trong thread riêng.
- Nhận AMCL, odometry, map, laser scan, trạng thái robot, sensor và Nav2.
- Chuyển dữ liệu ROS sang Qt signal cho các màn hình HMI.
- Nhận snapshot đội AGV từ Fleet Server qua /fleet/snapshot.

FIX v4.3.3 — Fleet Monitor:
- Bổ sung fleet_snapshot_signal.
- Chỉ subscribe /fleet/snapshot một lần.
- QoS RELIABLE + TRANSIENT_LOCAL khớp Fleet Server.
- Kiểm tra JSON trước khi chuyển snapshot sang UI.
- Ghi log khi HMI nhận snapshot Fleet đầu tiên.
"""

import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Int32, Bool
from action_msgs.msg import GoalStatusArray

from tf2_ros import Buffer, TransformListener

from PyQt6.QtCore import QObject, pyqtSignal

# Khoảng lùi timestamp khi lookup TF (giây).
_TF_LOOKBACK_SEC = 0.02


def _yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _stamp_minus(stamp, seconds: float):
    ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    ns -= int(seconds * 1_000_000_000)
    if ns < 0:
        ns = 0
    return Time(nanoseconds=ns)


# Map GoalStatus status code → text hiển thị
# FIX: EXECUTING không còn map sang "ok" (trước đây bị clear_nav2() nuốt
# mất, không bao giờ hiển thị). Để trống banner khi đang chạy bình thường —
# ErrorHeader chỉ cần báo warn/error, không cần báo "đang chạy" liên tục.
_GOAL_STATUS = {
    0: None,            # STATUS_UNKNOWN  — không hiển thị
    1: None,            # STATUS_ACCEPTED — chờ
    2: None,            # EXECUTING       — chạy bình thường, không cần banner
    3: ("warn",  "Nav2: Đang huỷ goal."),      # CANCELING
    4: None,            # SUCCEEDED       — xoá banner
    5: ("warn",  "Nav2: Goal bị huỷ."),        # CANCELED
    6: ("error", "Nav2: Goal thất bại!"),      # ABORTED
}


class RosInterface(QObject):
    pose_signal           = pyqtSignal(float, float, float)
    mapping_pose_signal   = pyqtSignal(float, float, float)
    odom_signal           = pyqtSignal(float, float, float)
    odom_twist_signal     = pyqtSignal(float, float)
    map_signal            = pyqtSignal(object)
    scan_signal           = pyqtSignal(list)
    robot_status_signal   = pyqtSignal(str)
    battery_signal        = pyqtSignal(int)
    connection_signal     = pyqtSignal(str)
    agv_moving_signal     = pyqtSignal(bool)
    conveyor_cargo_signal = pyqtSignal(int, bool)
    sensor_signal          = pyqtSignal(int, bool)
    bumper_signal          = pyqtSignal(str, bool)
    path_signal            = pyqtSignal(list)   # list of (x, y) world coords

    # Nav2 planner/controller status → ErrorHeader trong nav/routes
    # payload: JSON string {"level":"ok|warn|error","message":"..."}
    nav2_status_signal     = pyqtSignal(str)

    # Snapshot JSON tổng hợp nhiều AGV từ Fleet Server.
    fleet_snapshot_signal = pyqtSignal(str)

    def __init__(self, args=None):
        super().__init__()
        self._is_shutdown = False

        rclpy.init(args=args)
        self.node = Node("agv_hmi")

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)

        # TF stats
        self._scan_tf_exact_ok = 0
        self._scan_tf_fallback = 0
        self._scan_tf_drop     = 0
        self._scan_log_count   = 0

        # Fleet snapshot diagnostics
        self._fleet_snapshot_count = 0
        self._fleet_snapshot_invalid_count = 0

        # ── QoS ───────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        action_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscriptions ─────────────────────────────────────────
        self.node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose",
            self._amcl_cb, amcl_qos)
        self.node.create_subscription(
            Odometry, "/odometry/filtered",
            self._odom_cb, 10)
        self.node.create_subscription(
            OccupancyGrid, "/map",
            self._map_cb, map_qos)
        self.node.create_subscription(
            LaserScan, "/scan",
            self._scan_cb, sensor_qos)
        self.node.create_subscription(
            String,  "/robot_status",      self._robot_status_cb, best_effort)
        self.node.create_subscription(
            Int32,   "/battery_status",    self._battery_cb,      best_effort)
        self.node.create_subscription(
            String,  "/connection_status", self._connection_cb,   best_effort)
        self.node.create_subscription(
            Bool,    "/agv_status",        self._agv_status_cb,   best_effort)
        self.node.create_subscription(
            String,  "/conveyor_cargo",    self._conveyor_cargo_cb, best_effort)
        self.node.create_subscription(
            String,  "/sensor_states",     self._sensor_cb,        best_effort)
        self.node.create_subscription(
            String,  "/bumper_states",     self._bumper_cb,        best_effort)
        # ── Nav2 action status ─────────────────────────────────────
        # GoalStatusArray được publish bởi action server navigate_to_pose
        self.node.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._nav2_action_status_cb,
            action_qos,
        )


        # Snapshot tổng hợp nhiều AGV từ Fleet Server.
        # QoS phải khớp publisher trong agv_fleet/fleet_server.py.
        fleet_snapshot_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._fleet_snapshot_sub = self.node.create_subscription(
            String,
            "/fleet/snapshot",
            self._fleet_snapshot_cb,
            fleet_snapshot_qos,
        )

        # ── Publishers ────────────────────────────────────────────
        self._cmd_pub = self.node.create_publisher(
            Twist, "/cmd_vel", 10)
        self._init_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self._conveyor_pub = self.node.create_publisher(
            String, "/conveyor_cmd", 10)

        self._path_sub = self.node.create_subscription(
            Path,
            "/plan",
            self._path_cb,
            10,
        )

        # ── Spin thread ───────────────────────────────────────────
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name="ros_spin")
        self._spin_thread.start()

        self.node.get_logger().info(
            "AGV HMI node started (spin thread active, fleet snapshot enabled)"
        )

    def _spin_loop(self):
        try:
            self._executor.spin()
        except Exception:
            pass

    # ── TF helpers ────────────────────────────────────────────────

    def _lookup_tf(self, target: str, source: str, stamp):
        target = target.strip().lstrip("/")
        source = source.strip().lstrip("/")
        try:
            t = _stamp_minus(stamp, _TF_LOOKBACK_SEC)
            return self._tf_buffer.lookup_transform(
                target, source, t, timeout=Duration(seconds=0.0))
        except Exception:
            pass
        try:
            return self._tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=0.0))
        except Exception:
            return None

    def _lookup_tf_latest(self, target: str, source: str):
        target = target.strip().lstrip("/")
        source = source.strip().lstrip("/")
        try:
            return self._tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=0.0))
        except Exception:
            return None

    # ── Callbacks ─────────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self.pose_signal.emit(
            float(p.position.x),
            float(p.position.y),
            float(_yaw_from_quat(p.orientation)),
        )

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        self.odom_signal.emit(
            float(p.position.x),
            float(p.position.y),
            float(_yaw_from_quat(p.orientation)),
        )
        lin = float(msg.twist.twist.linear.x)
        ang = float(msg.twist.twist.angular.z)
        self.odom_twist_signal.emit(lin, ang)
        self.agv_moving_signal.emit(abs(lin) > 0.01 or abs(ang) > 0.01)

    def _map_cb(self, msg: OccupancyGrid):
        self.map_signal.emit(msg)

    def _scan_cb(self, msg: LaserScan):
        src_frame = msg.header.frame_id.strip().lstrip("/")
        if not src_frame:
            return

        tf = None
        try:
            t  = _stamp_minus(msg.header.stamp, _TF_LOOKBACK_SEC)
            tf = self._tf_buffer.lookup_transform(
                "map", src_frame, t, timeout=Duration(seconds=0.0))
            self._scan_tf_exact_ok += 1
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(
                    "map", src_frame, Time(), timeout=Duration(seconds=0.0))
                self._scan_tf_fallback += 1
            except Exception:
                self._scan_tf_drop += 1
                return

        self._scan_log_count += 1
        if self._scan_log_count % 100 == 0:
            total = (self._scan_tf_exact_ok
                     + self._scan_tf_fallback + self._scan_tf_drop)
            pct_e = self._scan_tf_exact_ok * 100 // total if total else 0
            pct_f = self._scan_tf_fallback * 100 // total if total else 0
            pct_d = self._scan_tf_drop * 100 // total if total else 0
            self.node.get_logger().info(
                f"[HMI scan TF] lookback={self._scan_tf_exact_ok}({pct_e}%), "
                f"fallback={self._scan_tf_fallback}({pct_f}%), "
                f"drop={self._scan_tf_drop}({pct_d}%), src={src_frame}")

        tx  = tf.transform.translation.x
        ty  = tf.transform.translation.y
        yaw = _yaw_from_quat(tf.transform.rotation)

        robot_tf = None
        for base_frame in ("base_footprint", "base_link"):
            robot_tf = self._lookup_tf("map", base_frame, msg.header.stamp)
            if robot_tf is not None:
                break

        if robot_tf is not None:
            self.mapping_pose_signal.emit(
                float(robot_tf.transform.translation.x),
                float(robot_tf.transform.translation.y),
                float(_yaw_from_quat(robot_tf.transform.rotation)),
            )
        else:
            self.mapping_pose_signal.emit(float(tx), float(ty), float(yaw))

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        max_points = 720
        step = max(1, n // max_points)

        pts = []
        for i in range(0, n, step):
            r = ranges[i]
            if not math.isfinite(r):
                continue
            if not (msg.range_min < r < msg.range_max):
                continue
            a  = msg.angle_min + i * msg.angle_increment
            lx = r * math.cos(a)
            ly = r * math.sin(a)
            pts.append((
                float(tx + lx * cy - ly * sy),
                float(ty + lx * sy + ly * cy),
            ))

        self.scan_signal.emit(pts)

    def _nav2_action_status_cb(self, msg: GoalStatusArray):
        """
        Parse GoalStatusArray từ navigate_to_pose action server.
        Chỉ lấy goal cuối (mới nhất) để hiển thị.
        Emit nav2_status_signal dạng JSON {"level":..., "message":...}
        để ErrorHeader.update_from_ros() parse được.
        """
        if not msg.status_list:
            # Không có goal nào → xoá banner
            self.nav2_status_signal.emit(
                json.dumps({"level": "ok", "message": ""}))
            return

        # Lấy goal cuối cùng trong list
        last = msg.status_list[-1]
        code = last.status

        entry = _GOAL_STATUS.get(code)
        if entry is None:
            # SUCCEEDED, EXECUTING, hoặc UNKNOWN → xoá banner
            self.nav2_status_signal.emit(
                json.dumps({"level": "ok", "message": ""}))
            return

        level, message = entry
        self.nav2_status_signal.emit(
            json.dumps({"level": level, "message": message}))

    def _fleet_snapshot_cb(self, msg: String):
        """Validate Fleet Server snapshot and forward it safely to Qt."""
        payload = str(msg.data or "").strip()
        if not payload:
            self._fleet_snapshot_invalid_count += 1
            if self._fleet_snapshot_invalid_count <= 3:
                self.node.get_logger().warn(
                    "Received empty payload on /fleet/snapshot"
                )
            return

        try:
            snapshot = json.loads(payload)
            if not isinstance(snapshot, dict):
                raise ValueError("snapshot root must be a JSON object")

            robots = snapshot.get("robots", [])
            if robots is None:
                robots = []
            if not isinstance(robots, list):
                raise ValueError("field 'robots' must be a list")
        except Exception as exc:
            self._fleet_snapshot_invalid_count += 1
            if self._fleet_snapshot_invalid_count <= 5:
                self.node.get_logger().warn(
                    f"Invalid /fleet/snapshot payload: {exc}"
                )
            return

        self._fleet_snapshot_count += 1
        if self._fleet_snapshot_count == 1:
            self.node.get_logger().info(
                "Fleet snapshot connected: "
                f"schema={snapshot.get('schema', 'unknown')}, "
                f"robots={len(robots)}"
            )

        self.fleet_snapshot_signal.emit(payload)

    def _robot_status_cb(self, msg: String):
        self.robot_status_signal.emit(msg.data)

    def _battery_cb(self, msg: Int32):
        self.battery_signal.emit(max(0, min(100, int(msg.data))))

    def _connection_cb(self, msg: String):
        state = msg.data.lower().strip()
        if state not in ("offline", "idle", "online"):
            state = "offline"
        self.connection_signal.emit(state)

    def _agv_status_cb(self, msg: Bool):
        self.agv_moving_signal.emit(bool(msg.data))

    def _conveyor_cargo_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.conveyor_cargo_signal.emit(
                int(d["belt_id"]) - 1, bool(d["has_cargo"]))
        except Exception:
            pass

    def _sensor_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.sensor_signal.emit(int(d["sensor_id"]), bool(d["state"]))
        except Exception:
            pass

    def _bumper_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.bumper_signal.emit(str(d["side"]), bool(d["triggered"]))
        except Exception:
            pass

    # ── Publishers ────────────────────────────────────────────────

    def _ros_context_alive(self) -> bool:
        if self._is_shutdown:
            return False
        try:
            return bool(rclpy.ok())
        except Exception:
            return False

    def _safe_publish(self, publisher, msg) -> bool:
        if not self._ros_context_alive():
            return False
        try:
            publisher.publish(msg)
            return True
        except Exception as exc:
            # During application shutdown, Qt timers/signals may deliver one last
            # callback after the ROS context has already been invalidated.
            if not self._is_shutdown:
                try:
                    self.node.get_logger().warn(f"ROS publish failed: {exc}")
                except Exception:
                    pass
            return False

    def publish_velocity(self, linear: float, angular: float):
        if not self._ros_context_alive():
            return
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self._safe_publish(self._cmd_pub, t)

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        if not self._ros_context_alive():
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov
        self._safe_publish(self._init_pose_pub, msg)

    def publish_conveyor_cmd(self, payload: dict):
        if not self._ros_context_alive():
            return
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        if self._safe_publish(self._conveyor_pub, msg):
            try:
                self.node.get_logger().info(f"[HMI] /conveyor_cmd -> {msg.data}")
            except Exception:
                pass

    def spin_once(self):
        pass  # spin thread handles everything

    def _path_cb(self, msg: Path):
        pts = []
        for pose_stamped in msg.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            pts.append((x, y))
        if pts:
            self.path_signal.emit(pts)

    # ── Shutdown ──────────────────────────────────────────────────
    def shutdown(self):
        if self._is_shutdown:
            return
        self._is_shutdown = True

        total = (self._scan_tf_exact_ok
                 + self._scan_tf_fallback + self._scan_tf_drop)
        if total > 0:
            try:
                pct_e = self._scan_tf_exact_ok * 100 // total
                pct_f = self._scan_tf_fallback * 100 // total
                pct_d = self._scan_tf_drop * 100 // total
                self.node.get_logger().info(
                    f"[HMI scan TF final] "
                    f"lookback={self._scan_tf_exact_ok}({pct_e}%), "
                    f"fallback={self._scan_tf_fallback}({pct_f}%), "
                    f"drop={self._scan_tf_drop}({pct_d}%)")
            except Exception:
                pass

        try:
            self.node.get_logger().info(
                "[HMI fleet final] "
                f"snapshots={self._fleet_snapshot_count}, "
                f"invalid={self._fleet_snapshot_invalid_count}"
            )
        except Exception:
            pass

        try: self._executor.shutdown(timeout_sec=1.0)
        except Exception: pass
        try: self.node.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass
