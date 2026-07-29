import math
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavClient:
    """Small wrapper around Nav2 NavigateToPose action client."""

    def __init__(self, node: Node):
        self.node = node
        self._current_goal_handle = None
        self._client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    def is_ready(self, timeout_sec: float = 0.0) -> bool:
        """Return True when navigate_to_pose action server is available."""
        try:
            return bool(self._client.wait_for_server(timeout_sec=float(timeout_sec)))
        except Exception as e:
            self.node.get_logger().warn(f"NavigateToPose wait_for_server lỗi: {e}")
            return False

    def send_goal(self, x, y, yaw=0.0, on_result=None, on_feedback=None):
        if not self.is_ready(timeout_sec=5.0):
            self.node.get_logger().error("NavigateToPose server không phản hồi")
            if on_result:
                on_result(False)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)

        try:
            fut = self._client.send_goal_async(
                goal,
                feedback_callback=lambda fb: self._on_feedback(fb, on_feedback),
            )
            fut.add_done_callback(lambda f: self._on_accepted(f, on_result))
        except Exception as e:
            self.node.get_logger().error(f"Send NavigateToPose goal lỗi: {e}")
            if on_result:
                on_result(False)

    def cancel_goal(self) -> bool:
        if self._current_goal_handle:
            try:
                self._current_goal_handle.cancel_goal_async()
                return True
            except Exception as e:
                self.node.get_logger().warn(f"Cancel NavigateToPose goal lỗi: {e}")
            finally:
                self._current_goal_handle = None
        return False

    def _make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.node.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(float(yaw) / 2.0)
        p.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return p

    def _on_accepted(self, future, on_result):
        try:
            gh = future.result()
        except Exception as e:
            self.node.get_logger().error(f"NavigateToPose goal response lỗi: {e}")
            if on_result:
                on_result(False)
            return

        if not gh.accepted:
            if on_result:
                on_result(False)
            return

        self._current_goal_handle = gh
        try:
            rf = gh.get_result_async()
            rf.add_done_callback(lambda f: self._on_result(f, on_result))
        except Exception as e:
            self.node.get_logger().error(f"NavigateToPose get_result lỗi: {e}")
            self._current_goal_handle = None
            if on_result:
                on_result(False)

    def _on_result(self, future, on_result):
        try:
            result = future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
        except Exception as e:
            self.node.get_logger().error(f"NavigateToPose result lỗi: {e}")
            success = False

        self._current_goal_handle = None
        if on_result:
            on_result(success)

    def _on_feedback(self, fb_msg, on_feedback):
        if on_feedback:
            try:
                on_feedback(fb_msg.feedback.distance_remaining)
            except Exception:
                pass
