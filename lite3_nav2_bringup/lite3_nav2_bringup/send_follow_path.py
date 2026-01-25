import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node


def compute_headings(points):
    if len(points) == 1:
        return [0.0]
    headings = []
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        headings.append(math.atan2(y1 - y0, x1 - x0))
    headings.append(headings[-1])
    return headings


def interpolate(points, step: float):
    if step <= 0:
        raise ValueError("Interpolation step must be > 0")
    headings = compute_headings(points)
    sampled = []
    for i in range(len(points) - 1):
        (x0, y0), (x1, y1) = points[i], points[i + 1]
        yaw = headings[i]
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        steps = max(int(dist / step), 1)
        for k in range(steps):
            t = k / steps
            sampled.append((x0 + t * dx, y0 + t * dy, yaw))
    sampled.append((points[-1][0], points[-1][1], headings[-1]))
    return sampled


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def build_path(sampled_pts, frame_id: str, stamp):
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for x, y, yaw in sampled_pts:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        path.poses.append(pose)
    return path


def path_to_points(path_msg: Path):
    points = []
    for pose in path_msg.poses:
        points.append((pose.pose.position.x, pose.pose.position.y))
    if len(points) < 1:
        raise ValueError("Received empty Path.")
    return points


class FollowPathClient(Node):
    def __init__(self, args):
        super().__init__("follow_path_client")
        self._client = ActionClient(self, FollowPath, "/follow_path")
        self._args = args
        self._processing = False
        self._latest_path = None
        self._last_processed_stamp = None

        self._sub = self.create_subscription(
            Path, args.waypoints_topic, self._on_waypoints, 10
        )
        self._timer = self.create_timer(0.1, self._on_timer)

    def send_goal(self, path_msg: Path):
        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = self._args.controller_id or ""
        goal.goal_checker_id = self._args.goal_checker_id or ""

        self.get_logger().info("Waiting for /follow_path action server...")
        if not self._client.wait_for_server(timeout_sec=self._args.wait):
            self.get_logger().error("Action server /follow_path not available.")
            return None, None

        self.get_logger().info(f"Sending FollowPath goal with {len(path_msg.poses)} poses.")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("FollowPath goal rejected.")

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"FollowPath action completed with status: {result.status}")
        return result.status, result.result

    def _on_waypoints(self, msg: Path):
        self._latest_path = msg

    def _on_timer(self):
        if self._processing:
            return
        if self._latest_path is None:
            return
        stamp = self._latest_path.header.stamp
        stamp_key = (stamp.sec, stamp.nanosec)
        if self._last_processed_stamp == stamp_key:
            return
        self._processing = True
        msg = self._latest_path
        self.get_logger().info(
            f"Processing waypoints with {len(msg.poses)} poses (stamp={stamp_key})."
        )

        try:
            points = path_to_points(msg)
            step_m = self._args.step if self._args.step is not None else 0.2
            sampled = (
                interpolate(points, step_m)
                if len(points) > 1
                else [(points[0][0], points[0][1], 0.0)]
            )
            frame_id = (
                self._args.frame
                if self._args.frame is not None
                else (msg.header.frame_id or "map")
            )
            stamp = self.get_clock().now().to_msg()
            path_msg = build_path(sampled, frame_id, stamp)

            attempts = max(self._args.retries, 0) + 1
            for attempt in range(1, attempts + 1):
                status, _ = self.send_goal(path_msg)
                if status is None:
                    self.get_logger().warn(
                        f"FollowPath server unavailable (attempt {attempt}/{attempts}), retrying in {self._args.retry_wait}s..."
                    )
                    rclpy.spin_once(self, timeout_sec=self._args.retry_wait)
                    continue
                if status == GoalStatus.STATUS_ABORTED and attempt < attempts:
                    self.get_logger().warn(
                        f"FollowPath aborted (attempt {attempt}/{attempts}), retrying in {self._args.retry_wait}s..."
                    )
                    rclpy.spin_once(self, timeout_sec=self._args.retry_wait)
                    continue
                break
            self._last_processed_stamp = stamp_key
        finally:
            self._processing = False


def main():
    import argparse
    import sys
    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser(
        description="Subscribe /waypoints Path and send FollowPath action goal."
    )
    parser.add_argument(
        "--waypoints-topic",
        default="/waypoints/raw",
        help="Path topic to subscribe (nav_msgs/Path)",
    )
    parser.add_argument(
        "--frame",
        default=None,
        help="Override frame_id in received Path (default: use incoming frame_id or map)",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=0.2,
        help="Interpolation step in meters for received Path",
    )
    parser.add_argument("--controller-id", default="", help="controller_id (optional)")
    parser.add_argument("--goal-checker-id", default="", help="goal_checker_id (optional)")
    parser.add_argument("--wait", type=float, default=10.0, help="Wait time for action server (s)")
    parser.add_argument("--retries", type=int, default=0, help="Retry count on ABORTED")
    parser.add_argument("--retry-wait", type=float, default=0.5, help="Wait seconds before retry")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = FollowPathClient(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

