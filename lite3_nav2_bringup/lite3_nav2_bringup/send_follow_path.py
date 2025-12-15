import math
import os
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


def yaw_to_quaternion(yaw: float):
    """Convert yaw (rad) to a quaternion (x, y, z, w)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def load_waypoints(file_path: str):
    """
    Load waypoints from YAML.
    Accepted formats:
    - A list of dicts: [{x: 0.0, y: 0.0}, ...]
    - A dict with key 'waypoints' holding such a list (and optional interpolation_step/step)
    - A list of [x, y] pairs
    """
    with open(file_path, "r") as f:
        data = yaml.safe_load(f)

    step = None
    if isinstance(data, dict):
        step = data.get("interpolation_step") or data.get("step")
        points_raw = data.get("waypoints", data)
    else:
        points_raw = data

    if not isinstance(points_raw, list):
        raise ValueError("Expected a list of waypoints or a dict containing key 'waypoints'.")

    points = []
    for idx, p in enumerate(points_raw):
        if isinstance(p, dict):
            x, y = p.get("x"), p.get("y")
        elif isinstance(p, (list, tuple)) and len(p) >= 2:
            x, y = p[0], p[1]
        else:
            raise ValueError(f"Waypoint #{idx} is not a dict or list with x/y.")
        if x is None or y is None:
            raise ValueError(f"Waypoint #{idx} missing x or y.")
        points.append((float(x), float(y)))

    if len(points) < 1:
        raise ValueError("No waypoints found in YAML.")
    return points, step


def compute_headings(points):
    """Compute heading (yaw) for each point based on the segment to the next point."""
    if len(points) == 1:
        return [0.0]

    headings = []
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        headings.append(math.atan2(y1 - y0, x1 - x0))
    headings.append(headings[-1])  # last point uses previous heading
    return headings


def interpolate(points, step: float):
    """
    Linear interpolation between points with given step (meters).
    Returns list of (x, y, yaw).
    """
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

    # add final point with last heading
    sampled.append((points[-1][0], points[-1][1], headings[-1]))
    return sampled


class FollowPathClient(Node):
    def __init__(self, action_name: str):
        super().__init__("follow_path_client")
        self._client = ActionClient(self, FollowPath, action_name)

    def send(self, path: Path, controller_id: str, goal_checker_id: str, timeout: float = 5.0):
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f"FollowPath action server '{self._client.action_name}' not available.")

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = controller_id
        goal_msg.goal_checker_id = goal_checker_id

        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("FollowPath goal rejected.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()

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


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Send a FollowPath goal built from a waypoints.yaml file.")
    parser.add_argument("--waypoints", help="Path to waypoints.yaml (default: package params/waypoints.yaml)")
    parser.add_argument("--frame", default="odom", help="Frame id for Path and PoseStamped headers")
    parser.add_argument(
        "--step",
        type=float,
        default=None,
        help="Interpolation step in meters (overrides YAML interpolation_step; default 0.2 if both absent)",
    )
    parser.add_argument("--controller-id", default="FollowPath", help="Controller plugin id (Nav2)")
    parser.add_argument("--goal-checker-id", default="", help="Goal checker id (Nav2)")
    parser.add_argument("--action-name", default="/follow_path", help="FollowPath action name")
    parser.add_argument("--wait-timeout", type=float, default=5.0, help="Seconds to wait for action server")
    args = parser.parse_args()

    if args.waypoints:
        waypoints_path = args.waypoints
    else:
        pkg_share = get_package_share_directory("lite3_nav2_bringup")
        waypoints_path = os.path.join(pkg_share, "params", "waypoints.yaml")

    points, yaml_step = load_waypoints(waypoints_path)
    step_m = args.step if args.step is not None else (yaml_step if yaml_step is not None else 0.2)
    sampled = interpolate(points, step_m)

    rclpy.init()
    node = FollowPathClient(args.action_name)
    node.get_logger().info(f"Using waypoints file: {waypoints_path}")
    node.get_logger().info(f"Interpolation step: {step_m} m")
    now = node.get_clock().now().to_msg()

    path_msg = build_path(sampled, args.frame, now)
    try:
        result = node.send(path_msg, args.controller_id, args.goal_checker_id, timeout=args.wait_timeout)
        node.get_logger().info(
            f"FollowPath finished with status={result.status}, result={result.result}"
        )
    except Exception as exc:  # noqa: BLE001 - surface any errors
        node.get_logger().error(f"Failed to send FollowPath goal: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

