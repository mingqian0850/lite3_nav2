#!/usr/bin/env python3
"""
Sequentially send /follow_path goals for segments defined in a YAML waypoint list.

Usage:
    ros2 run lite3_nav2_bringup follow_path_sequence_executor --path-file path.yaml
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import List, Sequence, Tuple

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)


Point = Tuple[float, float, float]

_STATUS_STRINGS = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def _status_to_string(status: int) -> str:
    return _STATUS_STRINGS.get(status, f"STATUS_{status}")


def _load_points_from_yaml(path_file: Path) -> Tuple[str, List[Point]]:
    with path_file.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("YAML root must be a mapping containing frame_id and points.")

    frame_id = data.get("frame_id", "odom")
    raw_points = data.get("points")
    if raw_points is None:
        raise ValueError("YAML must contain a 'points' list. Each entry can be a dict or [x,y,(z)].")

    points: List[Point] = []
    for idx, entry in enumerate(raw_points):
        if isinstance(entry, dict):
            if "position" in entry:
                entry = entry["position"]
            x = float(entry.get("x", 0.0))
            y = float(entry.get("y", 0.0))
            z = float(entry.get("z", 0.0))
        elif isinstance(entry, (list, tuple)):
            if len(entry) < 2:
                raise ValueError(f"Point #{idx} must have at least two values (x,y).")
            x = float(entry[0])
            y = float(entry[1])
            z = float(entry[2]) if len(entry) > 2 else 0.0
        else:
            raise ValueError(f"Unsupported point format at index {idx}: {entry!r}")
        points.append((x, y, z))

    if len(points) < 2:
        raise ValueError("Need at least two points to create a path.")

    return frame_id, points


def _yaw_between(p0: Point, p1: Point) -> float:
    return math.atan2(p1[1] - p0[1], p1[0] - p0[0])


def _quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _pose(frame_id: str, point: Point, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = point[0]
    pose.pose.position.y = point[1]
    pose.pose.position.z = point[2]
    qx, qy, qz, qw = _quat_from_yaw(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class FollowPathSequenceExecutor(Node):
    def __init__(
        self,
        *,
        frame_id: str,
        points: Sequence[Point],
        controller_id: str,
        goal_checker_id: str,
        action_name: str,
    ) -> None:
        super().__init__("follow_path_sequence_executor")
        self._frame_id = frame_id
        self._points = list(points)
        self._controller_id = controller_id
        self._goal_checker_id = goal_checker_id
        self._client = ActionClient(self, FollowPath, action_name)

    def _build_goal_for_segment(self, start_idx: int) -> FollowPath.Goal:
        start_pt = self._points[start_idx]
        end_pt = self._points[start_idx + 1]
        yaw = _yaw_between(start_pt, end_pt)

        path = NavPath()
        path.header.frame_id = self._frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses.append(_pose(self._frame_id, start_pt, yaw))
        path.poses.append(_pose(self._frame_id, end_pt, yaw))

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self._controller_id
        goal.goal_checker_id = self._goal_checker_id
        return goal

    def execute(self) -> bool:
        self.get_logger().info("Waiting for FollowPath action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("FollowPath action server not available.")
            return False

        for idx in range(len(self._points) - 1):
            goal = self._build_goal_for_segment(idx)
            start = self._points[idx]
            end = self._points[idx + 1]
            self.get_logger().info(
                f"Sending segment {idx+1}/{len(self._points)-1}: "
                f"({start[0]:.2f},{start[1]:.2f}) -> ({end[0]:.2f},{end[1]:.2f})"
            )

            send_future = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error("Goal rejected by controller_server.")
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            if result is None:
                self.get_logger().error("Controller returned no result.")
                return False

            status = result.status
            status_name = _status_to_string(status)
            self.get_logger().info(f"Segment {idx+1} finished with status: {status_name}")
            if status != GoalStatus.STATUS_SUCCEEDED:
                return False

        self.get_logger().info("All segments completed successfully.")
        return True


def _default_waypoints_file() -> Path | None:
    try:
        share_dir = Path(get_package_share_directory("lite3_nav2_bringup"))
    except PackageNotFoundError:
        return None
    candidate = share_dir / "params" / "waypoints.yaml"
    return candidate if candidate.exists() else None


def parse_args() -> argparse.Namespace:
    default_path = _default_waypoints_file()
    parser = argparse.ArgumentParser(
        description="Execute a sequence of FollowPath goals defined in a YAML waypoint file."
    )
    parser.add_argument(
        "--path-file",
        default=str(default_path) if default_path else None,
        help=(
            "YAML file describing ordered waypoints."
            + (f" (default: {default_path})" if default_path else "")
        ),
    )
    parser.add_argument("--action-name", default="/follow_path", help="FollowPath action topic.")
    parser.add_argument("--controller-id", default="FollowPath", help="Controller plugin ID.")
    parser.add_argument(
        "--goal-checker-id",
        default="general_goal_checker",
        help="Goal checker plugin ID.",
    )
    args = parser.parse_args()
    if args.path_file is None:
        parser.error(
            "Waypoints file not provided and default could not be resolved. "
            "Pass --path-file /path/to/waypoints.yaml"
        )
    return args


def main() -> None:
    args = parse_args()
    path_file = Path(args.path_file)
    frame_id, points = _load_points_from_yaml(path_file)

    rclpy.init()
    node = FollowPathSequenceExecutor(
        frame_id=frame_id,
        points=points,
        controller_id=args.controller_id,
        goal_checker_id=args.goal_checker_id,
        action_name=args.action_name,
    )
    success = node.execute()
    node.destroy_node()
    rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()


