import math
import os
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def load_waypoints_with_meta(file_path: str):
    with open(file_path, "r") as f:
        data = yaml.safe_load(f)

    frame_id = "map"
    points_raw = data

    if isinstance(data, dict):
        frame_id = data.get("frame_id", frame_id)
        points_raw = data.get("waypoints", data)

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

    return points, frame_id


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


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def build_path(points, frame_id: str, stamp):
    headings = compute_headings(points)
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for (x, y), yaw in zip(points, headings):
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


class WaypointsPublisher(Node):
    def __init__(self, args):
        super().__init__("waypoints_publisher")
        self.topic = args.topic

        if args.waypoints:
            waypoints_path = args.waypoints
        else:
            pkg_share = get_package_share_directory("lite3_nav2_bringup")
            waypoints_path = os.path.join(pkg_share, "params", "waypoints.yaml")

        points, yaml_frame = load_waypoints_with_meta(waypoints_path)
        self.frame_id = args.frame if args.frame is not None else yaml_frame
        self.path_msg = build_path(points, self.frame_id, self.get_clock().now().to_msg())

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Path, self.topic, qos)
        self.timer = self.create_timer(1.0 / args.rate, self._on_timer)

        self.get_logger().info(f"Publishing /waypoints Path to: {self.topic}")
        self.get_logger().info(f"Waypoints file: {waypoints_path}")
        self.get_logger().info(f"Frame id: {self.frame_id}")

    def _on_timer(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for pose in self.path_msg.poses:
            pose.header.stamp = self.path_msg.header.stamp
        self.pub.publish(self.path_msg)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Publish /waypoints Path from waypoints.yaml.")
    parser.add_argument("--waypoints", help="Path to waypoints.yaml (default: package params/waypoints.yaml)")
    parser.add_argument("--frame", default=None, help="Override frame_id in YAML (default: use YAML or map)")
    parser.add_argument("--topic", default="/waypoints", help="Topic to publish (nav_msgs/Path)")
    parser.add_argument("--rate", type=float, default=1.0, help="Publish rate in Hz")
    args = parser.parse_args()

    rclpy.init()
    node = WaypointsPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

