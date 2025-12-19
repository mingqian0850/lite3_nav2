import math
import os
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def load_waypoints(file_path: str):
    """
    Load waypoints from YAML.
    Formats:
    - list of dicts: [{x: 0.0, y: 0.0}, ...]
    - dict with key 'waypoints' holding such a list (and optional interpolation_step/step)
    - list of [x, y]
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


def build_markers(sampled_pts, frame_id: str, stamp):
    markers = MarkerArray()

    line = Marker()
    line.header.frame_id = frame_id
    line.header.stamp = stamp
    line.ns = "waypoints"
    line.id = 0
    line.type = Marker.LINE_STRIP
    line.action = Marker.ADD
    line.pose.orientation.w = 1.0
    line.scale.x = 0.03
    line.color.r = 0.0
    line.color.g = 1.0
    line.color.b = 0.0
    line.color.a = 0.9

    spheres = Marker()
    spheres.header.frame_id = frame_id
    spheres.header.stamp = stamp
    spheres.ns = "waypoints"
    spheres.id = 1
    spheres.type = Marker.SPHERE_LIST
    spheres.action = Marker.ADD
    spheres.pose.orientation.w = 1.0
    spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.08
    spheres.color.r = 0.0
    spheres.color.g = 0.3
    spheres.color.b = 1.0
    spheres.color.a = 0.9

    for x, y, yaw in sampled_pts:
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        line.points.append(pose.pose.position)
        spheres.points.append(pose.pose.position)

    markers.markers.append(line)
    markers.markers.append(spheres)
    return markers


class WaypointsVisualizer(Node):
    def __init__(self, args):
        super().__init__("waypoints_visualizer")
        self.frame_id = args.frame
        self.topic = args.topic
        self.path_topic = args.path_topic
        self.rate = args.rate

        if args.waypoints:
            waypoints_path = args.waypoints
        else:
            pkg_share = get_package_share_directory("lite3_nav2_bringup")
            waypoints_path = os.path.join(pkg_share, "params", "waypoints.yaml")

        points, yaml_step = load_waypoints(waypoints_path)
        self.step_m = args.step if args.step is not None else (yaml_step if yaml_step is not None else 0.2)
        self.sampled = interpolate(points, self.step_m)
        self.get_logger().info(f"Using waypoints file: {waypoints_path}")
        self.get_logger().info(f"Interpolation step: {self.step_m} m")
        self.get_logger().info(f"Marker topic: {self.topic}, Path topic: {self.path_topic}")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.topic, qos)
        self.path_pub = self.create_publisher(Path, self.path_topic, qos)

        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)

    def _on_timer(self):
        stamp = self.get_clock().now().to_msg()
        markers = build_markers(self.sampled, self.frame_id, stamp)
        path_msg = build_path(self.sampled, self.frame_id, stamp)
        self.marker_pub.publish(markers)
        self.path_pub.publish(path_msg)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Continuously publish waypoints markers/path for RViz2.")
    parser.add_argument("--waypoints", help="Path to waypoints.yaml (default: package params/waypoints.yaml)")
    parser.add_argument("--frame", default="map", help="Frame id for markers and path")
    parser.add_argument("--topic", default="/waypoints/markers", help="Marker topic")
    parser.add_argument("--path-topic", default="/waypoints/path", help="Path topic")
    parser.add_argument("--rate", type=float, default=1.0, help="Publish rate in Hz")
    parser.add_argument(
        "--step",
        type=float,
        default=None,
        help="Interpolation step in meters (overrides YAML interpolation_step; default 0.2 if both absent)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = WaypointsVisualizer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

