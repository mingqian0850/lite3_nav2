import math
import rclpy
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


def path_to_points(path_msg: Path):
    points = []
    for pose in path_msg.poses:
        points.append((pose.pose.position.x, pose.pose.position.y))
    if len(points) < 1:
        raise ValueError("Received empty Path on /waypoints.")
    return points


class WaypointsVisualizer(Node):
    def __init__(self, args):
        super().__init__("waypoints_visualizer")
        self.frame_id = args.frame
        self.topic = args.topic
        self.rate = args.rate
        self.step_m = args.step
        self._sampled = None
        self._latest_path = None
        self._last_processed_stamp = None

        self._sub = self.create_subscription(
            Path, args.waypoints_topic, self._on_waypoints, 10
        )
        self.get_logger().info(f"Waiting for /waypoints Path on: {args.waypoints_topic}")
        self.get_logger().info(f"Interpolation step: {self.step_m} m")
        self.get_logger().info(f"Marker topic: {self.topic}")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.topic, qos)

        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)

    def _on_waypoints(self, msg: Path):
        self._latest_path = msg

    def _on_timer(self):
        if self._latest_path is not None:
            stamp = self._latest_path.header.stamp
            stamp_key = (stamp.sec, stamp.nanosec)
            if self._last_processed_stamp != stamp_key:
                points = path_to_points(self._latest_path)
                self._sampled = (
                    interpolate(points, self.step_m)
                    if len(points) > 1
                    else [(points[0][0], points[0][1], 0.0)]
                )
                if self.frame_id is None:
                    self.frame_id = self._latest_path.header.frame_id or "map"
                self._last_processed_stamp = stamp_key
                self.get_logger().info(
                    f"Visualizing waypoints with {len(self._latest_path.poses)} poses (stamp={stamp_key})."
                )

        if self._sampled is None:
            return
        stamp = self.get_clock().now().to_msg()
        markers = build_markers(self._sampled, self.frame_id, stamp)
        self.marker_pub.publish(markers)


def main():
    import argparse
    import sys
    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser(description="Publish markers from /waypoints (nav_msgs/Path).")
    parser.add_argument(
        "--waypoints-topic",
        default="/waypoints/raw",
        help="Path topic to subscribe (nav_msgs/Path)",
    )
    parser.add_argument(
        "--frame",
        default=None,
        help="Override frame id for markers and path (default: use incoming frame_id or map)",
    )
    parser.add_argument("--topic", default="/waypoints/interpolated/markers", help="Marker topic")
    parser.add_argument("--rate", type=float, default=1.0, help="Publish rate in Hz")
    parser.add_argument(
        "--step",
        type=float,
        default=0.2,
        help="Interpolation step in meters for received Path",
    )
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

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

