#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lite3_nav2_bt_nodes
{

struct Point2d
{
  double x;
  double y;
};

static std::vector<Point2d> pathToPoints(const nav_msgs::msg::Path & path)
{
  std::vector<Point2d> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    points.push_back({pose.pose.position.x, pose.pose.position.y});
  }
  return points;
}

static std::vector<double> computeHeadings(const std::vector<Point2d> & points)
{
  if (points.size() == 1U) {
    return {0.0};
  }
  std::vector<double> headings;
  headings.reserve(points.size());
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & p0 = points[i];
    const auto & p1 = points[i + 1];
    headings.push_back(std::atan2(p1.y - p0.y, p1.x - p0.x));
  }
  headings.push_back(headings.back());
  return headings;
}

static std::vector<std::tuple<double, double, double>> interpolate(
  const std::vector<Point2d> & points, double step)
{
  std::vector<std::tuple<double, double, double>> sampled;
  if (points.size() < 2U) {
    return sampled;
  }

  const auto headings = computeHeadings(points);
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & p0 = points[i];
    const auto & p1 = points[i + 1];
    const double yaw = headings[i];
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double dist = std::hypot(dx, dy);
    const int steps = std::max(static_cast<int>(dist / step), 1);
    for (int k = 0; k < steps; ++k) {
      const double t = static_cast<double>(k) / steps;
      sampled.emplace_back(p0.x + t * dx, p0.y + t * dy, yaw);
    }
  }
  sampled.emplace_back(points.back().x, points.back().y, headings.back());
  return sampled;
}

class PathFromTopic : public BT::SyncActionNode
{
public:
  PathFromTopic(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    if (!config.blackboard->get("node", node_)) {
      throw BT::RuntimeError("PathFromTopic: missing blackboard entry 'node'");
    }

    if (!getInput("topic", topic_)) {
      topic_ = "/waypoints/raw";
    }

    // Use volatile durability to accept default publishers; transient_local
    // publishers remain compatible with a volatile subscriber.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      topic_, qos,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        latest_path_ = *msg;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/waypoints/raw", "Path topic to listen on"),
      BT::InputPort<double>("step", 0.05, "Interpolation step in meters"),
      BT::InputPort<std::string>("frame", "", "Override frame id (optional)"),
      BT::OutputPort<nav_msgs::msg::Path>("path", "Latest received path")
    };
  }

  BT::NodeStatus tick() override
  {
    // Pump the executor so subscriptions can be processed.
    rclcpp::spin_some(node_);

    if (!latest_path_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "PathFromTopic: no path received yet on '%s'", topic_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    double step = 0.05;
    if (getInput("step", step)) {
      if (step <= 0.0) {
        RCLCPP_WARN(
          node_->get_logger(), "PathFromTopic: step must be > 0 (got %.3f)", step);
        return BT::NodeStatus::FAILURE;
      }
    }

    auto points = pathToPoints(*latest_path_);
    if (points.empty()) {
      RCLCPP_WARN(node_->get_logger(), "PathFromTopic: received empty path.");
      return BT::NodeStatus::FAILURE;
    }

    std::vector<std::tuple<double, double, double>> sampled;
    if (points.size() == 1U) {
      sampled.emplace_back(points.front().x, points.front().y, 0.0);
    } else {
      sampled = interpolate(points, step);
    }
    if (sampled.empty()) {
      RCLCPP_WARN(node_->get_logger(), "PathFromTopic: interpolation produced empty path.");
      return BT::NodeStatus::FAILURE;
    }

    std::string frame_id;
    if (!getInput("frame", frame_id) || frame_id.empty()) {
      frame_id = latest_path_->header.frame_id.empty() ? "map" : latest_path_->header.frame_id;
    }

    const auto stamp = node_->get_clock()->now();
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;
    for (const auto & sample : sampled) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.header.stamp = stamp;
      const double x = std::get<0>(sample);
      const double y = std::get<1>(sample);
      const double yaw = std::get<2>(sample);
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      const double half = yaw * 0.5;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(half);
      pose.pose.orientation.w = std::cos(half);
      path.poses.push_back(pose);
    }

    setOutput("path", path);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_;
  std::optional<nav_msgs::msg::Path> latest_path_;
  std::string topic_;
};

}  // namespace lite3_nav2_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<lite3_nav2_bt_nodes::PathFromTopic>("PathFromTopic");
}


