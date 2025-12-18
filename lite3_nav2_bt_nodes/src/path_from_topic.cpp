#include <optional>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lite3_nav2_bt_nodes
{

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
      topic_ = "/input_path";
    }

    // Match the visualize_waypoints publisher (reliable + transient_local).
    // Depth>1 to tolerate bursts.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      topic_, qos,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        latest_path_ = *msg;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/input_path", "Path topic to listen on"),
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

    setOutput("path", *latest_path_);
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


