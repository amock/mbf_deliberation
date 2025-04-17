#include "mbf_btcpp/mbf_btcpp_plugin.hpp"
#include <behaviortree_ros2/plugins.hpp>


// mbf_msgs::action::GetPath::Goal
bool MBFPlanningNode::setGoal(RosActionNode::Goal& goal)
{
  RCLCPP_INFO(logger(), "[%s] prepare goal", name().c_str());
  auto in_pose = getInput<geometry_msgs::msg::PoseStamped>("in_pose");
  if(!in_pose)
  {
    throw BT::RuntimeError("Missing or malformed input [in_pose]: " + in_pose.error());
  }

  auto planner_name = getInput<std::string>("planner_name");
  if(!planner_name)
  {
    throw BT::RuntimeError("Missing or malformed input [planner_name]: " + planner_name.error());
  }

  goal.target_pose = in_pose.value();
  goal.planner = planner_name.value();
  goal.tolerance = 0.2;
  goal.use_start_pose = false;
  RCLCPP_INFO(logger(), "[%s] send goal", name().c_str());
  return true;
}

NodeStatus MBFPlanningNode::onResultReceived(
  const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s onResultReceived", name().c_str());
  setOutput("out_path", wr.result->path);
  return NodeStatus::SUCCESS;// : NodeStatus::FAILURE;
}

NodeStatus MBFPlanningNode::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void MBFPlanningNode::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}


///////////////////////////////
// MBFControlNode
////////
bool MBFControlNode::setGoal(RosActionNode::Goal& goal)
{
  // auto timeout = getInput<unsigned>("msec");
  auto in_path = getInput<nav_msgs::msg::Path>("in_path");
  if(!in_path)
  {
    throw BT::RuntimeError("Missing or malformed input [in_path]: " + in_path.error());
  }

  auto controller_name = getInput<std::string>("controller_name");
  if(!controller_name)
  {
    throw BT::RuntimeError("Missing or malformed input [controller_name]: " + controller_name.error());
  }

  goal.path = in_path.value();
  goal.controller = controller_name.value();
  goal.dist_tolerance = 0.2;

  return true;
}

NodeStatus MBFControlNode::onResultReceived(
  const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s onResultReceived", name().c_str());
  return NodeStatus::SUCCESS;// : NodeStatus::FAILURE;
}

NodeStatus MBFControlNode::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void MBFControlNode::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<WaitForPose>("WaitForPose", params);
  factory.registerNodeType<MBFPlanningNode>("MBFPlanningNode", params);
  factory.registerNodeType<MBFControlNode>("MBFControlNode", params);
}