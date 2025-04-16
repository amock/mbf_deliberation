#include "mbf_btcpp/mbf_btcpp_plugin.hpp"
#include <behaviortree_ros2/plugins.hpp>


// mbf_msgs::action::GetPath::Goal
bool MBFPlanningNode::setGoal(RosActionNode::Goal& goal)
{
  auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if(!goal_pose)
  {
    throw BT::RuntimeError("Missing or malformed input [goal_pose]: " + goal_pose.error());
  }
  goal.target_pose = goal_pose.value();
  
  goal.planner = "mesh_planner"; // TODO: ros parameter
  goal.tolerance = 0.2;
  goal.use_start_pose = false;
  return true;
}

NodeStatus MBFPlanningNode::onResultReceived(
  const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s onResultReceived", name().c_str());
  setOutput("path", wr.result->path);
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
  auto path_in = getInput<nav_msgs::msg::Path>("path");
  if(!path_in)
  {
    throw BT::RuntimeError("Missing or malformed input [path]: " + path_in.error());
  }
  goal.path = path_in.value();
  goal.controller = "mesh_controller";
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