

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <mbf_msgs/action/get_path.hpp>
#include <mbf_msgs/action/exe_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sstream>
#include <string>
#include <std_msgs/msg/string.hpp>

#include "mbf_btcpp_serialization.hpp"

using namespace BT;


class WaitForPose : public RosTopicSubNode<geometry_msgs::msg::PoseStamped>
{
public:
  WaitForPose(const std::string& name, const NodeConfig& conf,
                const RosNodeParams& params)
    : RosTopicSubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      OutputPort<geometry_msgs::msg::PoseStamped>("out_pose"),
    });
  }

  NodeStatus onTick(const std::shared_ptr<geometry_msgs::msg::PoseStamped>& last_msg) override
  {
    if(last_msg)  // empty if no new message received, since the last tick
    {
      RCLCPP_INFO(logger(), "[%s] new message! Set output", name().c_str());
      setOutput("out_pose", *last_msg);
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
  }
};

class MBFPlanningNode 
: public RosActionNode<mbf_msgs::action::GetPath>
{
public:
  MBFPlanningNode(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<mbf_msgs::action::GetPath>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ 
      InputPort<std::string>("planner_name"),
      InputPort<geometry_msgs::msg::PoseStamped>("in_pose"),
      OutputPort<nav_msgs::msg::Path>("out_path")
    });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};


class MBFControlNode 
: public RosActionNode<mbf_msgs::action::ExePath>
{
public:
  MBFControlNode(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<mbf_msgs::action::ExePath>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      InputPort<std::string>("controller_name"),
      InputPort<nav_msgs::msg::Path>("in_path")
    });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
