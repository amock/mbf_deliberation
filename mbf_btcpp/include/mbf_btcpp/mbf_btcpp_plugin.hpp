

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <mbf_msgs/action/get_path.hpp>
#include <mbf_msgs/action/exe_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sstream>
#include <string>
#include <std_msgs/msg/string.hpp>

using namespace BT;


namespace BT
{
template <> inline
geometry_msgs::msg::PoseStamped convertFromString(
  StringView str)
{
  geometry_msgs::msg::PoseStamped pose;
  std::istringstream iss{std::string(str)};
  double px, py, pz, ox, oy, oz, ow;
  if (!(iss >> px >> py >> pz >> ox >> oy >> oz >> ow))
  {
    throw BT::RuntimeError("Invalid input for PoseStamped. Expected 7 space-separated floats: "
                           "px py pz ox oy oz ow");
  }

  pose.pose.position.x = px;
  pose.pose.position.y = py;
  pose.pose.position.z = pz;
  pose.pose.orientation.x = ox;
  pose.pose.orientation.y = oy;
  pose.pose.orientation.z = oz;
  pose.pose.orientation.w = ow;

  return pose;
}

template <>
inline std::string toStr(
  const geometry_msgs::msg::PoseStamped &pose)
{
  std::ostringstream oss;
  oss << pose.pose.position.x << " "
      << pose.pose.position.y << " "
      << pose.pose.position.z << " "
      << pose.pose.orientation.x << " "
      << pose.pose.orientation.y << " "
      << pose.pose.orientation.z << " "
      << pose.pose.orientation.w;
  return oss.str();
}


template <> inline
nav_msgs::msg::Path convertFromString(
  StringView str)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  std::istringstream iss{std::string(str)};

  int n_poses = 0;
  iss >> n_poses;

  for(size_t i=0; i<n_poses; i++)
  {
    geometry_msgs::msg::PoseStamped pose;
    double px, py, pz, ox, oy, oz, ow;
    if (!(iss >> px >> py >> pz >> ox >> oy >> oz >> ow))
    {
      throw BT::RuntimeError("Invalid input for PoseStamped. Expected 7 space-separated floats: "
                            "px py pz ox oy oz ow");
    }
    pose.pose.position.x = px;
    pose.pose.position.y = py;
    pose.pose.position.z = pz;
    pose.pose.orientation.x = ox;
    pose.pose.orientation.y = oy;
    pose.pose.orientation.z = oz;
    pose.pose.orientation.w = ow;
    pose.header.frame_id = "map";
    path.poses.push_back(pose);
  }
  
  return path;
}

template <>
inline std::string toStr(
  const nav_msgs::msg::Path& path)
{
  std::ostringstream oss;

  oss << path.poses.size() << " ";
  for(size_t i=0; i<path.poses.size(); i++)
  {
    auto pose = path.poses[i];
    oss << pose.pose.position.x << " "
      << pose.pose.position.y << " "
      << pose.pose.position.z << " "
      << pose.pose.orientation.x << " "
      << pose.pose.orientation.y << " "
      << pose.pose.orientation.z << " "
      << pose.pose.orientation.w;
  }

  return oss.str();
}


} // namespace BT




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
      OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
    });
  }

  NodeStatus onTick(const std::shared_ptr<geometry_msgs::msg::PoseStamped>& last_msg) override
  {
    if(last_msg)  // empty if no new message received, since the last tick
    {
      RCLCPP_INFO(logger(), "[%s] new message", name().c_str());
    }
    return NodeStatus::SUCCESS;
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
      InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
      OutputPort<nav_msgs::msg::Path>("path")
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
      InputPort<nav_msgs::msg::Path>("path") 
    });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
