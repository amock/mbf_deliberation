

#include <behaviortree_ros2/bt_action_node.hpp>
#include <mbf_msgs/action/get_path.hpp>
#include <mbf_msgs/action/exe_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sstream>
#include <string>

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


class WaitForPose : public BT::StatefulActionNode
{
public:




  WaitForPose(const std::string& name, 
      const NodeConfig& conf,
      const RosNodeParams& params)
  : BT::StatefulActionNode(name, conf)
  , node_(params.nh)
  , message_received_(false)
  {
    std::cout << "LOCK" << std::endl;
    auto node = node_.lock();

    if(!node)
    {
      throw RuntimeError("The ROS node went out of scope. RosNodeParams doesn't take the "
                        "ownership of the node.");
    }
    
    auto topic_name = getInput<std::string>("topic_name");
    if(!topic_name)
    {
      throw BT::RuntimeError("Missing or malformed input [topic_name]: " + topic_name.error());
    }
    
    std::cout << "Connecting to topic: " << topic_name.value() << std::endl;
    
    create_subscriber(node, topic_name.value());

    std::cout << "Connected." << std::endl;
  }

  void create_subscriber(std::shared_ptr<rclcpp::Node> node, const std::string& topic_name)
  {
    // create a callback group for this particular instance
    callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_,
                                            node->get_node_base_interface());

    rclcpp::SubscriptionOptions option;
    option.callback_group = callback_group_;

    // The callback will broadcast to all the instances of RosTopicSubNode<T>
    auto callback = [this](const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
      last_msg_ = *msg;
      message_received_ = true;
    };
    sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(topic_name, 1, callback, option);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose")
    };
  }

  BT::NodeStatus onStart() override
  {
    message_received_ = false;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if(message_received_)
    {
      std::cout << "message received 2" << std::endl;
      setOutput("goal_pose", last_msg_);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    // Nothing special here
  }

private:

  std::weak_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  geometry_msgs::msg::PoseStamped last_msg_;
  std::atomic<bool> message_received_;
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
