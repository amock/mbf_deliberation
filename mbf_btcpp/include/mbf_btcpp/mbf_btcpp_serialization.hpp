#ifndef MBF_BTCPP_SERIALIZATION_HPP
#define MBF_BTCPP_SERIALIZATION_HPP

#include <behaviortree_ros2/bt_action_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

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

#endif // MBF_BTCPP_SERIALIZATION_HPP