#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "behaviortree_ros2/plugins.hpp"

using namespace BT;

static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <WaitForPose name="WaitForPose" 
              topic_name="/goal_pose" 
              out_pose="{a_goal}"/>
        <MBFPlanningNode name="MBFPlanningNode" 
              action_name="/move_base_flex/get_path" 
              in_pose="{a_goal}" 
              out_path="{a_path}" />
        <MBFControlNode name="MBFControlNode" 
              action_name="move_base_flex/exe_path" 
              in_path="{a_path}" />
      </Sequence>
    </BehaviorTree>
  </root>
  )";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("navigate_to_pose");

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;

  RegisterRosNode(factory, "./install/mbf_btcpp/share/mbf_btcpp/bt_plugins/libmbf_btcpp_plugin.so", params);

  auto tree = factory.createTreeFromText(xml_text);

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}