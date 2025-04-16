#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "behaviortree_ros2/plugins.hpp"



using namespace BT;

// Simple tree, used to execute once each action.
static const char* xml_text = R"(
  <root BTCPP_format="4">
      <BehaviorTree>
         <Sequence>
            <WaitForPose name="WaitForPose" topic_name="/goal_pose" goal_pose="{goal_pose}"/>
            <MBFPlanningNode name="MBFPlanningNode" action_name="move_base_flex/get_path" goal_pose="{goal_pose}" path="{path}" />
            <MBFControlNode name="MBFControlNode" action_name="move_base_flex/exe_path" path="{path}" />
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
  // params.default_port_value = "btcpp_string";

  RegisterRosNode(factory, "./install/mbf_btcpp/share/mbf_btcpp/bt_plugins/libmbf_btcpp_plugin.so", params);

  auto tree = factory.createTreeFromText(xml_text);

  for(int i = 0; i < 5; i++)
  {
    tree.tickWhileRunning();
  }

  return 0;
}