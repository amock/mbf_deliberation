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
              planner_name="{planner_name}" 
              in_pose="{a_goal}" 
              out_path="{a_path}" />
        <MBFControlNode name="MBFControlNode" 
              action_name="move_base_flex/exe_path"
              controller_name="{controller_name}" 
              in_path="{a_path}" />
      </Sequence>
    </BehaviorTree>
  </root>
  )";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("navigate_to_pose");
  nh->declare_parameter("planner", "");
  nh->declare_parameter("controller", "");

  std::string planner_name = nh->get_parameter("planner").as_string();
  std::string controller_name = nh->get_parameter("controller").as_string();

  if(planner_name == "")
  {
    RCLCPP_WARN(nh->get_logger(), "Starting without 'planner'!");
  }

  if(controller_name == "")
  {
    RCLCPP_WARN(nh->get_logger(), "Starting without 'controller'!");
  }

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;

  RegisterRosNode(factory, "./install/mbf_btcpp/share/mbf_btcpp/bt_plugins/libmbf_btcpp_plugin.so", params);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("planner_name", planner_name);
  blackboard->set("controller_name", controller_name);

  auto tree = factory.createTreeFromText(xml_text, blackboard);

  while(rclcpp::ok())
  {
    tree.tickWhileRunning();
  }

  return 0;
}