#!/usr/bin/env python3

import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import smach
import smach_ros

import threading
from geometry_msgs.msg import PoseStamped

from .states.util import WaitForGoal
from .states.mbf import MBFPlanningState, MBFControlState

def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node('mbf_smach')

    executor = SingleThreadedExecutor()
    def spin():
        rclpy.spin(node, executor=executor)

    node.declare_parameter("global_planner", "")
    node.declare_parameter("controller", "")

    global_planner = node.get_parameter("global_planner").get_parameter_value().string_value
    controller = node.get_parameter("controller").get_parameter_value().string_value

    if global_planner == "":
        node.get_logger().warning("Required parameter not set: 'global_planner'!")

    if controller == "":
        node.get_logger().warning("Required parameter not set: 'controller'!")

    base_sm = smach.StateMachine(outcomes=['preempted', 'aborted'])
    with base_sm:
        smach.StateMachine.add(
                'WAIT_FOR_GOAL',
                WaitForGoal(node),
                transitions={
                    'received_goal': 'GET_PATH',
                    'preempted': 'preempted',
                })

        smach.StateMachine.add(
                'GET_PATH',
                MBFPlanningState(node),
                transitions={
                    'succeeded': 'EXE_PATH',
                    'preempted': 'preempted',
                    'aborted': 'aborted'
                }
            )

        smach.StateMachine.add(
                'EXE_PATH',
                MBFControlState(node),
                transitions={
                    'succeeded': 'WAIT_FOR_GOAL',
                    'preempted': 'preempted',
                    'aborted': 'aborted',
                    'failed': 'aborted',
                }
            )

    spinner = threading.Thread(target=spin)
    spinner.start()

    print("Start Introspection")
    sis = smach_ros.IntrospectionServer('navigation', base_sm, '/MBF')
    sis.start()

    outcome = base_sm.execute()
    print("Navigation smach outcome: ", outcome)

    sis.stop()

    # cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
