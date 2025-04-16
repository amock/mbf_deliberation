#!/usr/bin/env python3

import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import smach
import smach_ros

from mbf_msgs.action import GetPath
from mbf_msgs.action import ExePath
# from mbf_msgs.action import Recovery # TODO: add recovery
import threading
from geometry_msgs.msg import PoseStamped


"""
TODO: Docs
"""
class MBFPlanningState(smach_ros.SimpleActionState):

    def __init__(self, node):
        super(MBFPlanningState, self).__init__(
            node,
            '/move_base_flex/get_path',
            GetPath,
            goal_cb=self.goal_cb,
            result_cb=self.result_cb,
            input_keys=['target_pose'],
            output_keys=['outcome', 'message', 'path', 'cost', 'recovery_behavior'],
            outcomes=['succeeded', 'preempted', 'aborted'])
        
    def goal_cb(self, user_data, goal : GetPath.Goal):
        print("Plan!")
        
        goal.use_start_pose = False
        goal.tolerance = 0.2  # 20cm tolerance to the target
        goal.target_pose = user_data.target_pose
        goal.planner = self.node.get_parameter("planner").get_parameter_value().string_value  # name of the planner to call see move base flex planners.yaml config

        print("Return goal!")

        return goal

    def result_cb(self, user_data, status, result : GetPath.Result):

        print("Got plan!")

        print(status)

        user_data.message = result.message
        user_data.outcome = result.outcome
        user_data.path = result.path
        user_data.cost = result.cost

        if result.outcome == GetPath.Result.SUCCESS:
            return 'succeeded'
        elif result.outcome == GetPath.Result.CANCELED:
            return 'preempted'
        else:
            # RECOVERY!
            user_data.recovery_behavior = 'clear_costmap'
            print("Set recovery behavior to %s", user_data.recovery_behavior)
            return 'aborted'
        

"""
TODO: Docs
"""
class MBFControlState(smach_ros.SimpleActionState):

    def __init__(self, node):
        super(MBFControlState, self).__init__(
            node,
            '/move_base_flex/exe_path',
            ExePath,
            goal_cb=self.goal_cb,
            result_cb=self.result_cb,
            input_keys=['path'],
            output_keys=['path', 'outcome', 'message', 'final_pose', 'dist_to_goal', 'angle_to_goal', 'recovery_behavior'],
            outcomes=['succeeded', 'aborted', 'failed', 'preempted'])

    def goal_cb(self, user_data, goal : ExePath.Goal):
        print("Execute!")
        
        goal.dist_tolerance = 0.2  # 20cm tolerance to the target
        goal.path = user_data.path
        goal.controller = self.node.get_parameter("controller").get_parameter_value().string_value  # name of the planner to call see move base flex planners.yaml config

        print("Return goal!")
        return goal
    
    def result_cb(self, user_data, status, result : ExePath.Result):
        print("Finished!")

        outcome_map = {
            ExePath.Result.COLLISION: 'COLLISION',
            ExePath.Result.CANCELED: 'CANCELED',
            ExePath.Result.BLOCKED_PATH: 'BLOCKED_PATH',
            ExePath.Result.FAILURE: 'FAILURE',
            ExePath.Result.INTERNAL_ERROR: 'INTERNAL_ERROR',
            ExePath.Result.INVALID_PATH: 'INVALID_PATH',
            ExePath.Result.MISSED_GOAL: 'MISSED_GOAL',
            ExePath.Result.INVALID_PLUGIN: 'INVALID_PLUGIN',
            ExePath.Result.MISSED_PATH: 'MISSED_PATH',
            ExePath.Result.NO_VALID_CMD: 'NO_VALID_CMD',
            ExePath.Result.NOT_INITIALIZED: 'NOT_INITIALIZED',
            ExePath.Result.OSCILLATION: 'OSCILLATION',
            ExePath.Result.PAT_EXCEEDED: 'PAT_EXCEEDED',
            ExePath.Result.ROBOT_STUCK: 'ROBOT_SUCK',
            ExePath.Result.TF_ERROR: 'TF_ERROR',
            ExePath.Result.SUCCESS: 'SUCCESS',
        }

        # unused?
        # controller_aborted_map = [
        #     ExePath.Result.TF_ERROR,
        #     ExePath.Result.INTERNAL_ERROR,
        #     ExePath.Result.INVALID_PATH,
        #     ExePath.Result.NOT_INITIALIZED,
        # ]

        controller_failed_map = [
            ExePath.Result.PAT_EXCEEDED,
            ExePath.Result.BLOCKED_PATH,
            ExePath.Result.FAILURE,
            ExePath.Result.MISSED_PATH,
            ExePath.Result.MISSED_GOAL,
            ExePath.Result.NO_VALID_CMD,
            ExePath.Result.OSCILLATION,
            ExePath.Result.ROBOT_STUCK,
        ]

        user_data.outcome = result.outcome
        user_data.message = result.message
        user_data.final_pose = result.final_pose
        user_data.dist_to_goal = result.dist_to_goal
        user_data.angle_to_goal = result.angle_to_goal
    
        # here we could select a recovery behavior 
        # best suited for the situation
        recovery_behavior = 'dummy_recovery'

        if result.outcome == ExePath.Result.SUCCESS:
            p = result.final_pose.pose.position
            self.node.get_logger().info("Controller arrived at goal: (%s), %s, %s" % (str(p), outcome_map[result.outcome], result.message))
            return 'succeeded'
        elif result.outcome == ExePath.Result.CANCELED:
            self.node.get_logger().info("Controller has been canceled.")
            return 'preempted'
        elif result.outcome in controller_failed_map:
            self.node.logwarn("Controller failed: %s, %s", outcome_map[result.outcome], result.message)
            user_data.recovery_behavior = recovery_behavior
            self.node.get_logger().info("Set recovery behavior to %s", recovery_behavior)
            return 'failed'
        else:
            self.node.fatal("Controller aborted: %s, %s", outcome_map[result.outcome], result.message)
            return 'aborted'
