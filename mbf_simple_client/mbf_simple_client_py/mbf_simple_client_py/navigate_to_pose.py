#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from mbf_msgs.action import GetPath
from mbf_msgs.action import ExePath

import threading
from geometry_msgs.msg import PoseStamped


"""
TODO: docs
"""
class GetPathClient:

    def __init__(self, node):
        self.node_ = node

        self.node_.declare_parameter("planner", "mesh_planner")
        self.planner_ = node.get_parameter('planner').get_parameter_value().string_value
        if self.planner_ == "":
            node.get_logger().error("Please set required parameter 'planner'")
            exit()

        self._get_path_client = ActionClient(node, 
            GetPath,
            '/move_base_flex/get_path')

        self.on_finished = None

    def get_path(self, pose):
        # define goal
        goal_msg = GetPath.Goal()
        goal_msg.use_start_pose = False
        goal_msg.tolerance = 0.2  # 20cm tolerance to the target
        goal_msg.target_pose = pose
        goal_msg.planner = self.planner_  # name of the planner to call see move base flex planners.yaml config

        self._get_path_client.wait_for_server()
        self._send_goal_future = self._get_path_client.send_goal_async(
            goal_msg
        )
        self._send_goal_future.add_done_callback(
            self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node_.get_logger().info('Goal rejected')
            return
        self.node_.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_cb)

    def get_result_cb(self, future):
        result  = future.result().result
        self.node_.get_logger().info(f'Result: {result}')
        if not self.on_finished is None:
            self.on_finished(result)

"""
TODO: docs
"""
class ExePathClient:

    def __init__(self, node):
        self.node_ = node

        self.node_.declare_parameter("controller", "mesh_controller")
        self.controller_ = node.get_parameter('controller').get_parameter_value().string_value
        if self.controller_ == "":
            node.get_logger().error("Please set required parameter 'controller'")
            exit()

        self._exe_path_client = ActionClient(node, 
            ExePath,
            '/move_base_flex/exe_path')

        self.on_finished = None

    def exe_path(self, path):
        exec_path_goal = ExePath.Goal()
        exec_path_goal.path = path
        exec_path_goal.controller = self.controller_

        self._exe_path_client.wait_for_server()
        self._send_goal_future = self._exe_path_client.send_goal_async(
            exec_path_goal,
            feedback_callback=self.feedback_cb
        )
        self._send_goal_future.add_done_callback(
            self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node_.get_logger().info('Goal rejected')
            return
        self.node_.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_cb)

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node_.get_logger().info(f'Received feedback {feedback.dist_to_goal}')

    def get_result_cb(self, future):
        result  = future.result().result
        self.node_.get_logger().info(f'Result: {result}')
        if not self.on_finished is None:
            self.on_finished(result)


class MBFSimpleClientNode(Node):

    def get_path_finished(self, result):
        # continue
        if result.outcome == result.SUCCESS:
            self.get_logger().info("YES! Got path. Next: Drive along path")
            self._exe_path_client.exe_path(result.path)
        else:
            self.get_logger().error(f'Error: {result}')

    def exe_path_finished(self, result):
        self.get_logger().info("Finished!")

    def __init__(self):
        super().__init__('mbf_simple_client_node')

        self._get_path_client = GetPathClient(self)
        self._exe_path_client = ExePathClient(self)

        # make transitions
        self._get_path_client.on_finished = self.get_path_finished
        self._exe_path_client.on_finished = self.exe_path_finished

        self._subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_cb,
            10)
        
        self.get_logger().info("Waiting for goal...")

    def goal_pose_cb(self, pose):
        self.get_logger().info('Sending goal to MBF: "%s"' % pose)
        self._get_path_client.get_path(pose)


def main(args=None):
    rclpy.init(args=args)

    action_client = MBFSimpleClientNode()


    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
