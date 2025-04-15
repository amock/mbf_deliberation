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
class WaitForGoal(smach.State):

    def __init__(self, node):
        super(WaitForGoal, self).__init__(
            outcomes=['received_goal', 'preempted'],
            input_keys=[],
            output_keys=['target_pose'])
        self.target_pose = PoseStamped()
        self.signal = threading.Event()
        self.subscriber = None
        self.node = node

    def execute(self, user_data):

        print("Waiting for a goal...")
        self.signal.clear()
        self.subscriber = self.node.create_subscription(
            PoseStamped, 
            'goal_pose',
            self.goal_callback, 10)

        while rclpy.ok() and not self.signal.is_set() and not self.preempt_requested():
            print("Waiting for a goal...")
            self.signal.wait(1)
            rclpy.spin_once(self.node)

        if self.preempt_requested() or not rclpy.ok():
            self.service_preempt()
            return 'preempted'

        user_data.target_pose = self.target_pose
        pos = self.target_pose.pose.position
        self.node.get_logger().info("Received goal pose: (%f, %f, %f)" % (pos.x, pos.y, pos.z) )

        return 'received_goal'

    def goal_callback(self, msg):
        print("Received goal pose: ", str(msg))
        self.target_pose = msg
        self.signal.set()

    def request_preempt(self):
        smach.State.request_preempt(self)
        self.signal.set()