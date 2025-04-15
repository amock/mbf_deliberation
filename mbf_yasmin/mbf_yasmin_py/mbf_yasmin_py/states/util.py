#!/usr/bin/env python3

import rclpy
import yasmin
from geometry_msgs.msg import PoseStamped
from yasmin import Blackboard
from yasmin_ros import MonitorState

from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

class WaitForPose(MonitorState):
    """
    WaitForPose
    """

    def __init__(self) -> None:
        super(WaitForPose, self).__init__(
            PoseStamped,  # msg type
            "/goal_pose",  # topic name
            [SUCCEED, ABORT, CANCEL],  # outcomes
            self.monitor_handler,  # monitor handler callback
            msg_queue=10,  # queue for the monitor handler callback
        )

    def monitor_handler(self, blackboard: Blackboard, msg: PoseStamped) -> str:
        yasmin.YASMIN_LOG_INFO(msg)

        blackboard["target_pose"] = msg
        return SUCCEED
