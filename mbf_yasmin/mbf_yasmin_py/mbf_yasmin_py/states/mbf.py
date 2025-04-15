#!/usr/bin/env python3

import rclpy
import yasmin
from yasmin import Blackboard
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL

from mbf_msgs.action import GetPath, ExePath
from geometry_msgs.msg import PoseStamped


class MBFPlanningState(ActionState):
    """
    Class representing the state of the MBF planning action.

    Inherits from ActionState and implements methods to handle the
    MBF actions in a finite state machine.

    Attributes:
        None
    """

    def __init__(self) -> None:
        super(MBFPlanningState, self).__init__(
            GetPath,  # action type
            "/move_base_flex/get_path",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            self.print_feedback,  # callback to process the feedback
        )

        if not self._node.has_parameter("global_planner"):
            self._node.declare_parameter("global_planner", "")


    def create_goal_handler(self, blackboard: Blackboard) -> GetPath.Goal:
        goal = GetPath.Goal()
        goal.use_start_pose = False
        goal.target_pose = blackboard["target_pose"]  # Retrieve the input value 'n' from the blackboard
        goal.planner = self._node.get_parameter("global_planner").get_parameter_value().string_value

        goal.tolerance = 0.2
        return goal

    def response_handler(self, blackboard: Blackboard, response: GetPath.Result) -> str:
        blackboard["path"] = response.path
        return SUCCEED

    def print_feedback(
        self, blackboard: Blackboard, feedback: GetPath.Feedback
    ) -> None:
        yasmin.YASMIN_LOG_INFO(f"Received feedback: {feedback}")

class MBFControlState(ActionState):
    """
    Class representing the state of the MBF planning action.

    Inherits from ActionState and implements methods to handle the
    MBF actions in a finite state machine.

    Attributes:
        None
    """

    def __init__(self) -> None:
        super(MBFControlState, self).__init__(
            ExePath,  # action type
            "/move_base_flex/exe_path",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            self.print_feedback,  # callback to process the feedback
        )

        if not self._node.has_parameter("controller"):
            self._node.declare_parameter("controller", "")

    def create_goal_handler(self, blackboard: Blackboard) -> ExePath.Goal:
        goal = ExePath.Goal()
        goal.dist_tolerance = 0.2  # 20cm tolerance to the target
        goal.path = blackboard["path"]
        goal.controller = self._node.get_parameter("controller").get_parameter_value().string_value  # name of the planner to call see move base flex planners.yaml config
        print("ExePath:", goal)
        return goal

    def response_handler(self, blackboard: Blackboard, response: ExePath.Result) -> str:
        # TODO: handle errors
        return SUCCEED

    def print_feedback(
        self, blackboard: Blackboard, feedback: ExePath.Feedback
    ) -> None:
        yasmin.YASMIN_LOG_INFO(f"Received feedback: {feedback.dist_to_goal}")
