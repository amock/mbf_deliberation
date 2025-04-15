#!/usr/bin/env python3

import rclpy
import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_ros import ActionState, MonitorState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from mbf_msgs.action import GetPath, ExePath
from geometry_msgs.msg import PoseStamped

from .states.util import WaitForPose
from .states.mbf import MBFPlanningState, MBFControlState


def main():
    """
    Main function to execute the ROS 2 action client demo.

    This function initializes the ROS 2 client, sets up the finite state
    machine, adds the states, and starts the action processing.

    Parameters:
        None

    Returns:
        None

    Raises:
        KeyboardInterrupt: If the user interrupts the execution.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state(
        "WAIT_FOR_POSE",
        WaitForPose(),
        transitions={
            SUCCEED: "GET_PATH",
            CANCEL: "outcome4",
            ABORT: "outcome4"
        }
    )

    sm.add_state(
        "GET_PATH",
        MBFPlanningState(),
        transitions={
            SUCCEED: "EXE_PATH",
            CANCEL: "outcome4",
            ABORT: "outcome4",
        },
    )

    sm.add_state(
        "EXE_PATH",
        MBFControlState(),
        transitions={
            SUCCEED: "WAIT_FOR_POSE",
            CANCEL: "outcome4",
            ABORT: "outcome4",
        },
    )

    # Publish FSM information
    YasminViewerPub("MBF_YASMIN", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()

    # Execute the FSM
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()