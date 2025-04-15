# mbf_deliberation

This repository aims to provide a collection of examples for calling MBF actions, ranging from simple Python / C++ nodes to more sophisticated delibration libraries: https://github.com/ros-wg-delib/awesome-ros-deliberation.
*Note*: This repository is compatible with every implementation listed in MBF, such as [mesh_navigation](github.com/naturerobots/mesh_navigation), ...


## MBF Simple Nav

### Python Nodes

Calling MBF actions from plain Python ROS 2 nodes.

See more examples here: [mbf_simple_client/mbf_simple_client_py](mbf_simple_client/mbf_simple_client_py) . 

```bash
ros2 run mbf_simple_client_py navigate_to_goal --ros-args -p global_planner:="mesh_planner" controller:="mesh_controller"
```

Replace the values for `global_planner` and `controller` with any planner or controller you defined in your MBF config.

Publish a pose on topic `/goal_pose`.

### CPP Node

TODO

## SMACH

Requirements: executive_smach

Link: https://github.com/ros/executive_smach

## BT.CPP

TODO

## Yasmin

Link: https://github.com/uleroboticsgroup/yasmin






