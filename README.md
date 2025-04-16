# mbf_deliberation

This repository provides a collection of examples for calling [move_base_flex (MBF)](https://github.com/naturerobots/move_base_flex) actions, ranging from simple Python and C++ nodes to more sophisticated deliberation libraries like [awesome-ros-deliberation](https://github.com/ros-wg-delib/awesome-ros-deliberation). All packages in this repository are compatible with any implementation that inherits from MBF, such as [mesh_navigation](https://github.com/naturerobots/mesh_navigation). Each package includes examples showing how MBF can be integrated into the respective framework across different use cases.

For consistency, all packages include a minimalistic node named `navigate_to_pose` which:

1. waits for a goal pose on the `/goal_pose` topic (the default topic to use with RViz),
2. uses a global planner implementation of MBF to plan a path from the robot's pose to the given goal pose. The planner name must be set via the `global_planner` parameter and must exist in your MBF config,
3. uses a controller implementation of MBF to move the robot along the computed path until it reaches the goal. The controller name must be set via the `controller` parameter and must also exist in your MBF config.

This node does not handle any edge cases and therefore does not trigger recovery behaviors. For more robust edge-case handling, refer to the more advanced examples in each package.

## MBF Simple Nav

### Python Nodes

Calling MBF actions from plain Python ROS 2 nodes.

See more examples here: [mbf_simple_client/mbf_simple_client_py](mbf_simple_client/mbf_simple_client_py) . 

```bash
ros2 run mbf_simple_client_py navigate_to_pose --ros-args -p global_planner:="mesh_planner" -p controller:="mesh_controller"
```

Replace the values for `global_planner` and `controller` with any planner or controller you defined in your MBF config.

Publish a pose on topic `/goal_pose`.

### CPP Node

TODO

## SMACH

Requirements: 
- executive_smach

Link: https://github.com/ros/executive_smach

*Warning*: Unless PR [#129](https://github.com/ros/executive_smach/pull/129) is not merged use [this](https://github.com/amock/executive_smach/tree/remove-done-cond) on branch `remove-done-cond`. Otherwise it will not work (tested for ROS 2 humble).

```bash
ros2 run mbf_smach navigate_to_pose --ros-args -p global_planner:="mesh_planner" -p controller:="mesh_controller"
```

(I haven't found a viewer that is working out of the box)

## BT.CPP

TODO

## Yasmin

Requirements: 
- [Yasmin](https://github.com/uleroboticsgroup/yasmin), available in official apt repositories

### Yasmin, Python API

```bash
ros2 run mbf_yasmin_py navigate_to_pose --ros-args -p global_planner:="mesh_planner" -p controller:="mesh_controller"
```

Run visualization server of Yasmin:

```bash

```




