# mbf_deliberation

This repository provides a collection of examples for calling [move_base_flex (MBF)](https://github.com/naturerobots/move_base_flex) actions, ranging from simple Python and C++ nodes to more sophisticated deliberation libraries, see [awesome-ros-deliberation](https://github.com/ros-wg-delib/awesome-ros-deliberation). All packages in this repository are compatible with any implementation that inherits from MBF, such as [mesh_navigation](https://github.com/naturerobots/mesh_navigation). Each package includes examples showing how MBF can be integrated into the respective framework across different use cases.

For consistency, all packages include a minimalistic node named `navigate_to_pose` which:

1. waits for a goal pose on the `/goal_pose` topic (the default topic to use with RViz),
2. uses a global planner implementation of MBF to plan a path from the robot's pose to the given goal pose. The planner name must be set via the `planner` parameter and must exist in your MBF config,
3. uses a controller implementation of MBF to move the robot along the computed path until it reaches the goal. The controller name must be set via the `controller` parameter and must also exist in your MBF config.

This node does not handle any edge cases and therefore does not trigger recovery behaviors. For more robust edge-case handling, refer to the more advanced examples in each package.

## MBF Simple Nav

### [Python Nodes](mbf_simple_client/mbf_simple_client_py)

Calling MBF actions from plain Python ROS 2 nodes.

```bash
ros2 run mbf_simple_client_py navigate_to_pose --ros-args -p planner:="mesh_planner" -p controller:="mesh_controller"
```

<!-- ### CPP Nodes

TODO -->

## [SMACH](mbf_smach)

Requirements: 
- [executive_smach](https://github.com/ros/executive_smach), available in official apt repositories. *Warning*: Unless PR [#129](https://github.com/ros/executive_smach/pull/129) is not merged use [this](https://github.com/amock/executive_smach/tree/remove-done-cond) on branch `remove-done-cond` and compile from source. Otherwise it will not work (tested for ROS 2 humble).

```bash
ros2 run mbf_smach navigate_to_pose --ros-args -p planner:="mesh_planner" -p controller:="mesh_controller"
```

## [BT.CPP](mbf_btcpp)

Requirements:
- [BT.CPP](https://www.behaviortree.dev/), available in official apt repositories
- [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2), on humble you have to compile this from source

```bash
ros2 run mbf_btcpp navigate_to_pose --ros-args -p planner:="mesh_planner" -p controller:="mesh_controller"
```

## Yasmin

Requirements: 
- [Yasmin](https://github.com/uleroboticsgroup/yasmin), available in official apt repositories

### [Yasmin, Python API](mbf_yasmin/mbf_yasmin_py)

```bash
ros2 run mbf_yasmin_py navigate_to_pose --ros-args -p planner:="mesh_planner" -p controller:="mesh_controller"
```

<details>
<summary>Visualization</summary>

Run visualization server of Yasmin:

```bash
ros2 run yasmin_viewer yasmin_viewer_node
```

Open your web-browser on http://localhost:5000 . For mesh navigation the result should look like this:

![MBF Yasmin](./mbf_yasmin/mbf_yasmin_py/.media/mbf_yasmin.png)

Set a goal via RViz 2D Goal Pose tool and you'll see the state machine transitions in your browser.

</details>

