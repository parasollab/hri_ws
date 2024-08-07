# Trajectory Utilities Package

This package provides utilities for working with robot trajectories. It currently provides a ROS node to subscribe to the `joint_trajectory` topic and writes any collected trajectories to a YAML file. The provided nodes are outlined below.

## trajectory_subscriber Node

This node subscribes to the `joint_trajectory` topic and writes any collected to a YAML file with the provided filename.

To run:

```bash
ros2 run trajectory_utils trajectory_subscriber ./path/to/your/output.yaml
```

## test_publisher Node

This node provides a way to test the `trajectory_subscriber`. It publishes random trajectories for a 3-dof manipulator to the `joint_trajectory` topic. It should not be used if another node is publishing real trajectory data.

To run:

```bash
ros2 run trajectory_utils test_publisher
```

## test_state_publisher Node

This node provides a way to test the mirror functionality for the virtual robot. It publishes random ur5e joint angles to the `physical_joint_state` topic. It should not be used if another node is publishing the real physical robot configurations.

To run:

```bash
ros2 run trajectory_utils test_state_publisher
```
