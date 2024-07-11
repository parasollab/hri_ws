# Point Cloud to OctoMap Package

This package is written in C++ and provides two nodes indended to facilitate sharing an environment representation between Unity and downstream ROS nodes.

Those nodes are `pointcloud_subscriber` and `pointcloud_publisher`.

## pointcloud_subscriber Node

This node subscribes to the `point_cloud` topic and reads in point cloud data. It converts it to an occupancy grid and writes that occupancy grid to the `maps/map.bt` file. It is recommended to run this node from this directory (`pointcloud_to_map`) to ensure that the map file is written to the `maps` directory here. It will also write out the point cloud data to the `map.pcd` file.

This node also publishes the occupancy grid information on the `octomap_binary` topic.

To run:

```bash
ros2 run pointcloud_to_map pointcloud_subscriber
```

## pointcloud_publisher Node

This node publishes random point cloud data as a means of testing the subscriber node. It should not be used, if real point cloud data is being published from Unity or elsewhere.

To run:

```bash
ros2 run pointcloud_to_map pointcloud_publisher
```

## Visualizing the Occupancy Grid

To visualize the occupancy grid that gets created by the subscriber node, rviz2 can be used. First, install the rviz octomap plugins:

```bash
sudo apt-get install ros-humble-octomap-rviz-plugins
```

Then, open rviz2:

```bash
rviz2
```

This will open the rviz2 gui. Click 'Add' at the bottom of the left panel to add a new display. Then, choose 'OccupancyGrid' and in the left panel change the name of the occupancy grid topic to `octomap_binary`.
