# Camera Utils

Prerequisite: Install [ROS2 Realsense SDK](https://github.com/realsenseai/realsense-ros)

Run this to start the camera:

```bash
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true 
```

Then launch the demo recorder:

```bash
ros2 launch camera_utils record_demo.launch.py
```

Publish to the `/record_start` topic to start recording:

```bash
ros2 topic pub --once /record_start std_msgs/msg/Bool "{data: true}"
```

Publish the above command again to stop.