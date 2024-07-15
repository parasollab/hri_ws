# ROS2 HRI Workspace

This workspace is written for ROS2 Humble and provides packages for the XR HRI project.

## Installing ROS2 Humble

> Note: ROS2 Humble is already installed on some of the computers in the lab, however, you may prefer to use your own machine.

ROS2 Humble can be installed natively on Ubuntu 22.04 using [these instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

If you are using Windows, you can install Ubuntu 22.04 using Windows Subsystem for Linux and use the above instructions. You can also use a docker image such as [this one](https://hub.docker.com/_/ros) which has ROS preinstalled - make sure to get the Humble version. Alternatively, [this docker image](https://hub.docker.com/r/tiryoh/ros2-desktop-vnc) also provides a graphical display in the browser for using Gazebo/Rviz - again be sure to use the Humble version.

## Setup

Before starting anything, source the ROS2 Humble setup file in each new terminal tab or window that you open:

```bash
source /opt/ros/humble/setup.bash
```

Each node that you run will require you to open a new terminal tab or window.

Initialize and update the git submodules:

```bash
git submodule init
git submodule update
```

## Building this Workspace

To build this workspace, run:

```bash
colcon build
```

If you'd like to only build a specific package, you can use this command:

```bash
colcon build --packages-select <package_name>
```

After building, run the setup file for this workspace. Do this in every terminal tab or window that you open.

```bash
source install/setup.bash
```

## Run the ROS TCP Endpoint for Unity

Run this before any other ROS node if you are connecting to Unity (replace the IP with your computer's IP):

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

In Unity, before building and running your project, go to `Robotics`->`ROS Settings` and change the `ROS_IP` address to the address of the computer that is running ROS (same as in the instruction above).

## Adding a New Package

Follow the [ROS tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) for instructions on how to add a new package in either C++/CMake or Python.

Please add a `README.md` file to the root of your new package describing the nodes/launch files that are available and how to run them.