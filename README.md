# ROS2 HRI Workspace

> Note: Tested on ROS2 Jazzy, but likely also supports Humble and Galactic.

This workspace is written for ROS2 and provides packages for the XR HRI project.

## Installing ROS2

> Note for Parasol users: ROS2 Humble is already installed on some of the computers in the lab, however, you may prefer to use your own machine.

We recommend using Ubuntu 24.04 with ROS2 Jazzy, which can be installed using [these instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html).

Alternatively, ROS2 Humble can be installed natively on Ubuntu 22.04 using [these instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

If you are using Windows, you can install Ubuntu 22.04 using Windows Subsystem for Linux and use the above instructions. You can also use a docker image such as [this one](https://hub.docker.com/_/ros) which has ROS preinstalled - make sure to get the Humble version. Alternatively, [this docker image](https://hub.docker.com/r/tiryoh/ros2-desktop-vnc) also provides a graphical display in the browser for using Gazebo/Rviz - again be sure to use the Humble version.

## Setup

Before starting anything, source the ROS2 setup file in each new terminal tab or window that you open. Swap `jazzy` for your ROS2 distribution if necessary.

```bash
source /opt/ros/jazzy/setup.bash
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

In Unity, before building and running your project, go to `Robotics`->`ROS Settings` and change the `ROS IP Address` to the address of the computer that is running ROS (same as in the instruction above).

## Adding a New Package

Follow the [ROS tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) for instructions on how to add a new package in either C++/CMake or Python.

Please add a `README.md` file to the root of your new package describing the nodes/launch files that are available and how to run them.

## Feature Expansive Reward Learning (FERL)

If you would like to run FERL, additionally clone this repository in the `src` directory of the workspace. We do not include it by default due to it's dependencies.

```bash
git clone https://github.com/imngui/ferl_ros2.git
```

### Dependencies

FERL depends on TrajOpt and OpenRave which can be found and installed using the links below.

[TrajOpt](https://github.com/courtneymcbeth/trajopt)

[OpenRave](https://github.com/courtneymcbeth/openrave)

