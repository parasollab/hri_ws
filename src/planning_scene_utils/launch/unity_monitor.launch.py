from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planning_scene_utils',
            executable='planning_scene_watcher',
            name='planning_scene_watcher',
            output='screen',
        ),
        Node(
            package='planning_scene_utils',
            executable='latency_measurer',
            name='latency_measurer',
            output='screen',
        ),
        Node(
            package='planning_scene_utils',
            executable='fps_monitor',
            name='fps_monitor',
            output='screen',
        ),
    ])
