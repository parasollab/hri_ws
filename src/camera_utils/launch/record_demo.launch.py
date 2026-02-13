from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    demo_name = LaunchConfiguration('demo_name')

    return LaunchDescription([
        DeclareLaunchArgument('demo_name', default_value='test'),

        Node(
            package='trajectory_utils',
            executable='trajectory_subscriber',
            name='trajectory_subscriber',
            parameters=[{
                'traj_dir': '/home/courtney/hri_ws/src/trajectory_utils/trajectories',
                'topics': "['/left/joint_trajectory', '/right/joint_trajectory']"
            }],
            output='screen'
        ),

        Node(
            package='camera_utils',
            executable='pointcloud_subscriber',
            name='pointcloud_recorder',
            parameters=[{
                'topic': '/camera/camera/depth/color/points',
                'output_dir': PathJoinSubstitution([
                    TextSubstitution(text='/home/courtney/hri_ws/src/camera_utils/data/'),
                    demo_name
                ]),
                'start_topic': '/record_start'
            }],
            output='screen'
        ),

        Node(
            package='camera_utils',
            executable='record_camera',
            name='record_camera_node',
            parameters=[{
                'camera_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'start_topic': '/record_start',
                'image_dir': PathJoinSubstitution([
                    TextSubstitution(text='/home/courtney/hri_ws/src/camera_utils/data/'),
                    demo_name
                ])
            }],
            output='screen'
        )
    ])
