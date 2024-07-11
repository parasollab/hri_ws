import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import yaml
from yaml.representer import SafeRepresenter
import sys

class TrajectorySubscriber(Node):

    def __init__(self, filename):
        super().__init__('trajectory_subscriber')
        self.subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.filename = filename
        self.trajectories = []

    def listener_callback(self, msg):
        trajectory_dict = self.trajectory_to_dict(msg)
        self.trajectories.append(trajectory_dict)
        with open(self.filename, 'w') as f:
            yaml.dump({'trajectories': self.trajectories}, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f'New JointTrajectory added to {self.filename}')

    def trajectory_to_dict(self, msg):
        return {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id
            },
            'joint_names': msg.joint_names,
            'points': [
                {
                    'positions': point.positions,
                    'velocities': point.velocities,
                    'accelerations': point.accelerations,
                    'effort': point.effort,
                    'time_from_start': {
                        'sec': point.time_from_start.sec,
                        'nanosec': point.time_from_start.nanosec
                    }
                } for point in msg.points
            ]
        }

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        print("Usage: ros2 run trajectory_utils trajectory_subscriber <filename>")
        return
    node = TrajectorySubscriber(filename)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
