import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import sys
import numpy as np
import pickle

class NumpyTrajectorySubscriber(Node):

    def __init__(self, filename):
        super().__init__('numpy_trajectory_subscriber')
        self.subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.filename = filename
        self.trajectories = []
        self.include_velocity = False

    def listener_callback(self, msg):
        trajectory = self.parse_msg(msg)
        self.trajectories.append(trajectory)
        with open(self.filename, 'wb') as f:
            pickle.dump(self.trajectories, f)
        self.get_logger().info(f'New JointTrajectory added to {self.filename}')

    def parse_msg(self, msg):
        positions = np.array([point.positions for point in msg.points])
        velocities = np.array([point.velocities for point in msg.points])

        if self.include_velocity:
            # Concatenate positions and velocities
            trajectory = np.concatenate((positions, velocities), axis=1)
        else:
            trajectory = positions

        return trajectory

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        print("Usage: ros2 run trajectory_utils numpy_trajectory_subscriber <filename>")
        return
    node = NumpyTrajectorySubscriber(filename)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
