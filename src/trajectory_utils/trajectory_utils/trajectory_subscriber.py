import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import csv
from datetime import datetime
import os
import ast

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')

        # Declare and get parameters
        self.declare_parameter('traj_dir', '/tmp')
        self.declare_parameter('topics', "['/left/joint_trajectory', '/right/joint_trajectory']")

        traj_dir = self.get_parameter('traj_dir').get_parameter_value().string_value
        topics_param = self.get_parameter('topics').get_parameter_value().string_value

        # Convert the stringified list (from launch file or CLI) into a Python list
        topics = ast.literal_eval(topics_param) if isinstance(topics_param, str) else topics_param

        self.get_logger().info(f"Saving trajectory data to: {traj_dir}")

        self.subs = []
        self.csv_initialized = {}

        for topic in topics:
            now = datetime.now()
            datetime_string = now.strftime("%Y_%m_%d_%H_%M_%S")
            filename = os.path.join(traj_dir, topic.replace('/', '_')[1:] + '_' + datetime_string + '.csv')
            self.csv_initialized[filename] = False

            self.subs.append(
                self.create_subscription(
                    JointTrajectory,
                    topic,
                    lambda msg, fname=filename: self.listener_callback(fname, msg),
                    10
                )
            )

            self.get_logger().info(f"Subscribed to {topic}")

    def listener_callback(self, filename, msg):
        header = ['stamp_sec', 'stamp_nanosec']
        for joint in msg.joint_names:
            header.append(f'{joint}_position')
            header.append(f'{joint}_velocity')
            header.append(f'{joint}_effort')

        file_exists = os.path.exists(filename)
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists or not self.csv_initialized[filename]:
                writer.writerow(header)
                self.csv_initialized[filename] = True

            stamp_sec = msg.header.stamp.sec
            stamp_nanosec = msg.header.stamp.nanosec

            for point in msg.points:
                row = [stamp_sec, stamp_nanosec]
                for i, joint in enumerate(msg.joint_names):
                    pos = point.positions[i] if i < len(point.positions) else ''
                    vel = point.velocities[i] if point.velocities and i < len(point.velocities) else ''
                    eff = point.effort[i] if point.effort and i < len(point.effort) else ''
                    row.extend([pos, vel, eff])
                writer.writerow(row)
        self.get_logger().info(f'Appended JointTrajectory data to {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
