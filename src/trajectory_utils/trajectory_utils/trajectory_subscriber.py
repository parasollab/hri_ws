import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import csv
import sys
from datetime import datetime
import os

class TrajectorySubscriber(Node):

    def __init__(self, traj_dir, topics):
        super().__init__('trajectory_subscriber')
        self.subs = []
        # Keep track of which CSV files have had headers written
        self.csv_initialized = {}

        for topic in topics:
            # Create a unique filename per topic with a CSV extension.
            now = datetime.now()
            datetime_string = now.strftime("%Y_%m_%d_%H_%M_%S")
            filename = os.path.join(traj_dir, topic.replace('/', '_')[1:] + datetime_string + '.csv')
            self.csv_initialized[filename] = False

            # Use a default argument in the lambda to capture the current filename.
            self.subs.append(
                self.create_subscription(
                    JointTrajectory,
                    topic,
                    lambda msg, fname=filename: self.listener_callback(fname, msg),
                    10
                )
            )

    def listener_callback(self, filename, msg):
        # Build the CSV header from the first received message.
        # The header columns: time info and for each joint: <joint_name>_position, <joint_name>_velocity, <joint_name>_effort.
        header = ['stamp_sec', 'stamp_nanosec']
        for joint in msg.joint_names:
            header.append(f'{joint}_position')
            header.append(f'{joint}_velocity')
            header.append(f'{joint}_effort')

        # Open the file in append mode.
        file_exists = os.path.exists(filename)
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # If the file is new or the header hasn't been written yet, write the header.
            if not file_exists or not self.csv_initialized[filename]:
                writer.writerow(header)
                self.csv_initialized[filename] = True

            # Get the header time stamp from the message header.
            stamp_sec = msg.header.stamp.sec
            stamp_nanosec = msg.header.stamp.nanosec

            # For each trajectory point in the message, write a row.
            for point in msg.points:
                row = [stamp_sec, stamp_nanosec]
                # The joint_names order corresponds to the indices in positions (and optionally in velocities and effort).
                for i, joint in enumerate(msg.joint_names):
                    # Always record position (if available).
                    pos = point.positions[i] if i < len(point.positions) else ''
                    # Record velocity if available, otherwise leave blank.
                    vel = point.velocities[i] if (point.velocities and i < len(point.velocities)) else ''
                    # Record effort if available, otherwise leave blank.
                    eff = point.effort[i] if (point.effort and i < len(point.effort)) else ''
                    row.extend([pos, vel, eff])
                writer.writerow(row)
        self.get_logger().info(f'Appended JointTrajectory data to {filename}')

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) > 2:
        traj_dir = sys.argv[1]
        topics = sys.argv[2:]
    else:
        print("Usage: ros2 run trajectory_utils trajectory_subscriber <traj_dir> <topics>")
        return
    node = TrajectorySubscriber(traj_dir, topics)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
