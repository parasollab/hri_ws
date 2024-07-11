import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
from time import sleep

class TestPublisher(Node):

    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint1', 'joint2', 'joint3']
        point = JointTrajectoryPoint()
        point.positions = [random.uniform(-1.0, 1.0) for _ in range(3)]
        point.velocities = [random.uniform(-1.0, 1.0) for _ in range(3)]
        point.accelerations = [random.uniform(-1.0, 1.0) for _ in range(3)]
        point.effort = [random.uniform(-1.0, 1.0) for _ in range(3)]
        point.time_from_start.sec = random.randint(0, 10)
        point.time_from_start.nanosec = random.randint(0, 1000000000)
        msg.points = [point]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
