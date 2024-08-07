import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random
from time import sleep

class TestPublisher(Node):

    def __init__(self):
        super().__init__('test_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/physical_joint_state', 1)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wrist_3_link', 'wrist_2_link', 'wrist_1_link', 'forearm_link', 'upper_arm_link', 'shoulder_link']
        msg.position = [random.uniform(0, 360.0) for _ in range(6)]
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
