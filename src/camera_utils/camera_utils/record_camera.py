import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os

class RecordCamera(Node):
    def __init__(self):
        super().__init__('record_camera')

        # Declare and get parameters
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('start_topic', '/record_start')
        self.declare_parameter('image_dir', '/tmp/images')

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        start_topic = self.get_parameter('start_topic').get_parameter_value().string_value
        self.image_dir = self.get_parameter('image_dir').get_parameter_value().string_value

        os.makedirs(self.image_dir, exist_ok=True)

        self.image_sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(Bool, start_topic, self.start_callback, 10)

        self.bridge = CvBridge()
        self.image_count = 0
        self.depth_count = 0
        self.recording = False

        self.get_logger().info(f"Subscribed to {camera_topic} and {depth_topic}")
        self.get_logger().info(f"Saving images to {self.image_dir}")
        self.get_logger().info(f"Listening for start signal on {start_topic}")

    def image_callback(self, msg):
        if not self.recording:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.join(self.image_dir, f'image_{self.image_count}.jpg')
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved image: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")
        self.image_count += 1

    def depth_callback(self, msg):
        if not self.recording:
            return
        try:
            if msg.encoding == "16UC1":
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                filename = os.path.join(self.image_dir, f'depth_{self.depth_count}.png')
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f"Saved depth image: {filename}")
            elif msg.encoding == "32FC1":
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                filename = os.path.join(self.image_dir, f'depth_{self.depth_count}.exr')
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f"Saved depth image: {filename}")
            else:
                self.get_logger().error(f"Unsupported encoding: {msg.encoding}")
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")
        self.depth_count += 1

    def start_callback(self, msg):
        self.recording = not self.recording
        self.get_logger().info(f"Recording: {self.recording}")

def main(args=None):
    rclpy.init(args=args)
    node = RecordCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
