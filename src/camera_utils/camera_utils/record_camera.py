import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
import sys

class RecordCamera(Node):
    def __init__(self, camera_topic, depth_topic, image_dir, start_topic):
        super().__init__('record_camera')
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10
        )
        self.image_dir = image_dir
        self.image_count = 0
        self.depth_count = 0

        self.recording = False
        self.create_subscription(
            Bool,
            start_topic,
            self.start_callback,
            10
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):
        if not self.recording:
            return

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Save the image to a file
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
            # Convert ROS Image message to OpenCV format
            if msg.encoding == "16UC1":
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                filename = os.path.join(self.image_dir, f'depth_{self.depth_count}.png')
                cv2.imwrite(filename, cv_image)  # Save directly as PNG
                self.get_logger().info(f"Depth image saved as {filename}")

            elif msg.encoding == "32FC1":
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                filename = os.path.join(self.image_dir, f'depth_{self.depth_count}.exr')
                cv2.imwrite(filename, cv_image)  # Save as EXR or TIFF for floating point
                self.get_logger().info(f"Depth image saved as {filename}")

            else:
                self.get_logger().error(f"Unsupported encoding: {msg.encoding}")

        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

        self.depth_count += 1

    def start_callback(self, msg):
        self.recording = not self.recording

def main(args=None):
    rclpy.init(args=args)

    camera_topic = '/camera/camera/color/image_raw'
    depth_topic = '/camera/camera/depth/image_rect_raw'
    start_topic = '/record_start'

    if len(sys.argv) > 1:
        image_dir = sys.argv[1]
    else:
        print("Usage: ros2 run camera_utils record_camera <image_dir>")
        return
    
    node = RecordCamera(camera_topic, depth_topic, image_dir, start_topic)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

