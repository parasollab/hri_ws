import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class RecordCamera(Node):
    def __init__(self):
        super().__init__('record_camera')

        # Declare and get parameters
        self.declare_parameter('camera_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('start_topic', '/record_start')
        self.declare_parameter('image_dir', '/tmp/images')

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        start_topic = self.get_parameter('start_topic').get_parameter_value().string_value
        self.image_dir = self.get_parameter('image_dir').get_parameter_value().string_value

        # Create BundleSDF directory structure
        self.rgb_dir = os.path.join(self.image_dir, 'rgb')
        self.depth_dir = os.path.join(self.image_dir, 'depth')
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        self.image_sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Bool, start_topic, self.start_callback, 10)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.recording = False
        self.camera_intrinsics_saved = False

        self.get_logger().info(f"Subscribed to {camera_topic}, {depth_topic}, and {camera_info_topic}")
        self.get_logger().info(f"Saving RGB images to {self.rgb_dir}")
        self.get_logger().info(f"Saving depth images to {self.depth_dir}")
        self.get_logger().info(f"Listening for start signal on {start_topic}")

    def camera_info_callback(self, msg):
        # Save camera intrinsics once to cam_K.txt
        if not self.camera_intrinsics_saved:
            cam_K_path = os.path.join(self.image_dir, 'cam_K.txt')
            K = np.array(msg.k).reshape(3, 3)
            np.savetxt(cam_K_path, K, fmt='%.6f')
            self.camera_intrinsics_saved = True
            self.get_logger().info(f"Saved camera intrinsics to {cam_K_path}")

    def image_callback(self, msg):
        if not self.recording:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Save as PNG (OpenCV handles BGR correctly)
            filename = os.path.join(self.rgb_dir, f'{self.frame_count:04d}.png')
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved RGB image: {filename}")
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

    def depth_callback(self, msg):
        if not self.recording:
            return
        try:
            if msg.encoding == "16UC1":
                # Depth already in uint16, assume it's in millimeters
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            elif msg.encoding == "32FC1":
                # Depth in meters (float32), convert to millimeters (uint16)
                cv_image_float = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                # Convert meters to millimeters and cast to uint16
                cv_image = (cv_image_float * 1000.0).astype(np.uint16)
            else:
                self.get_logger().error(f"Unsupported depth encoding: {msg.encoding}")
                return

            # Use same frame count as RGB for matching filenames
            filename = os.path.join(self.depth_dir, f'{self.frame_count:04d}.png')
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved depth image: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

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
