import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import message_filters  # Import message_filters for synchronization
import cv2


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # Create subscribers for color and depth_registered image topics
        color_image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        depth_registered_image_sub = message_filters.Subscriber(self, Image, '/camera/depth_registered/image_rect')
        self.camera_info_sub = self.create_subscription(CameraInfo, 
                                                        '/camera/depth_registered/camera_info', 
                                                        self.camera_info_callback, 
                                                        10)
        
        # Initialize camera matrix to None
        self.camera_matrix = None

        # Synchronize the color and depth_registered image topics
        ts = message_filters.ApproximateTimeSynchronizer([color_image_sub, depth_registered_image_sub], 10, 0.1)
        ts.registerCallback(self.image_callback)

    def camera_info_callback(self, msg):
        """Extract and save the camera matrix K in the required flat format."""
        # Extracting the intrinsic camera matrix from the CameraInfo message
        K = msg.k  # K is already a 9-element array in row-major order
        self.camera_matrix = [K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8]]
        self.get_logger().info(f"Camera Matrix: {self.camera_matrix}")

    def image_callback(self, color_msg, depth_msg):
        # Convert both messages to NumPy arrays
        color_image = self.image_msg_to_numpy(color_msg)  # RGB image: (height, width, 3)
        depth_image = self.image_msg_to_numpy(depth_msg)  # Depth image: (height, width)

        # Ensure depth image is reshaped to match the height and width of the color image
        if color_image.shape[:2] != depth_image.shape[:2]:
            self.get_logger().error('Color and depth images do not have matching sizes.')
            return

        combined_data = {
            'rgb': color_image,
            'depth': depth_image/1000,  # Convert depth values from mm to meters
            'K': self.camera_matrix
        }

        # Print the combined data using logger (shape of arrays and camera matrix values)
        self.get_logger().info(f"Combined Data:\nRGB Shape: {combined_data['rgb'].shape}\n"
                                f"RGB stats: {np.min(combined_data['rgb'])}, {np.max(combined_data['rgb'])}\n"
                                f"Depth Shape: {combined_data['depth'].shape}\n"
                                f"Depth stats: {np.min(combined_data['depth'])}, {np.max(combined_data['depth'])}\n"
                                f"Camera Matrix: {combined_data['K']}")
        
        # Save the combined RGB-D image as a NumPy array
        np.savez('/kinova-ros2/image_transfer/rgbd_image.npz', **combined_data)

        self.get_logger().info('Saved RGB-D image (4 channels).')

    def image_msg_to_numpy(self, msg):
        """Convert ROS Image message to NumPy array."""
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        if encoding == 'rgb8':
            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
        elif encoding == 'mono16' or encoding == '16UC1':  # Handle 16-bit single-channel images
            np_array = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
        else:
            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))

        return np_array


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
