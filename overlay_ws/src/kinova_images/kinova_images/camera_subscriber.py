import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import message_filters  # Import message_filters for synchronization
import cv2


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # Create subscribers for color and depth_registered image topics
        color_image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        depth_registered_image_sub = message_filters.Subscriber(self, Image, '/camera/depth_registered/image_rect')

        # Synchronize the color and depth_registered image topics
        ts = message_filters.ApproximateTimeSynchronizer([color_image_sub, depth_registered_image_sub], 10, 0.1)
        ts.registerCallback(self.image_callback)

    def image_callback(self, color_msg, depth_msg):
        # Convert both messages to NumPy arrays
        color_image = self.image_msg_to_numpy(color_msg)  # RGB image: (height, width, 3)
        depth_image = self.image_msg_to_numpy(depth_msg)  # Depth image: (height, width)

        # Ensure depth image is reshaped to match the height and width of the color image
        if color_image.shape[:2] != depth_image.shape[:2]:
            self.get_logger().error('Color and depth images do not have matching sizes.')
            return

        # Normalize depth image to 0-255 and convert it to uint8 (optional)
        # depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # Stack the color and depth images along the last axis to create a (height, width, 4) array
        rgbd_image = np.dstack((color_image, depth_image))  # Shape: (height, width, 4)

        # Save the combined RGB-D image as a NumPy array
        np.save('/kinova-ros2/image_transfer/rgbd_image.npy', rgbd_image)

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
