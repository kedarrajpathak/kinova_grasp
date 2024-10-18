import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import numpy as np
import cv2
import struct

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.create_subscription(Image, '/camera/color/image_raw', self.color_image_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_image_callback, 10)
        self.create_subscription(Image, '/camera/depth_registered/image_rect', self.depth_registered_image_callback, 10)
        self.create_subscription(PointCloud2, '/camera/depth/color/points', self.point_cloud_callback, 10)

    def color_image_callback(self, msg):
        np_array = self.image_msg_to_numpy(msg)
        np.save('/image_transfer/color_image.npy', np_array)
        self.get_logger().info('Saved color image.')

    def depth_image_callback(self, msg):
        np_array = self.image_msg_to_numpy(msg)
        np.save('/image_transfer/depth_image.npy', np_array)
        self.get_logger().info('Saved depth image.')

    def depth_registered_image_callback(self, msg):
        np_array = self.image_msg_to_numpy(msg)
        np.save('/image_transfer/depth_registered_image.npy', np_array)
        self.get_logger().info('Saved depth registered image.')

    def point_cloud_callback(self, msg):
        points = self.point_cloud_msg_to_numpy(msg)
        np.save('/image_transfer/point_cloud.npy', points)
        self.get_logger().info('Saved point cloud.')

    def image_msg_to_numpy(self, msg):
        """Convert ROS Image message to NumPy array."""
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        if encoding == 'rgb8':
            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
        elif encoding == 'mono16':
            np_array = np.frombuffer(msg.data, dtype=np.uint16).reshape((height, width))
        else:
            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))

        return np_array

    def point_cloud_msg_to_numpy(self, msg):
        """Convert ROS PointCloud2 message to NumPy array."""
        points = []
        for point in self.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])

        return np.array(points)

    def read_points(self, cloud, field_names=None, skip_nans=False):
        """Read points from PointCloud2 message."""
        fmt = 'fff'
        point_step = struct.calcsize(fmt)
        for p in range(0, len(cloud.data), point_step):
            point = struct.unpack(fmt, cloud.data[p:p + point_step])
            if skip_nans and (np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2])):
                continue
            yield point

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
