import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        timer_period = 40  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = 0.25
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.25
        msg.pose.orientation.x = 1.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: Position ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})')

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()