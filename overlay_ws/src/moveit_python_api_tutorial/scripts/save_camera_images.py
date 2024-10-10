# subscribe to the topics
# /camera/depth_registered/camera_info (sensor_msgs/CameraInfo)
# /camera/depth_registered/image_rect (sensor_msgs/Image)
# /camera/color/image_raw (sensor_msgs/Image) RGB8 encoding
# Save the data from above topics to a Numpy file .npz with keys
# 'depth' + 'K' + 'rgb'


import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

class CameraDataSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_data = None
        self.camera_info = None
        self.rgb_data = None

        rospy.init_node('camera_data_saver', anonymous=True)
        
        rospy.Subscriber('/camera/depth_registered/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/depth_registered/image_rect', Image, self.depth_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)

    def camera_info_callback(self, data):
        self.camera_info = data.K  # Assuming K is the intrinsic camera matrix

    def depth_callback(self, data):
        self.depth_data = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def rgb_callback(self, data):
        self.rgb_data = self.bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')

    def save_data(self):
        if self.depth_data is not None and self.camera_info is not None and self.rgb_data is not None:
            np.savez('camera_data.npz', depth=self.depth_data, K=self.camera_info, rgb=self.rgb_data)
            rospy.loginfo("Data saved to camera_data.npz")
        else:
            rospy.logwarn("Not all data received yet")

if __name__ == '__main__':
    saver = CameraDataSaver()
    rospy.spin()
    saver.save_data()