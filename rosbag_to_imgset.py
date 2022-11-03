import cv2
import rclpy
import argparse
import os
import time
from numpy import numarray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node

AQ_TIME = 1 # 1s

class Rosbag2Imgset(Node):

    def __init__(self):
        super().__init__('rosbag2')
        self.bridge = CvBridge()
        queue_size = 30
        self.frame_id = 0
        self.c_time = 0.001 # ms

        parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
        parser.add_argument("-i", help="rosbag input path (e.g. ~/test/st.bag)",)   # -c: acquisition cycle
        parser.add_argument("-o", help="rosbag Output directory.")                  # -o: output path
        # args = parser.parse_args()
       
        # message filter
        self.color_sub = Subscriber(self, Image, "/camera/color/image_raw")
        self.depth_sub = Subscriber(self, Image, "/camera/depth/image_rect_raw")
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size, 0.01,  # defines the delay (in seconds) with which messages can be synchronized
        )
        self.ts.registerCallback(self.sync_callback)

    def sync_callback(self, color_msg, depth_msg): #Message Filters
        assert color_msg.header.stamp == depth_msg.header.stamp

        if self.get_clock().now().to_msg().sec - self.c_time > AQ_TIME:
            self.c_time = self.get_clock().now().to_msg().sec
            self.get_logger().info("currrmenr frame id = " + str(self.frame_id))

        #  the same timestamp every fixed hz

            try:
                cv_color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="mono8")
                cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg)

            except CvBridgeError as e:
                print(e)
           
            path = '/home/turtle/Desktop/test/color_img'
            path2 = '/home/Users/Desktop/test/depth_img'
            cv2.imshow("color_img", cv_color_image)
            cv2.imwrite(os.path.join(path, 'cv_color_image{}.png'), cv_color_image)
            # 저장은 되는데 순차적으로 사진이 안들어 옴;; 멀까
            
            # cv2.imwrite('/home/turtle/robot_ws/src/rosimgset/color_img/cv_color_image{}.png' .format(self.frame_id),cv_color_image, params=[cv2.IMWRITE_PNG_COMPRESSION,0])
            cv2.imshow("depth_img", cv_depth_image)
            cv2.imwrite(os.path.join(path2,'cv_depth_image{}.png'),cv_depth_image)
            # cv2.imwrite('/home/turtle/robot_ws/src/rosimgset/depth_img/cv_depth_image{}.png' .format(self.frame_id),cv_depth_image, params=[cv2.IMWRITE_PNG_COMPRESSION,0])
            cv2.waitKey(1)

            self.frame_id += 1 # update frame_id

        # 3. save two topics of the same timestamp every fixed hz


if __name__ == '__main__':

    rclpy.init(args=None)
    rclpy.create_node("rosbag_to_imgset")
    rosbag2imgset = Rosbag2Imgset()
    # executor = MultiThreadedExecutor()
    # executor.add_node(rosbag2imgset)
    rclpy.spin(rosbag2imgset)
    rosbag2imgset.destroy_node()
    rclpy.shutdown()

