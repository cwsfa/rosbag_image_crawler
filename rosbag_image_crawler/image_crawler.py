import cv2
import rclpy
import os
import getpass as gt

import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node

USERNAME = gt.getuser()

class ImageCrawler(Node):

    def __init__(self):
        super().__init__('rosbag2')
        self.bridge = CvBridge()
        self.n_frame = 0
        self.c_time = 0.0 # (s)

        # ros parameters
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('cycle', 1.0) # (s)
        self.declare_parameter('output_dir', '/home/' + USERNAME + '/clawed_image')
        
        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.cycle = self.get_parameter('cycle').value
        self.output_dir = self.get_parameter('output_dir').value
        
        # check if output directory exists
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            os.makedirs(os.path.join(self.output_dir, "color"))
            os.makedirs(os.path.join(self.output_dir, "depth"))

        # message filter
        self.color_sub = Subscriber(self, Image, color_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            100, 0.01,  # defines the delay (in seconds) with which messages can be synchronized
        )
        self.ts.registerCallback(self.sync_callback)
        
        self.get_logger().info("Image crawler is ready...")


    def sync_callback(self, color_msg, depth_msg): # message filter callback
        """_summary_: callback function for message filter

        Args:
            color_msg (Image): color image message
            depth_msg (Image): depth image message
        """
        assert color_msg.header.stamp == depth_msg.header.stamp

        if self.get_clock().now().to_msg().sec - self.c_time > self.cycle:
            self.c_time = self.get_clock().now().to_msg().sec # update current time
            self.get_logger().info("Number of frames crawled => " + str(self.n_frame))

            try:
                # convert ros image message to cv2 image
                cv_color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='passthrough')
                cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg)

            except CvBridgeError as e:
                print(e)

            # save crawled images
            cv2.imwrite(os.path.join(self.output_dir, "color", str(self.n_frame) + ".png"), cv_color_image)
            cv2.imwrite(os.path.join(self.output_dir, "depth", str(self.n_frame) + ".png"), cv_depth_image)

            # show crawled image
            cv_depth_image = cv2.applyColorMap(cv2.convertScaleAbs(cv_depth_image, alpha=0.03), cv2.COLORMAP_JET)
            dst = np.hstack((cv_color_image, cv_depth_image))
            text = "Frame: " + str(self.n_frame)
            cv2.putText(dst, text, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
            cv2.imshow('clawed image', dst)
            cv2.waitKey(1)

            self.n_frame += 1 # update number of frames crawled

def main(args=None):
    rclpy.init(args=None)

    try:
        image_crawler = ImageCrawler()
        try:
            rclpy.spin(image_crawler)
        except KeyboardInterrupt:
            image_crawler.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            image_crawler.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()