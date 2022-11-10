import cv2
import rclpy
import os
import glob

import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node

USERNAME = os.getlogin()


class ImageCrawler(Node):

    def __init__(self):
        super().__init__('image_crawler')
        self.get_logger().info('Image Crawler Node started.')
        # ros parameters
        self.declare_parameter('is_debug', False)
        self.declare_parameter('is_label', False)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('rate', 1.0)  # (s)
        self.declare_parameter('output_dir', '/home/' + USERNAME + '/clawed_image')

        self.is_debug = self.get_parameter('is_debug').value
        self.is_label = self.get_parameter('is_label').value
        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.rate = self.get_parameter('rate').value
        self.output_dir = self.get_parameter('output_dir').value
        if self.is_label: self.output_dir += '_label'

        self.get_logger().info(f'Show image: {self.is_debug}')
        self.get_logger().info(f'Enable labeling: {self.is_label}')
        self.get_logger().info(f'Color image topic: {color_topic}')
        self.get_logger().info(f'Depth image topic: {depth_topic}')
        self.get_logger().info(f'Crawling rate rate: {self.rate}')
        self.get_logger().info(f'Saving directory: {self.output_dir}')

        # check if output directory exists
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            os.makedirs(os.path.join(self.output_dir, 'color'))
            os.makedirs(os.path.join(self.output_dir, 'depth'))

        # param init
        self.bridge = CvBridge()
        self.c_time = 0.0  # (s)
        self.n_frame = starting_frame(self.output_dir, use_label=self.is_label)

        if self.is_label:
            self.label = False
            self.create_subscription(Bool, 'image_crawler/label', self.label_callback, 1)

        # message filter
        self.color_sub = Subscriber(self, Image, color_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            100, 0.01,  # defines the delay (in seconds) with which messages can be synchronized
        )
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info('Image crawler is ready...')

    def sync_callback(self, color_msg, depth_msg):  # message filter callback
        """_summary_: callback function for message filter

        Args:
            color_msg (Image): color image message
            depth_msg (Image): depth image message
        """
        assert color_msg.header.stamp == depth_msg.header.stamp

        if self.get_clock().now().to_msg().sec - self.c_time > self.rate:
            self.c_time = self.get_clock().now().to_msg().sec  # update current time
            frame_name = self.get_frame_name(use_label=self.is_label)
            self.get_logger().info('Number of frames crawled => ' + str(self.n_frame))
            try:
                # convert ros image message to cv2 image
                cv_color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
                cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg)
                # save crawled images
                cv2.imwrite(os.path.join(self.output_dir, 'color', frame_name), cv_color_image)
                cv2.imwrite(os.path.join(self.output_dir, 'depth', frame_name), cv_depth_image)
                # show crawled image
                if self.is_debug:
                    cv_depth_image = cv2.applyColorMap(cv2.convertScaleAbs(cv_depth_image, alpha=0.03), cv2.COLORMAP_JET)
                    dst = np.hstack((cv_color_image, cv_depth_image))
                    text = 'Frame: ' + str(self.n_frame)
                    cv2.putText(dst, text, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
                    cv2.imshow('clawed image', dst)
                    cv2.waitKey(1)
                # update number of frames crawled
                self.n_frame += 1
            except CvBridgeError as e:
                self.get_logger().error(str(e))

    def label_callback(self, msg):
        self.label = msg.data

    def get_frame_name(self, use_label=False):
        if use_label:
            label = '1' if self.label else '0'
            return f'{str(self.n_frame)}_{label}.png'
        return f'{str(self.n_frame)}.png'


def starting_frame(output_dir, use_label=False):
    """
    Initialize the frame number. Start from after the latest number or 0.
    """
    try:
        seperator = '_' if use_label else '.'
        files = os.listdir(output_dir + '/color')
        return int(sorted([i.split(seperator)[0] for i in files], key=int)[-1]) + 1
    except:
        return 0


def main(args=None):
    rclpy.init(args=args)
    image_crawler = ImageCrawler()
    try:
        rclpy.spin(image_crawler)
    except KeyboardInterrupt:
        image_crawler.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        image_crawler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
