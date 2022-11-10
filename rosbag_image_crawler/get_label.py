import rclpy
from std_msgs.msg import Int16, Bool
from rclpy.node import Node
from threading import Timer


class LabelNode(Node):
    def __init__(self):
        super().__init__('label_node')
        self.get_logger().info('Label Node started.')
        # ros parameters
        self.declare_parameter('label_rate', 0.5)
        self.label_rate = self.get_parameter('label_rate').value
        # init parameters
        self.cx, self.vx = 100.0, 100.0
        self.range = 20
        # subscribers
        self.create_subscription(Int16, 'calibration/cx_err', self.cx_callback, 10)
        self.create_subscription(Int16, 'calibration/vx_err', self.vx_callback, 10)
        # publisher
        self.label_pub = self.create_publisher(Bool, 'image_crawler/label', 1)
        # thread
        Timer(self.label_rate, self.send_label).start()

    def send_label(self):
        msg = Bool()
        if abs(self.cx) < self.range and abs(self.vx) < self.range:
            msg.data = True
        else:
            msg.data = False
        self.label_pub.publish(msg)
        self.cx, self.vx = 100, 100
        Timer(self.label_rate, self.send_label).start()

    def cx_callback(self, msg):
        self.cx = msg.data

    def vx_callback(self, msg):
        self.vx = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = LabelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
