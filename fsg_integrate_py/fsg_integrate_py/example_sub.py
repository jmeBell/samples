import rclpy
from rclpy.node import Node

from fsg_msgs.msg import SegmentArray


class FsgSub(Node):

    def __init__(self):
        super().__init__('fsg_sub')
        self.subscription = self.create_subscription(
            SegmentArray,
            'fsg_segments',
            self.fsg_seg_callback,
            10)
        self.subscription

    def fsg_seg_callback(self, msg):
        for i in msg.segments:
            self.get_logger().info('Segment points: (%f, %f) (%f, %f)' % (i.ax, i.ay, i.bx, i.by))


def main(args=None):
    rclpy.init(args=args)

    fsg_sub = FsgSub()

    rclpy.spin(fsg_sub)

    # Destroy the node explicitly
    fsg_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
