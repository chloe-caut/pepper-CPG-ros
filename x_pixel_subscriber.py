import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class XPixelSubscriber(Node):
    def _init_(self):
        super()._init_('x_pixel_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            '/hand_tracking/x_pixel',
            self.x_pixel_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def x_pixel_callback(self, msg):
        self.get_logger().info('Received x_pixel: %f' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    x_pixel_subscriber = XPixelSubscriber()
    rclpy.spin(x_pixel_subscriber)
    x_pixel_subscriber.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
