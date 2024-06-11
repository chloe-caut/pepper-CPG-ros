import rclpy
from std_msgs.msg import Float32

def x_pixel_callback(msg):
    x_pixel = msg.data
    print('Received x_pixel: ', x_pixel)
    #using therceived data


def main():
    rclpy.init()
    node = rclpy.create_node('pepper_subscriber')
                             

    # Create a subscriber to the x_pixel topic
    subscriber = node.create_subscription(Float32, '/x_pixel', x_pixel_callback, 10)
    try:
        # Spin the node so the subscriber can receive messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        #    Close the node when Ctrl+C is pressed
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
