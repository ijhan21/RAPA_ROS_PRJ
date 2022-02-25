import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial

ser = serial.Serial("/dev/ttyACM0", 115200)

class TestSubscriber(Node):

    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            String,
            'websock_echo',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Received from websocket bridge: "%s"' % msg.data)
        ser.write()

def main(args=None):
    rclpy.init(args=args)
    test_subscriber = TestSubscriber()
    rclpy.spin(test_subscriber)

    test_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()