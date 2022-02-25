import rclpy
import serial
import time
from std_msgs.msg import String
from rclpy.node import Node


class HoverControl(Node):
    def __init__(self):
        super().__init__('serial_connect_node')
        self.publisher_=self.create_subscription(String, '/cmd_hover',self.listener_callback ,1)
        self.drive_option = ['a','b','l','r','j']
        self.ser = serial.Serial("/dev/ttyACM0", 115200)
        self.get_logger().info('Start Serial_Connect_Node')

    def listener_callback(self, msg):
        self.get_logger().info('Received from websocket bridge: "%s"' % msg.data)
        data = msg.data
        if data in self.drive_option:
            self.ser.write(bytes(data, encoding='ascii'))
        else:
            self.get_logger().info('Nothing Command')
    def finish(self):
        self.ser.write(bytes('j', encoding='ascii')) # j : steer 0  speed 0


def main(args=None):
    rclpy.init(args=args)
    serial_subscriber = HoverControl()
    
    try:
        rclpy.spin(serial_subscriber)
    except KeyboardInterrupt as k:
        serial_subscriber.get_logger().info('Finishing Node')
        serial_subscriber.finish()
    serial_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
