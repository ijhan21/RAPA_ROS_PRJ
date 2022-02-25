import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import Twist
import time
class CmdPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)        
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)        
        self.count = 0
        self.twist = Twist()

    def timer_callback(self):
        # self.twist.linear.x = (random.random()-0.5)*1
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)
        self.get_logger().info('Data: "%f"' % (self.twist.linear.x))
        self.count += 1
        if self.count ==10:
            print("종료")
            self.twist.linear.x = 0.
            self.publisher_.publish(self.twist)
            time.sleep(1)
            raise KeyboardInterrupt
    def stop(self):
        self.twist.linear.x = 0.
        self.publisher_.publish(self.twist)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    custom_msg_publisher = CmdPublisher()
    try:
        rclpy.spin(custom_msg_publisher)
    except KeyboardInterrupt:
        custom_msg_publisher.stop()
        print("finish")
    finally:
        custom_msg_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()