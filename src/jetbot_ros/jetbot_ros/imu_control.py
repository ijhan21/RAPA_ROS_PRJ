import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import numpy as np
import math
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class ImuSubscriber(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.listener_callback,
            qos_profile_sensor_data)
        # self.subscription  

    def listener_callback(self, scan):
        x = scan.orientation.x 
        y = scan.orientation.y
        z = scan.orientation.z 
        w = scan.orientation.w 

        ax = scan.linear_acceleration.x 
        ay = scan.linear_acceleration.y
        az = scan.linear_acceleration.z 

        rate = 180/np.pi

        roll_x, pitch_y, yaw_z = euler_from_quaternion(x, y, z, w)
        # self.get_logger().info('orientation: "%s"' % str((roll_x, pitch_y, yaw_z)))
        # self.get_logger().info('linear_acceleration: "%s"' % str((ax, ay, az)))
        self.get_logger().info('rate: "%s"' % str(int(yaw_z*rate)))
        pass

def main(args=None):
    rclpy.init(args=args)
    imu_subcriber = ImuSubscriber()
    rclpy.spin(imu_subcriber)

    imu_subcriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()