import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import math
from sensor_msgs.msg import Imu

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

MEAN_UNIT = 30
TH_RANGE=50

class BasicSubscriber(Node):    
    def __init__(self):
        super().__init__('lidar_sensor')        
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile_sensor_data)        
        self.subscription_imu = self.create_subscription(Imu,'/imu',self.listener_callback,qos_profile_sensor_data)
        self.yaw_z = None
        self.size = 1000
        self.over_rate = 200
        
    def callback(self, data):        
        self.x =list()
        self.y =list()
        laser_range = data.ranges # 정면 부분

        unit = np.pi/180
        for idx, point in enumerate(laser_range):
            if point>4 or point<=0:
                self.x.append(int(-1))
                self.y.append(int(-1))
            else:
                self.x.append(int(point*self.over_rate*np.cos(idx*unit)+self.size//2))
                self.y.append(int(point*self.over_rate*np.sin(idx*unit)+self.size//2))
            
        img = np.zeros((self.size, self.size, 3))
        pre_xx, pre_yy = None, None
        for xx, yy in zip(self.x, self.y):
            if xx == -1: continue
            if pre_xx is None and pre_yy is None:
                pre_xx=xx
                pre_yy =yy
            else:
                point_range = np.sqrt(np.square(pre_xx-xx)+np.square(pre_yy-yy))
                if point_range<TH_RANGE: # 가까운 점일 경우
                # if True: # 가까운 점일 경우
                    cv2.line(img, (pre_xx, pre_yy), (xx, yy), (255,255,255), 2)
                    pre_xx, pre_yy = xx, yy
                else:
                    pre_xx, pre_yy = None, None

            # cv2.circle(img,(xx+self.size//2,yy+self.size//2), radius=1,color=(255,255,255),thickness=1)
      
        img = cv2.rotate(img,rotateCode= cv2.ROTATE_90_COUNTERCLOCKWISE)
        img = cv2.flip(img, 1)
        # img_float32 = np.float32(img)
        img_float32 = np.uint8(img)
        gray = cv2.cvtColor(img_float32,cv2.COLOR_RGB2GRAY)
        # gray = np.unit8()
        edges = cv2.Canny(gray,50,100,apertureSize = 3)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180., 40, minLineLength=5, maxLineGap=20)

        if lines is not None: # 라인 정보를 받았으면
            for i in range(lines.shape[0]):
                pt1 = (lines[i][0][0], lines[i][0][1]) # 시작점 좌표 x,y
                pt2 = (lines[i][0][2], lines[i][0][3]) # 끝점 좌표, 가운데는 무조건 0
                cv2.line(img, pt1, pt2, (255, 0, 255), 5, cv2.LINE_AA)
        cv2.rectangle(img, (self.size//2-20,self.size//2-30),(self.size//2+20,self.size//2+30),(0,255,255),2)
        cv2.imshow('plot', img)
        # cv2.imshow('edges', edges)
        key = cv2.waitKey(100)

        #         pass
        # rho, theta, threshold, min_line_len, max_line_gap = 1, 1*np.pi/180, 30,10,20
        # lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        # if lines is not None:
        #     for line in lines:
        #         x1, y1, x2, y2 = line
        #         cv2.line(img,(x1, y1), (x2, y2), (255,0,0), 3)

        # cv2.imshow('plot', img)
        # key = cv2.waitKey(100)
        if key == 27:
            raise KeyboardInterrupt

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
        self.yaw_z=int(yaw_z*rate) # 절대 각도
        # self.get_logger().info('orientation: "%s"' % str((roll_x, pitch_y, yaw_z)))
        # self.get_logger().info('linear_acceleration: "%s"' % str((ax, ay, az)))
        self.get_logger().info('rate: "%s"' % str(int(yaw_z*rate)))
        pass

def main(args=None):
    rclpy.init(args=args)
    basic_subcriber = BasicSubscriber()
    try:
        rclpy.spin(basic_subcriber)
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        basic_subcriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()    