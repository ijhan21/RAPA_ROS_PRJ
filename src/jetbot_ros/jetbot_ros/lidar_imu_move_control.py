import random
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import time
# from .utils import *

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
        # self.get_logger().info('rate: "%s"' % str(int(yaw_z*rate)))
        pass

MEAN_UNIT = 30
TH_RANGE=40
def func(data):
    return data[-1]
class BasicSubscriber(Node):    
    def __init__(self):
        super().__init__('lidar_sensor')        
        self.TH_DISTANCE = 100
        self.TH_LENGTH = 30
        self.yaw_z = None
        self.size = 1000
        self.over_rate = 200
        # self.twist = Twist()
        self.first_angle = None
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)        
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile_sensor_data)        
        self.subscription_imu = self.create_subscription(Imu,'/imu',self.listener_callback,qos_profile_sensor_data)
        self.pre_diff_cos_theta = 0.
        self.direction = 1.
    def callback(self, data):        
        if self.first_angle is None: return
        self.x =list()
        self.y =list()
        laser_range = np.array(data.ranges)
        # laser_range = np.hstack( [laser_range[self.first_angle*-1:],laser_range[:self.first_angle*-1]])
        # laser_range[:60]=-2
        # laser_range[120:240]=-2
        # laser_range[300:]=-2
        # laser_range = laser_range[60:120]
        # laser_range[:180] = -2
        laser_range[90:270] = -2
        unit = np.pi/180
        uncatched_range = list()
        for idx, point in enumerate(laser_range):
            length = 0.5
            if point==-2:
                uncatched_range.append((int(length*self.over_rate*np.cos(idx*unit)+self.size//2),int(length*self.over_rate*np.sin(idx*unit)+self.size//2)))
            else:
                uncatched_range.append((0, 0))
            if point>4 or point<=0:
                self.x.append(int(-1))
                self.y.append(int(-1))
            else:
                if point==-2:continue
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
      
        # img_float32 = np.float32(img)
        img_float32 = np.uint8(img)
        gray = cv2.cvtColor(img_float32,cv2.COLOR_RGB2GRAY)
        # gray = np.unit8()
        edges = cv2.Canny(gray,50,100,apertureSize = 3)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180., 40, minLineLength=5, maxLineGap=20)
        ltd = list()
        if lines is not None: # 라인 정보를 받았으면
            for i in range(lines.shape[0]):
                pt1 = (lines[i][0][0], lines[i][0][1]) # 시작점 좌표 x,y
                pt2 = (lines[i][0][2], lines[i][0][3]) # 끝점 좌표, 가운데는 무조건 0
                
                length = np.sqrt((lines[i][0][1]-lines[i][0][3])**2+(lines[i][0][0]-lines[i][0][2])**2)                
                # if length <100:continue
                # theta = np.arctan((lines[i][0][1]-lines[i][0][3])/(lines[i][0][0]-lines[i][0][2]+0.1))
                theta = (lines[i][0][1]-lines[i][0][3],lines[i][0][0]-lines[i][0][2]+0.1)
                distance = np.sqrt(((lines[i][0][1]+lines[i][0][3])/2-self.size//2)**2+((lines[i][0][0]+lines[i][0][2])/2-self.size//2)**2)

                ltd.append((length, theta, distance))
                cv2.line(img, pt1, pt2, (255, 0, 255), 5, cv2.LINE_AA)
        # 각 직선과 센터와의 거리 측정
        # 가장 가까운 직선과 일정거리에 도달하면 직선의 기울기에 따라 z값 조정하면서 이동
        if len(ltd)==0:return
        ltd.sort(key=func)        
        
        distance = ltd[0][2]
        theta = ltd[0][1]
        origin_line=(1,0)
        cos_theta = np.cos((theta[0]*origin_line[0]+theta[1]*origin_line[1])/((np.sqrt(theta[0]**2+theta[1]**2)*(origin_line[0]**2+origin_line[1]**2))))
        # theta_compare = theta_compare if theta_compare<0 else theta_compare+180
        # if distance < 10.: 
        diff_cos_theta = round(1-cos_theta,0)
        self.twist = Twist()
        random_num = (random.random()-0.5)+0.3
        if self.pre_diff_cos_theta<diff_cos_theta:
            self.direction *=-1.
            pass        
        if distance>50: 
            self.direction*=-1
            self.twist.linear.x = 0.2
            self.twist.angular.z = self.direction*10.0*diff_cos_theta**2
        elif distance<50: 
            self.twist.linear.x = 0.2
            self.twist.angular.z = self.direction*10.0*diff_cos_theta**2
        else:            
            self.twist.linear.x = 0.2
        self.pre_diff_cos_theta = diff_cos_theta
        print(diff_cos_theta,distance)
        # print('right' if self.twist.angular.z<0 else 'left')
        self.cmd_vel_publisher.publish(self.twist)     
        cv2.line(img, (self.size//2,self.size//2),(self.size//2-int(np.sin(self.first_angle*np.pi/180)*30),self.size//2-int(np.cos(self.first_angle*np.pi/180)*30)),(0,255,255),15)
        cv2.circle(img, (self.size//2-int(np.sin(self.first_angle*np.pi/180)*30),self.size//2-int(np.cos(self.first_angle*np.pi/180)*30)), 10,(0,0,255), -1)
        for x, y in uncatched_range:            
            cv2.circle(img, (x, y), 2, (100, 100, 100), 3)
        img = cv2.rotate(img,rotateCode= cv2.ROTATE_90_COUNTERCLOCKWISE)
        img = cv2.flip(img, 1)
        cv2.imshow('plot', img)
        # cv2.imshow('edges', edges)
        key = cv2.waitKey(100)
        if key == 27:
            self.twist.angular.z = 0.
            self.twist.linear.x = 0.0
            self.cmd_vel_publisher.publish(self.twist)
            time.sleep(1)

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
        # self.get_logger().info('rate: "%s"' % str(int(yaw_z*rate)))
        # if self.first_angle is None:
        #     self.first_angle = int(yaw_z*rate)
        self.first_angle = int(yaw_z*rate)
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