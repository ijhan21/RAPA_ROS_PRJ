from cv2 import threshold
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np

MEAN_UNIT = 30
TH_RANGE=50

class BasicSubscriber(Node):    
    def __init__(self):
        super().__init__('lidar_sensor')        
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile_sensor_data)        
        self.size = 800
        self.over_rate = 300
        
    def callback(self, data):        
        self.x =list()
        self.y =list()
        laser_range = data.ranges # 정면 부분

        unit = np.pi/180
        for idx, point in enumerate(laser_range):
            if point>1 or point<=0:
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
        # edges = gray.copy()
        # lines = cv2.HoughLines(edges,1,np.pi/180,45)
        # if lines is not None:
        #     for line in lines:
        #         rho,theta = line[0]
        #         a = np.cos(theta)
        #         b = np.sin(theta)
        #         x0 = a*rho
        #         y0 = b*rho
        #         x1 = int(x0 + 1000*(-b))
        #         y1 = int(y0 + 1000*(a))
        #         x2 = int(x0 - 1000*(-b))
        #         y2 = int(y0 - 1000*(a))

        #         cv2.line(img,(x1,y1),(x2,y2),(0,0,255),1)

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