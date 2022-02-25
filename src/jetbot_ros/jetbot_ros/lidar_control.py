import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import cv2
from matplotlib.animation import FuncAnimation
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
class BasicSubscriber(Node):
    
    def __init__(self):
        super().__init__('lidar_sensor')
        # plt.ion()
        # self.x =list()
        # self.y =list()
        self.fig = plt.figure(figsize=(5,5))  
        self.ax = plt.subplot()
        # plt.autoscale(enable=True)      
        # self.ani = FuncAnimation(self.fig, self.aniFunc, interval=50)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile_sensor_data)
        # plt.show()

    def callback(self, data):        
        self.x =list()
        self.y =list()
        laser_range = data.ranges # 정면 부분
        unit = np.pi/180
        for idx, point in enumerate(laser_range):
            if point<0.:point=0.
            self.x.append(point*np.cos(idx*unit))
            self.y.append(point*np.sin(idx*unit))
        self.ax.cla()
        plt.xlim(-5,5)
        plt.ylim(-5,5)
        self.ax.scatter(self.x, self.y, marker='D', s=5)
        img = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img  = img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        cv2.imshow('plot', img)
        cv2.waitKey(10)       
        

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