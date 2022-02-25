import rclpy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class BasicPublisher(Node):
    
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera', qos_profile_sensor_data)
        
        timer_period = 0.05
        self.cap =cv2.VideoCapture(0)
        # self.cap =cv2.VideoCapture("'nvarguscamerasrc! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=(fraction)30/1 ! nvtee ! nvvidconv flip-method=0 ! video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! videoconvert ! appsink'")
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge=CvBridge()
        self.timer_callback()

    def timer_callback(self):        
        ret, frame = self.cap.read()
        # print(ret)
        if ret:
            #cv2.imshow('img', frame)
            # print(frame)
            msg = self.bridge.cv2_to_compressed_imgmsg(frame,dst_format='jpg')
            # msg = self.bridge.cv2_to_imgmsg(frame)
            # print(msg)
            self.publisher_.publish(msg)

            key = cv2.waitKey(500)

        # cv2.destroyAllWindows()
        # self.cap.release()
        # raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    basic_publisher = BasicPublisher()
    try:
        rclpy.spin(basic_publisher)
    except KeyboardInterrupt:
        print("프로그램 종료")
    finally:
        basic_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()    
