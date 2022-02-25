import rclpy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class BasicSubscriber(Node):
    
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(CompressedImage, 'camera', self.timer_callback, qos_profile_sensor_data)
        self.bridge=CvBridge()

    def timer_callback(self, data):        
        frame = self.bridge.compressed_imgmsg_to_cv2(data)
        # frame = self.bridge.imgmsg_to_cv2(data)
        h, w, c = frame.shape
        frame_left = frame[:,:w//2,:]
        frame_right = frame[:,w//2:,:]
        frame = (frame_left+frame_right)//2
        merge = np.vstack([frame_left, frame_right, frame])
        # cv2.imshow('left', frame_left)
        # cv2.imshow('right', frame_right)
        cv2.imshow('frame', merge)

        key = cv2.waitKey(1)
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