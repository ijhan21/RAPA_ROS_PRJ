import rclpy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class BasicSubscriber(Node):
    
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(CompressedImage, 'camera', self.timer_callback, qos_profile_sensor_data)
        self.bridge=CvBridge()

    def timer_callback(self, data):        
        frame = self.bridge.compressed_imgmsg_to_cv2(data)
        # frame = self.bridge.imgmsg_to_cv2(data)
        cv2.imshow('subscription Img', frame)

        key = cv2.waitKey(100)
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