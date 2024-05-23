import cv2
import os
import glob
import pathlib

import rclpy.logging
from .vision_challenge04.cameraCalibration import calibrate

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

class CameraCalibration(Node):
    def __init__(self):
        super().__init__('take_image')
        self.camera_subcriber = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.BASE_DIR = str(pathlib.Path(__file__).resolve().parent)
        self.saving_dir = self.BASE_DIR + '/saved_images'
        self.frame = None
        self.flag = False
        self.timer = self.create_timer(0.1, self.getPics)
        self.cnt = 0
        
    
    def image_callback(self, msg):
        try:
            self.frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().info(str(e))
    
    def getPics(self):
        rclpy.logging.get_logger('rclpy').info(f'Images taken: {self.cnt}')
        cv2.imshow('camera', self.frame)
        img_name = os.path.join(self.saving_dir, f'saved_ima_{self.cnt}.jpg')
        if cv2.waitKey(1) & 0xFF==ord('1'):
            cv2.imwrite(img_name, self.frame)
            rclpy.logging.get_logger('rclpy').info('Saved at: ' + img_name)
            cv2.imshow('Captured img', self.frame)
            self.cnt += 1
        
def main(args=None):
    rclpy.init(args=args)
    try:
        cameraCalibration_ = CameraCalibration()
        rclpy.spin(cameraCalibration_)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()