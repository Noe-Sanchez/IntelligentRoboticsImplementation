import cv2
import os
import glob
import pathlib

import rclpy.logging

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

import time 

class takeImage(Node):
    def __init__(self):
        super().__init__('take_image')
        self.camera_subcriber = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.BASE_DIR = str(pathlib.Path(__file__).resolve().parent)
        self.saving_dir = self.BASE_DIR + '/saved_images'
        self.dataset_dir = self.BASE_DIR + '/dataset'
        self.frame = None
        self.flag = False
        self.timer = self.create_timer(0.1, self.getPics)
        self.cnt = 0

        self.current_label = ord('0')
        self.labels = self.getLabelsDict()

        self.showDict()

    def showDict(self):
        # Show labels as with its key number
        for key, value in self.labels.items():
            self.get_logger().info(f'{chr(key)}: {value}')
        

        
    def image_callback(self, msg):
        try:
            self.frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().info(str(e))
    
    def getLabelsDict(self):
        labels = {}
        for i, file in enumerate(glob.glob(self.dataset_dir + '/*')):
            labels[ord(str(i + 1))] = file.removeprefix(self.dataset_dir + '/')
        return labels
    
    def getPics(self):
        if self.frame is None:
            return

        key = cv2.waitKey(1) & 0xFF

        if key in self.labels.keys():
            self.current_label = key
            self.cnt = 0
            self.get_logger().info(f'Label changed to {self.labels[key]}')
        elif key == ord('s'):
            self.showDict()
            while True:
                key = cv2.waitKey(1) & 0xFF
                if key in self.labels.keys():
                    self.current_label = key
                    break
            self.cnt = 0

        cv2.imshow('frame', self.frame)

        if self.current_label not in self.labels.keys():
            return
        
        img_name = f'{self.dataset_dir}/{self.labels[self.current_label]}/{self.cnt}.jpg'
        
        
        cv2.imwrite(img_name, self.frame)
        self.get_logger().info(f'Image saved as {img_name}')
        self.cnt += 1
        
        time.sleep(0.3)
        
def main(args=None):
    rclpy.init(args=args)
    try:
        takeImage_ = takeImage()
        rclpy.spin(takeImage_)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()