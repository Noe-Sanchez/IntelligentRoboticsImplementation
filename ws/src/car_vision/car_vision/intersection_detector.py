import time
import numpy as np
import pathlib

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

import cv2


BASE_PATH = pathlib.Path(__file__).parent.absolute()
print("BASE_PATH: ", BASE_PATH)

class intersectionDetector(Node):
    def __init__(self):
        super().__init__('car_inference')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.visualization_pub = self.create_publisher(Image, 'intersection_detector', 10)
        self.distance_pub = self.create_publisher(Float32, 'intersection_distance', 10)
        self.prevtime = time.time()
        self.frames = 0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # take only lower half
        h, w, _ = frame.shape
        cropped_frame = frame[h//2:h, :]
        # detect horizontal lines
        imgray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("Imgray", imgray)
        cv2.imshow("Cropped Frame", cropped_frame)
        h, w = imgray.shape
        # Binarize the image if necessary
        _, binary_image = cv2.threshold(imgray, 90, 255, cv2.THRESH_BINARY)
        binary_image = cv2.bitwise_not(binary_image)
        #cv2.imshow("Binary Image", binary_image)

        kernel = np.ones((2,2), np.uint8)
        binary_image = cv2.erode(binary_image, kernel, iterations=1)

        kernel = np.ones((5,5), np.uint8)
        binary_image = cv2.dilate(binary_image, kernel, iterations=1)
        cv2.imshow("Binary Image", binary_image)
        # find lines
        lines = cv2.HoughLinesP(binary_image, 1, np.pi/180, 100, minLineLength=100, maxLineGap=15)
        
        # find horizontal lines and draw
        TRESHOLD = 0.2
        distance = -1
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # filter horizontal lines, treshold is n% of the image height
                if abs(y2-y1) < int(h*TRESHOLD):
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # calculate distance
                    distance = h - y1
        # convert binary to rgb format
        self.visualization_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))  
        self.distance_pub.publish(Float32(data=float(distance)))
        cv2.waitKey(1)      

def main(args=None):
    rclpy.init(args=args)

    try:
        intersectionDetector_ = intersectionDetector()
        rclpy.spin(intersectionDetector_)
        intersectionDetector_.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
    