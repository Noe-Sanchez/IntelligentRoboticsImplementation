import cv2
import numpy as np
import threading
import pathlib
from enum import IntEnum


import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from car_interfaces.msg import Detection
from std_msgs.msg import Float32
from sensor_msgs.msg import Image


DEBUG = False
BASE_DIR = str(pathlib.Path(__file__).resolve().parent)

class DetectionEnumerator(IntEnum):
    NO_DETECTION = 0
    RED_CIRCLE = 1
    YELLOW_CIRCLE = 2
    GREEN_CIRCLE = 3
      
class carVision(Node):
    def __init__(self):
        super().__init__("car_vision_publisher")
        self.detection_publisher = self.create_publisher(Detection, '/carDetections', 10)
        self.road_publisher = self.create_publisher(Float32, '/roadError', 10)

        self.camera_publisher = self.create_publisher(Image, '/image_detections', 10)
        self.camera_subcriber = self.create_subscription(Image, '/image_raw', self.camera_callback, 10)

        time_interval = 0.1 # seconds
        self.timer = self.create_timer(time_interval, self.timer_callback)
        
        self.bridge = CvBridge()
        self.frame = None

    def camera_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().info(str(e))
    
    def getColoredCirles(self):
        if self.frame is None:
            return []
         # color verde using (hMin = 50 , sMin = 30, vMin = 50), (hMax = 80 , sMax = 255, vMax = 255)
        lower_green = np.array([50, 30, 50])
        upper_green = np.array([80, 255, 255])

        # amarillo usando (hMin = 20 , sMin = 120, vMin = 140), (hMax = 100 , sMax = 200, vMax = 255)
        lower_yellow = np.array([20, 120, 140])
        upper_yellow = np.array([100, 200, 255])

        #rojo using (hMin = 0 , sMin = 125, vMin = 90), (hMax = 5 , sMax = 255, vMax = 255)
        lower_red = np.array([0, 125, 90])
        upper_red = np.array([5, 255, 255])

        detected_outputs = []
        hsvFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        unmasked_image = self.frame.copy()
        # Masked image red
        mask = cv2.inRange(hsvFrame, lower_red, upper_red)
        # detected_output_red = cv2.bitwise_and(image, image, mask=mask)
        detected_outputs.append(cv2.bitwise_and(unmasked_image, unmasked_image, mask=mask))
        unmasked_image = self.frame.copy()
        # Masked image green
        mask = cv2.inRange(hsvFrame, lower_green, upper_green)
        # detected_output_green = cv2.bitwise_and(image, image, mask=mask)
        detected_outputs.append(cv2.bitwise_and(unmasked_image, unmasked_image, mask=mask))
        unmasked_image = self.frame.copy()
        # Masked image yellow
        mask = cv2.inRange(hsvFrame, lower_yellow, upper_yellow)
        # detected_output_yellow = cv2.bitwise_and(image, image, mask=mask)
        detected_outputs.append(cv2.bitwise_and(unmasked_image, unmasked_image, mask=mask))

        circles_pts = {}

        for detected_output, color in zip(detected_outputs, [DetectionEnumerator.RED_CIRCLE, DetectionEnumerator.GREEN_CIRCLE, DetectionEnumerator.YELLOW_CIRCLE ]):
            # if DEBUG:
            #     cv2.imshow("Masked image " + str(color), detected_output)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
            circles_pts[color] = []
            gray = cv2.cvtColor(detected_output, cv2.COLOR_BGR2GRAY)
            # blur = cv2.gaussianBlur(gray, 5)
            blur = cv2.GaussianBlur(gray,(9,9),2)
            canny = cv2.Canny(blur, 75, 250)

            # if DEBUG:
            #     cv2.imshow("Blur image " + str(color), blur)

            # Morph open 
            thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
            opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)

            # Find contours and filter using contour area and aspect ratio
            cnts = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if len(cnts) == 2 else cnts[1]
            for c in cnts:
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.04 * peri, True)
                area = cv2.contourArea(c)
                if len(approx) > 5 and area > 1000 and area < 500000:
                    ((x, y), r) = cv2.minEnclosingCircle(c)
                    #circle_pts.append((int(x), int(y), int(r)))
                    circles_pts[color].append(r)
                    if color == DetectionEnumerator.RED_CIRCLE:
                        color_rgb = (0, 0, 255)
                    elif color == DetectionEnumerator.GREEN_CIRCLE:
                        color_rgb = (0, 255, 0)
                    else:
                        color_rgb = (0, 255, 255)
                    
                    cv2.circle(self.frame, (int(x), int(y)), int(r), color_rgb, 2)

            # cv2.imshow('Detections', self.frame)
            self.camera_publisher.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
            print ("Publishing image")        
        return circles_pts

    def timer_callback(self):
        detection_msg = Detection()
        circle_pts = self.getColoredCirles()
        min_distance = 254
        max_radius = -1
        useRadius = True
        detected_color = DetectionEnumerator.NO_DETECTION
        # print(circle_pts)
        if len(circle_pts) > 0:
            for color_key in circle_pts.keys():
                if len(circle_pts[color_key]) < 1:
                    continue
                
                # if radius is used, then the distance is the radius and the maxium radius should be looked for
                if useRadius:
                    for radius in circle_pts[color_key]:
                        if max_radius < radius:
                            max_radius = radius
                            detected_color = color_key
        
                    detection_msg.distance = max_radius
                else:
                    for distance in circle_pts[color_key]:
                        if min_distance > distance:
                            min_distance = distance
                            detected_color = color_key

                    detection_msg.distance = min_distance

            detection_msg.detection_type = detected_color

        self.detection_publisher.publish(detection_msg)
        

def main(args=None):
    rclpy.init(args=args)

    try:
        carVision_ = carVision()
        rclpy.spin(carVision_)
        carVision_.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
    