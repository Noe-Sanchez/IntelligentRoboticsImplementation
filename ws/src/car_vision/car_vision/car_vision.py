import cv2
import numpy as np
import threading
import pathlib
from enum import IntEnum
import matplotlib.pyplot as plt


import rclpy
from rclpy.node import Node

from car_interfaces.msg import Detection
from std_msgs.msg import Float32


DEBUG = False
BASE_DIR = str(pathlib.Path(__file__).resolve().parent)


class DetectionEnumerator(IntEnum):
    NO_DETECTION = 0
    RED_CIRCLE = 1
    YELLOW_CIRCLE = 2
    GREEN_CIRCLE = 3

#Functions
class Vision():
    def __init__(self):
        camera_number = 0
        self.cap = cv2.VideoCapture(camera_number)
        self.camera_matrix = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.rvecs = None
        self.tvecs = None

    def camera_thread(self):
        ret, self.frame = self.cap.read()
        # self.im1 = self.ax1.imshow(self.frame)
        # self.im2 = self.ax1.imshow(self.frame)

        # plt.ion()
        while True:
            ret, self.frame = self.cap.read()

            if not ret:
                print("Could not receive frame")
                break

            cv2.imshow('Original image', self.frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    def getColoredCirles(self):
        # color verde
        lower_green = np.array([61, 67, 73])
        upper_green = np.array([102, 255, 255])

        # amarillo
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        #rojo
        lower_red = np.array([136, 87, 111])
        upper_red = np.array([180, 255, 255])

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

        for detected_output, color in zip(detected_outputs, [DetectionEnumerator.RED_CIRCLE, DetectionEnumerator.GREEN_CIRCLE, DetectionEnumerator.YELLOW_CIRCLE]): 
            if DEBUG:
                cv2.imshow("Masked image " + str(color), detected_output)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()
            circles_pts[color] = []
            gray = cv2.cvtColor(detected_output, cv2.COLOR_BGR2GRAY)
            blur = cv2.medianBlur(gray, 5)
            canny = cv2.Canny(blur, 75, 250)

            if DEBUG:
                cv2.imshow("Canny image " + str(color), canny)

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

            cv2.imshow('Detections', self.frame)
        
        return circles_pts
        
class carVision(Node):
    def __init__(self, camera_vision):
        super().__init__("car_vision_publisher")
        self.detection_publisher = self.create_publisher(Detection, '/carDetections', 10)
        self.road_publisher = self.create_publisher(Float32, '/roadError', 10)
        time_interval = 0.1 # seconds
        self.timer = self.create_timer(time_interval, self.timer_callback)
        self.vision = camera_vision

    def timer_callback(self):
        detection_msg = Detection()
        circle_pts = self.vision.getColoredCirles()
        min_distance = 254
        detected_color = DetectionEnumerator.NO_DETECTION
        # print(circle_pts)
        if len(circle_pts) > 0:
            for color_key in circle_pts.keys():
                if len(circle_pts[color_key]) < 1:
                    continue
                
                # if radius is used, then the distance is the radius and the maxium radius should be looked for
                for distance in circle_pts[color_key]:
                    if min_distance > distance:
                        min_distance = distance
                        detected_color = color_key

            detection_msg.detection_type = detected_color
            detection_msg.distance = min_distance

        self.detection_publisher.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    vision = Vision()

    try:
        if not DEBUG:
            threading.Thread(target=vision.camera_thread).start()
        carVision_ = carVision(vision)
        rclpy.spin(carVision_)
        carVision_.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
    