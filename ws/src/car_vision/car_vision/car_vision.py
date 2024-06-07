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

import json

DEBUG = True
BASE_DIR = str(pathlib.Path(__file__).resolve().parent)

#lower_bound for a reversed list
def lower_bound(arr, target):
    left, right = 0, len(arr)
    while left < right:
        mid = (left + right) // 2
        if arr[mid][0][1] <= target:
            right = mid
        else:
            left = mid + 1
    return left
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
        self.camera_subcriber = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)

        time_interval = 0.1 # seconds
        self.timer = self.create_timer(time_interval, self.timer_callback)
        
        self.frame = None  
        self.directions = [1,0,-1,0,1]

        # self.retrieve_camera_info()

        # Get the HSV values for the colors from a json file
        try:
            
            with open("/home/alexis/IntelligentRoboticsImplementation/ws/src/car_vision/car_vision/hsv_colors.json", 'r') as f:
                self.hsv_dict = json.load(f) 
        except FileNotFoundError:
            self.hsv_dict = {
                "green": [[0, 0, 0], [255, 255, 255]],
                "red": [[0, 0, 0], [255, 255, 255]],
                "yellow": [[0, 0, 0], [255, 255, 255]],
            }

        # Deserialize the dictionary and convert the values to numpy array
        self.hsv_dict = {key: [np.array(value[0]), np.array(value[1])] for key, value in self.hsv_dict.items()}

    def camera_callback(self, msg):
        try:
            self.frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            print("Frame received")
        except Exception as e:
            self.get_logger().info(str(e))

    def retrieve_camera_info(self):
        data = np.load(BASE_DIR + '/calibration.npz')
        self.repError = data['repError']
        self.cameraMatrix = data['cameraMatrix']
        self.distCoeffs = data['distCoeffs']
        self.rvecs = data['rvecs']
        self.tvecs = data['tvecs']

        self.get_logger().info('Camera info retrieved')


    def floodfill(self, frame, new_binary_image, x, y):
        stack = [(x, y)]
        new_binary_image[x][y] = 255
        frame[x][y] = 0

        while stack:
            x, y = stack.pop()
            for i in range(len(self.directions) - 1):
                new_x = x + self.directions[i]
                new_y = y + self.directions[i + 1]
                
                if new_x >= 0 and new_y >= 0 and new_x < frame.shape[0] and new_y < frame.shape[1]:
                    if frame[new_x][new_y] == 255 and new_binary_image[new_x][new_y] == 0:
                        stack.append((new_x, new_y))
                        new_binary_image[new_x][new_y] = 255
                        frame[new_x][new_y] = 0
        
        return new_binary_image
    
    def getCurvedLine(self, threshed_image):    
        height, width = threshed_image.shape
        scale = 0.2

        y1 = int(height - (height * scale))
        x_diff = int((width * scale) / 2)

        crop_img = threshed_image[y1:height, x_diff:width - x_diff]

        contours,hierarchy = cv2.findContours(crop_img,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        new_image = np.zeros((height, width), np.uint8)

        if (len(contours) < 1):
            return new_image
        
        max_contour = max(contours, key=cv2.contourArea) 
        
        for contour in max_contour:
            x, y = contour[0]
            if new_image[height - y - 1, x + x_diff - 1] != 1:
                self.floodfill(threshed_image, new_image, height - y - 1, x + x_diff - 1)

        return new_image

    def getWaypoints(self, show_image=False):
        if self.frame is None:
            return 0.0

        h, w, _ = self.frame.shape
        croped_frame = self.frame[h - h // 4:h, w // 4: w - w // 4].copy()
        h, w, _ = croped_frame.shape
        # Convert the image to grayscale
        imgray = cv2.cvtColor(croped_frame, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("Imgray", imgray)
        h, w = imgray.shape
        # Binarize the image if necessary
        _, binary_image = cv2.threshold(imgray, 90, 255, cv2.THRESH_BINARY)
        binary_image = cv2.bitwise_not(binary_image)
        #cv2.imshow("Binary Image", binary_image)

        kernel = np.ones((2,2), np.uint8)
        binary_image = cv2.erode(binary_image, kernel, iterations=1)

        binary_image = cv2.dilate(binary_image, kernel, iterations=1)
        

        #track_line = self.getCurvedLine(binary_image)
        track_line = binary_image

        
        # Set initial point at the bottom of the image and in the middle
        initial_point = (int(w / 2), h - 1)

        # Find contours of the binary image
        contours, _ = cv2.findContours(track_line, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if (len(contours) < 1):
            return 0.0
        
        # Get the biggest contour
        centerline_contour = max(contours, key=cv2.contourArea)

        # Get the center of the contour
        M = cv2.moments(centerline_contour)
        
        if M["m00"] == 0:
            return 0.0
        coords = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if show_image:
            cv2.drawContours(croped_frame, contours, -1, (0,0,255))
            cv2.circle(croped_frame, coords, 1, (255, 255, 0), -1)
            cv2.circle(croped_frame, initial_point, 1, (255, 255, 0), -1)
            self.camera_publisher.publish(CvBridge().cv2_to_imgmsg(croped_frame, "bgr8"))

        y = initial_point[0] - coords[0]
        x = initial_point[1] - coords[1]
        angle = np.degrees(np.arctan2(y, x))

        return angle
    
    def getColoredCirles(self):
        if self.frame is None:
            return []
        
        self.frame = cv2.resize(self.frame, (800,400))
        
        detected_outputs = []
        hsvFrame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        copy_frame = self.frame.copy()
        

        for color in self.hsv_dict.keys():
            unmasked_image = self.frame.copy()
            lower = self.hsv_dict[color][0]
            upper = self.hsv_dict[color][1]
            mask = cv2.inRange(hsvFrame, lower, upper)
            detected_outputs.append(cv2.bitwise_and(unmasked_image, unmasked_image, mask=mask))
            
        circles_pts = {}

        for detected_output, color in zip(detected_outputs, [DetectionEnumerator.YELLOW_CIRCLE, DetectionEnumerator.RED_CIRCLE, DetectionEnumerator.GREEN_CIRCLE]):
            if DEBUG:
                cv2.imshow("Masked image " + str(color), detected_output)
                cv2.waitKey(1)
                # cv2.destroyAllWindows()
            circles_pts[color] = []
            gray = cv2.cvtColor(detected_output, cv2.COLOR_BGR2GRAY)
            # blur = cv2.gaussianBlur(gray, 5)
            blur = cv2.GaussianBlur(gray,(9,9),2)

            #diialte 
            kernel = np.ones((5,5), np.uint8)
            dilate = cv2.dilate(blur, kernel, iterations=2)

            blur = cv2.GaussianBlur(dilate,(9,9),2)

            if DEBUG:
                cv2.imshow("Blur image " + str(color), blur)

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
                    
                    cv2.circle(copy_frame, (int(x), int(y)), int(r), color_rgb, 2)

            cv2.imshow('Detections', copy_frame)
            self.camera_publisher.publish(CvBridge().cv2_to_imgmsg(copy_frame, "bgr8"))
            print ("Publishing image")        
        return circles_pts

    def timer_callback(self):
        self.road_publisher.publish(Float32(data=self.getWaypoints(show_image=True)))
        return
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
        
        self.road_publisher.publish(Float32(data=self.getWaypoints()))
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
    