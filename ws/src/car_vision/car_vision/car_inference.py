import ultralytics
import time
import numpy as np
import pathlib
from enum import IntEnum
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from car_interfaces.msg import Detection
from car_interfaces.msg import Inference, InferenceArray
from sensor_msgs.msg import Image, CompressedImage
import json

import cv2

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

BASE_PATH = pathlib.Path(__file__).parent.absolute()
print("BASE_PATH: ", BASE_PATH)

class carInference(Node):
    def __init__(self):
        super().__init__('car_inference')
        self.model = ultralytics.YOLO(f'{BASE_PATH}/yolov8n_custom/weights/best.pt')
        # Get the HSV values for the colors from a json file
        try:
            
            with open("/workspace/robotics/IntelligentRoboticsImplementation/ws/src/car_vision/car_vision/hsv_colors.json", 'r') as f:
                self.hsv_dict = json.load(f) 
        except FileNotFoundError:
            print("HSV file not found")
            self.hsv_dict = {
                "green": [[0, 0, 0], [255, 255, 255]],
                "red": [[0, 0, 0], [255, 255, 255]],
                "yellow": [[0, 0, 0], [255, 255, 255]],
            }
        self.frame=None
        self.bridge = CvBridge()
        # self.image_sub = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.detection_publisher = self.create_publisher(Detection, '/carDetections', 10)
        self.comp_sub = self.create_subscription(CompressedImage, '/video_source/compressed', self.image_callback, 10)
        self.visualization_pub = self.create_publisher(Image, 'carInferenceVisualization', 10)
        self.circles_image_publisher = self.create_publisher(Image, '/image_circles', 10)
        self.detection_pub = self.create_publisher(InferenceArray, 'carInferences', 10)
        self.prevtime = time.time()
        self.frames = 0
        self.threshold = 0.5
        
        circles_time_interval = 0.2
        self.last_time_circles = time.time()
        self.circles_timer = self.create_timer(circles_time_interval, self.circles_timer_callback)
        
        self.prev_time = time.time()

        self.yolov8_warmup(10, False)

        # Print all labels and their corresponding class ids
        print (self.model.names)


    def image_callback(self, msg):
        print(f"Image received with time {self.prev_time - time.time()}")
        self.prev_time = time.time()
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.frame = frame.copy()
        # frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, verbose=False)
        boxes, confidences, classids = self.generate_boxes_confidences_classids_v8(results)

        # Create InferenceArray message
        inferenceArray = []
        inferenceArrayMsg = InferenceArray()
        for i in range(len(boxes)):
            inferenceMsg = Inference()
            inferenceMsg.bbox = boxes[i]
            inferenceMsg.confidence = confidences[i]
            inferenceMsg.class_id = int(classids[i])
            inferenceArray.append(inferenceMsg)

            x, y, w, h = boxes[i]
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            class_name = self.model.names[classids[i]]
            cv2.putText(frame, class_name, (x, y+h+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print(f"Detected {class_name} with confidence {confidences[i]}")

            
        # cv2.imshow("frame", frame)
        # cv2.waitKey(1)
        # publish visualization
        self.visualization_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        inferenceArrayMsg.detections = inferenceArray
        print(inferenceArray)
        self.detection_pub.publish(inferenceArrayMsg)
    
    def yolov8_warmup(self, repetitions=1, verbose = False):
        # Warmup model
        startTime = time.time()
        # create an empty frame to warmup the model
        for i in range(repetitions):
            warmupFrame = np.zeros((360, 640, 3), dtype=np.uint8)
            self.model.predict(source=warmupFrame, verbose=verbose)
        print(f"Model warmed up in {time.time() - startTime} seconds")

    def generate_boxes_confidences_classids_v8(self, outs):
            boxes = []
            confidences = []
            classids = []

            for out in outs:
                    for box in out.boxes:
                        x1, y1, x2, y2 = [round(x) for x in box.xyxy[0].tolist()]
                        class_id = box.cls[0].item()
                        prob = round(box.conf[0].item(), 2)
                        if prob > self.threshold:
                            # Append to list
                            boxes.append([x1, y1, x2-x1, y2-y1])
                            confidences.append(float(prob))
                            classids.append(class_id)
        
            return boxes, confidences, classids
    
    def getColoredCirles(self):
        print("searching circles")
        if self.frame is None:
            return []
        #print(f"circles frame: {self.frame.shape}")
        frame = self.frame
        if DEBUG:
            cv2.imshow("Original image", frame)
        
        detected_outputs = []
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        copy_frame = frame.copy()
        published_image = np.zeros((frame.shape[0], frame.shape[1], 3), np.uint8) 
        for color in self.hsv_dict.keys():
            # print(f"Color mask: {self.hsv_dict[color]}")
            unmasked_image = frame.copy()
            #print(f"unmasked_image: {unmasked_image.shape}")
            
            lower = self.hsv_dict[color][0]
            upper = self.hsv_dict[color][1]
            lower = np.array(lower, np.uint8)
            upper = np.array(upper, np.uint8)
            mask = cv2.inRange(hsvFrame, lower, upper)
            #print(f"mask: {mask.shape}")
            binary_image = cv2.bitwise_and(unmasked_image, unmasked_image, mask=mask)
            # if it is red, add the other side of the color spectrum
            if color == "red":
                lower = self.hsv_dict[color][0]
                upper = self.hsv_dict[color][1]
                lower[0] = 145
                upper[0] = 190
                lower = np.array(lower, np.uint8)
                upper = np.array(upper, np.uint8)
                                        
                mask2 = cv2.inRange(hsvFrame, lower, upper)
                mask = cv2.bitwise_or(mask, mask2)
            # cv2.imshow("Masked image " + str(color), binary_image)
            detected_outputs.append(cv2.bitwise_and(unmasked_image, unmasked_image, mask=mask))
        
        
        circles_pts = {}
        
        

        for detected_output, color in zip(detected_outputs, [DetectionEnumerator.YELLOW_CIRCLE, DetectionEnumerator.RED_CIRCLE, DetectionEnumerator.GREEN_CIRCLE]):
            # make the frame black and white, binary (all white or all black)
            detected_output = cv2.cvtColor(detected_output, cv2.COLOR_BGR2GRAY)
            _, detected_output = cv2.threshold(detected_output, 30, 255, cv2.THRESH_BINARY)
            
            # append to the black image to publish the binary image, coloured according to the color
            if color == DetectionEnumerator.RED_CIRCLE:
                mask = np.zeros((detected_output.shape[0], detected_output.shape[1], 3), np.uint8)
                mask[:,:] = [0, 0, 255]
                mask_filtered = cv2.bitwise_and(mask, mask, mask=detected_output)
                published_image = cv2.add(published_image, mask_filtered)
            elif color == DetectionEnumerator.GREEN_CIRCLE:
                mask = np.zeros((detected_output.shape[0], detected_output.shape[1], 3), np.uint8)
                mask[:,:] = [0, 255, 0]
                mask_filtered = cv2.bitwise_and(mask, mask, mask=detected_output)
                published_image = cv2.add(published_image, mask_filtered)
            else:
                mask = np.zeros((detected_output.shape[0], detected_output.shape[1], 3), np.uint8)
                mask[:,:] = [0, 255, 255]
                mask_filtered = cv2.bitwise_and(mask, mask, mask=detected_output)
                published_image = cv2.add(published_image, mask_filtered)
            # make three dimensional (rgb)
            detected_output = cv2.merge((detected_output, detected_output, detected_output))
            # if DEBUG:
            # cv2.imshow("Masked binary image " + str(color), detected_output)
            # cv2.waitKey(1)
                # cv2.destroyAllWindows()
            # erode and dilate
            erodeKernel, dilateKernel = np.ones((1,1), np.uint8), np.ones((7,7), np.uint8)
            #detected_output = cv2.erode(detected_output, erodeKernel, iterations=2)
            #detected_output = cv2.dilate(detected_output, dilateKernel, iterations=3)
            
            # blur
            detected_output = cv2.GaussianBlur(detected_output, (7,7), 2)
            if DEBUG:
                cv2.imshow("Final image " + str(color), detected_output)
                cv2.waitKey(1)
                # cv2.destroyAllWindows()
            
            # circle detection
            # make the image be of type uin8, either 0 or 255 
            detected_output = cv2.cvtColor(detected_output, cv2.COLOR_BGR2GRAY)
            
            circles = cv2.HoughCircles(detected_output, cv2.HOUGH_GRADIENT, 1, 100, param1=50, param2=15, minRadius=10, maxRadius=50)
            circles_pts[color] = []
            if circles is not None and len(circles) > 0:
                for circle in circles[0, :]:
                    # cv2.circle(copy_frame, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
                    # circles_pts[color] = circle[2]
                    print(f"FOUND {str(color)} circle")
                    # draw a cross on the center of the circle
                    draw_color = (100, 255, 0) if color == DetectionEnumerator.GREEN_CIRCLE else (0, 100, 255) if color == DetectionEnumerator.RED_CIRCLE else (100, 255, 255)
                    copy_frame = cv2.circle(copy_frame, (int(circle[0]), int(circle[1])), int(circle[2]), draw_color, 2)
                    circles_pts[color].append(float(circle[2]))
        if DEBUG:
            cv2.imshow("Circles image ", copy_frame)
        
        self.circles_image_publisher.publish(CvBridge().cv2_to_imgmsg(published_image, "bgr8"))
        #     circles_pts[color] = []
        #     gray = cv2.cvtColor(detected_output, cv2.COLOR_BGR2GRAY)
        #     # blur = cv2.gaussianBlur(gray, 5)
        #     blur = cv2.GaussianBlur(gray,(9,9),2)

        #     #diialte 
        #     kernel = np.ones((5,5), np.uint8)
        #     dilate = cv2.dilate(blur, kernel, iterations=2)

        #     blur = cv2.GaussianBlur(dilate,(9,9),2)

        #     if DEBUG:
        #         cv2.imshow("Blur image " + str(color), blur)

        #     # Morph open 
        #     thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        #     kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        #     opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)

        #     # Find contours and filter using contour area and aspect ratio
        #     cnts = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #     cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        #     for c in cnts:
        #         peri = cv2.arcLength(c, True)
        #         approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        #         area = cv2.contourArea(c)
        #         if len(approx) > 5 and area > 1000 and area < 500000:
        #             ((x, y), r) = cv2.minEnclosingCircle(c)
        #             #circle_pts.append((int(x), int(y), int(r)))
        #             circles_pts[color].append(r)
        #             if color == DetectionEnumerator.RED_CIRCLE:
        #                 color_rgb = (0, 0, 255)
        #             elif color == DetectionEnumerator.GREEN_CIRCLE:
        #                 color_rgb = (0, 255, 0)
        #             else:
        #                 color_rgb = (0, 255, 255)
                    
        #             cv2.circle(copy_frame, (int(x), int(y)), int(r), color_rgb, 2)

        #     cv2.imshow('Detections', copy_frame)
        #     self.camera_publisher.publish(CvBridge().cv2_to_imgmsg(copy_frame, "bgr8"))
        #     print ("Publishing image")    
        if DEBUG:
            cv2.waitKey(1)   
        
        #self.camera_publisher_colors.publish(CvBridge().cv2_to_imgmsg(copy_frame, "bgr8"))
        return circles_pts
    
    def circles_timer_callback(self):
        print(f"[CIRCLES] Time since last run: {time.time()-self.last_time_circles}")
        self.last_time_circles = time.time()
        if self.frame is None:
            return
        
        detection_msg = Detection()
        start_time = time.time()
        circle_pts = self.getColoredCirles()
        print(f"Circle finding took: {time.time()-start_time}")
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
        carInference_ = carInference()
        rclpy.spin(carInference_)
        carInference_.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
    