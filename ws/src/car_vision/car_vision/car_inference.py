import ultralytics
import time
import numpy as np
import pathlib

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from car_interfaces.msg import Inference, InferenceArray
from sensor_msgs.msg import Image

import cv2


BASE_PATH = pathlib.Path(__file__).parent.absolute()
print("BASE_PATH: ", BASE_PATH)

class carInference(Node):
    def __init__(self):
        super().__init__('car_inference')
        self.model = ultralytics.YOLO(f'{BASE_PATH}/yolov8n_custom/weights/best.pt')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.visualization_pub = self.create_publisher(Image, 'carInferenceVisualization', 10)
        self.detection_pub = self.create_publisher(InferenceArray, 'carInferences', 10)
        self.prevtime = time.time()
        self.frames = 0
        self.threshold = 0.7

        self.yolov8_warmup(10, False)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
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
            cv2.putText(frame, class_name, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            
        # cv2.imshow("frame", frame)
        # cv2.waitKey(1)
        # publish visualization
        self.visualization_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        inferenceArrayMsg.detections = inferenceArray

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
    