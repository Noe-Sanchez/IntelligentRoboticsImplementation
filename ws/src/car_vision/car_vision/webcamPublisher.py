import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("webcam_pub")
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

        self.pub = self.create_publisher(Image, "/video_stream", 10)

    def run(self):
        while True:
            try:
                r, frame = self.cap.read()
                if not r:
                    return
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

                # # BGR8
                # self.bgr8pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

                # # RGB8
                # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # self.rgb8pub.publish(self.bridge.cv2_to_imgmsg(frame_rgb, "rgb8"))

                # # MONO8
                # frame_mono = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                #self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "mono8"))

            except CvBridgeError as e:
                print(e)
                self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

