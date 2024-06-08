import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge


from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

class Publisher(Node):
	def __init__(self):
		super().__init__('imgcomp_pub')
		self.declare_parameter('width',100)
		self.declare_parameter('height',50)

		self.width = self.get_parameter('width').get_parameter_value().integer_value
		self.height = self.get_parameter('height').get_parameter_value().integer_value
		self.camera_subcriber = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
		self.comp_publisher = self.create_publisher(CompressedImage, '/video_source/compressed', 10)
		self.msg_comp = CompressedImage()

	def camera_callback(self, msg):
		try:
		    self.frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
		    self.frame = cv2.resize(self.frame, (self.width, self.height))
		    print("Frame received")
		    result, encoded = cv2.imencode('.jpg', self.frame)
		    self.msg_comp.format = 'jpeg'
		    self.msg_comp.data = encoded.tobytes()
		    self.comp_publisher.publish(self.msg_comp)
		except Exception as e:
		    self.get_logger().info(str(e))


def main(args=None):
    rclpy.init(args=args)

    try:
        carVision_ = Publisher()
        rclpy.spin(carVision_)
        carVision_.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()

