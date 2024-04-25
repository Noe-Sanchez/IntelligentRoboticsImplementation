import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import numpy as np

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.subscription = self.create_subscription(Int32,'/number_of_points',self.number_of_points_callback,10)
        self.publisher = self.create_publisher(Point, '/trajectory', 10)

    def publish_point_callback(self, trajectory):
        for point in trajectory:
            self.publisher.publish(point)

    def number_of_points_callback(self, msg):
        num_points = msg.data
        trajectory = []

        # Generate points for the polygon trajectory
        for i in range(num_points):
            point = Point()
            # Generate points on a regular polygon with radius 1 meter
            point.x = 1.0 * np.cos(2 * np.pi * i / num_points)
            point.y = 1.0 * np.sin(2 * np.pi * i / num_points)
            point.z = 0.0  # Assuming 2D trajectory, so z-coordinate is 0
            trajectory.append(point)

        # Publish the trajectory points to the '/trajectory' topic
        self.publish_point_callback(trajectory)

def main(args=None):
    rclpy.init(args=args)
    trajectory_generator = TrajectoryGenerator()
    rclpy.spin(trajectory_generator)
    trajectory_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
