import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32
import numpy as np

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.subscription = self.create_subscription(Int32, '/number_of_points', self.number_of_points_callback, 10)
        self.publisher = self.create_publisher(PoseArray, '/trajectory', 10)
        self.diameter = 0.3

    def number_of_points_callback(self, msg):
        num_points = msg.data
        trajectory = PoseArray()
        trajectory.header.frame_id = 'map'  # Assuming the trajectory is defined in the 'map' frame

        # Generate points for the polygon trajectory
        for i in range(num_points):
            pose = Pose()
            # Generate poses on a regular polygon with radius 1 meter
            pose.position.x = self.diameter * np.cos(2 * np.pi * i / num_points)
            pose.position.y = self.diameter* np.sin(2 * np.pi * i / num_points)
            pose.position.z = 0.0  # Assuming 2D trajectory, so z-coordinate is 0
            pose.orientation.w = 1.0  # Assuming no rotation
            trajectory.poses.append(pose)

        # Publish the trajectory as a PoseArray
        self.publisher.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    trajectory_generator = TrajectoryGenerator()
    rclpy.spin(trajectory_generator)
    trajectory_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
