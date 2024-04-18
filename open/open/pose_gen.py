import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int16

import yaml
import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

class PoseGenerator(Node):
    def __init__(self):
        super().__init__('pose_gen')

        self.poses_publisher = self.create_publisher(PoseArray, 'pose', 10)
        self.flag_subscriber = self.create_subscription(Int16, 'trajectory_number', self.flag_callback, 10)

        self.get_logger().info('Trajectory node online')
        
        self.poses = PoseArray
        self.desired_trajectory = 0
        
        #print(get_package_prefix('open'))
        mock = get_package_prefix('open').split('/')
        mock.pop(-1)
        mock.pop(-1)
        self.config_file = '/'.join(mock)
        #print(self.config_file)

        self.config_file = os.path.join(
            self.config_file,
            'src',
            'IntelligentRoboticsImplementation',
            'open',
            'config',
            'trajectories.yaml'
        )

        with open(self.config_file, 'r') as file:
            self.trajectories = yaml.safe_load(file)
        print(self.trajectories)
        #print((len(self.trajectories['trajectory1'])/2)-1)
        #print((len(self.trajectories['trajectory1'])/2))
        #print((len(self.trajectories['trajectory1'])))

    def flag_callback(self, msg):
        self.desired_trajectory = msg.data

        if(self.desired_trajectory > 0 and self.desired_trajectory < 3):
            traj = 'trajectory' + str(self.desired_trajectory)
            self.poses = PoseArray()

            for i in range(int((len(self.trajectories[traj])/4))):
                current_pose = Pose()
                current_pose.position.x = self.trajectories[traj]['x'+str(i+1)]
                current_pose.position.y = self.trajectories[traj]['y'+str(i+1)]
                current_pose.orientation.x = self.trajectories[traj]['u'+str(i+1)]
                current_pose.orientation.y = self.trajectories[traj]['w'+str(i+1)]
                current_pose.orientation.z = 0.0

                self.poses.poses.append(current_pose)
        else:
            self.get_logger().error("Invalid trajectory")

        self.poses_publisher.publish(self.poses)

def main(args=None):
    rclpy.init(args=args)
    m_s = PoseGenerator()
    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    