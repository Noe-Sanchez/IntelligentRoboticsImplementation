import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Pose2D
import numpy as np

# Odometry node
class Odometry(Node):
    def __init__(self):
        
        # Node name
        super().__init__('controller_node')

        # Define QoS
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Angular velocity subscribers
        self.odom_subscriber = self.create_subscription(Pose2D, 'odom', self.odom_callback, qos_profile=qos_profile_sub) # odometry topic subscriber

        # Odometry publisher
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10) # velocity topic publisher
        self.vel_period = 0.05 # velocity publishing period (seconds)
        self.vel_timer = self.create_timer(self.odom_period, self.vel_callback) # velocity publishing timer
        self.msg_vel = Twist() # velocity message
        self.variablesInit()

        
        self.get_logger().info('Odometry node successfully initialized!!!')
        
    # Variables inicialization
    def variablesInit(self):
        
        # Twist message
        self.msg_vel.linear.x = 0.0
        self.msg_vel.linear.y = 0.0
        self.msg_vel.linear.z = 0.0
        self.msg_vel.angular.x = 0.0
        self.msg_vel.angular.y = 0.0
        self.msg_vel.angular.z = 0.0
        
        # Robot constants
        self.kp = 0.05 # proportional constant
        self.ki = 0.19 # integration constant

        # Robot pose variables
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0

    # Odometry callback
    def vel_callback(self):
        m

    # Update left motor angular velocity
    def odom_callback(self, msg):
        self.theta = msg.theta
        self.x = msg.x
        self.y = msg.y
        #self.get_logger().info('OmegaL: {}'.format(msg.data))

# Run node
def main(args=None):
    rclpy.init(args=args)
    odom = Odometry()
    rclpy.spin(odom)
    odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    