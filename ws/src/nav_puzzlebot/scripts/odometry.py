#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from std_srvs.srv import Empty
import numpy as np

# Odometry node
class Odometry(Node):
    def __init__(self):
        
        # Node name
        super().__init__('odometry_node')

        # Define QoS
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Angular velocity subscribers
        self.wL_subscriber = self.create_subscription(Float32, 'VelocityEncL', self.wL_callback, qos_profile=qos_profile_sub) # wL topic subscriber
        self.wR_subscriber = self.create_subscription(Float32, 'VelocityEncR', self.wR_callback, qos_profile=qos_profile_sub) # wR topic subscriber

        # Odometry reset
        self.odom_reset_srv = self.create_service(Empty, 'reset_odom', self.reset_odom_callback) # odometry reset service
        # Odometry publisher
        self.odom_publisher = self.create_publisher(Pose2D, 'odom', qos_profile=qos_profile_sub) # odometry topic publisher
        self.odom_period = 0.05 # odometry publishing period (seconds)
        self.odom_timer = self.create_timer(self.odom_period, self.odom_callback) # odometry publishing timer
        self.msg_odom = Pose2D() # odometry message
        self.variablesInit()

        
        self.get_logger().info('Odometry node successfully initialized!!!')
        
    # Variables inicialization
    def variablesInit(self):
        
        # Pose2D message
        self.msg_odom.theta = 0.0
        self.msg_odom.x = 0.0
        self.msg_odom.y = 0.0

        # Robot constants
        self.r = 0.05 # wheel radius (meters)
        self.l = 0.19 # distance between wheels (meters)
        self.ang_const = self.r / (self.l) # angular constant for theta calculation
        self.lin_const = self.r / 2 # linear constant for x and y calculation
        self.dt = self.odom_period # time between samples (seconds)

        # Robot pose variables
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta_prev = 0.0
        self.x_prev = 0.0
        self.y_prev = 0.0

        # Robot measured variables
        self.wL = 0.0
        self.wR = 0.0

    # Odometry callback
    def odom_callback(self):

        # Current pose calculations
        self.theta = self.theta_prev + self.ang_const * (self.wR - self.wL) * self.dt
        self.x = self.x_prev + self.lin_const * (self.wR + self.wL) * np.cos(self.theta_prev) * self.dt
        self.y = self.y_prev + self.lin_const * (self.wR + self.wL) * np.sin(self.theta_prev) * self.dt
        
        if(self.theta > 2*np.pi): self.theta -= 2*np.pi
        if(self.theta < 0): self.theta += 2*np.pi

        # Update message
        self.msg_odom.theta = self.theta
        self.msg_odom.x = self.x
        self.msg_odom.y = self.y

        # Publish message
        self.odom_publisher.publish(self.msg_odom)
        self.get_logger().info('Odometry: {}'.format(self.msg_odom))
        #self.get_logger().info('Omegas: {} {}'.format(self.wL, self.wR))

        # Update previous pose
        self.theta_prev = self.theta
        self.x_prev = self.x
        self.y_prev = self.y

    # Update left motor angular velocity
    def wL_callback(self, msg):
        self.wL = msg.data
        #self.get_logger().info('OmegaL: {}'.format(msg.data))
    
    # Update right motor angular velocity
    def wR_callback(self, msg):
        self.wR = msg.data
        #Sself.get_logger().info('OmegaR: {}'.format(msg.data))
        
    def reset_odom_callback(self, request, response):
        self.variablesInit()
        return response

# Run node
def main(args=None):
    rclpy.init(args=args)
    odom = Odometry()
    rclpy.spin(odom)
    odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    