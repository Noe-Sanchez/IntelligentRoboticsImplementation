import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Pose2D
import numpy as np

# Odometry node
class Controller(Node):
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
        #self.point_subscriber = self.create_subscription(Pose2D, 'Point', self.point_callback, qos_profile=qos_profile_sub) # odometry topic subscriber

        # Velocity publisher
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10) # velocity topic publisher
        self.vel_period = 0.05 # velocity publishing period (seconds)
        self.vel_timer = self.create_timer(self.vel_period, self.vel_callback) # velocity publishing timer
        self.msg_vel = Twist() # velocity message
        self.variablesInit()

        
        self.get_logger().info('Controller node successfully initialized!!!')
        
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
        self.Kv = 0.5 # proportional linear velocity constant
        self.Kw = 0.5 # proportional angular velocity constant

        # Robot pose variables
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0

        # Robot velocity variables
        self.v = 0.0
        self.w = 0.0

        # Angle and robot distance between points
        self.psi = 0
        self.time = 0
        self.distance = 0
        self.maxerror = 0

        # Flags when angVelocity and Linear Velocity end
        self.vfinish = True
        self.afinish = True

        # How many points have been crossed
        self.count = 0

        # Points
        self.puntos = [[0,0], [1,0], [1,1], [0,1]]

        # Max & min velocities
        self.maxang = 1.25
        self.maxlinear = 0.3
        self.minlinear = 0.05
        self.minang = 0.4

    # Odometry callback
    def vel_callback(self):

        # After one part of the trajectory is ready, update points
        if self.vfinish and self.afinish:
            self.distance = 0
            self.vfinish = False
            self.afinish = False
            if self.count < len(self.puntos) - 1: 
                self.xT = self.puntos[self.count+1][0]
                self.yT = self.puntos[self.count+1][1]
                self.count +=1
            else:
                self.count = 0
                self.xT = self.puntos[self.count][0]
                self.yT = self.puntos[self.count][1]
                #self.finish = True
            
        # Obtain distance and angle between points
        deltay = self.yT - self.y
        deltax = self.xT - self.x
        error_d = np.sqrt((deltax)**2 + (deltay)**2)
        thetaT = np.arctan2(self.yT, self.xT)
        errorDer = 0

        #Narrow robot angle between [0, 2pi]
        if(self.psi > 2*np.pi): self.psi -= 2*np.pi
        elif(self.psi < 0): self.psi += 2*np.pi
    
        #Obtain angular error and its complementary error, and see which is the lowest
        errorIzq = self.psi - thetaT
        if(errorIzq < 0): errorDer = errorIzq + 2*np.pi
        elif(errorIzq > 0): errorDer = errorIzq - 2*np.pi

        if(abs(errorIzq) < abs(errorDer)): error_ang = errorIzq
        else: error_ang = errorDer

        #First, let robot turn until error_ang is diminutive
        if (error_ang <= -0.05 or error_ang >= 0.05):
            self.w = self.Kw * error_ang
        else:
            self.w = 0.0
            self.afinish = True

        #If robot turn is finish, let robot have linear velocity
        if self.afinish:
            if error_d > 0.01 : self.v = self.Kv * error_d
            else: self.vfinish = True
        else:
            self.v = 0.0

        #if self.finish:
        #    self.v = 0.0
        #    self.w = 0.0

        #Publish velocities
        if self.v > self.maxlinear:
            self.msg_vel.linear.x = self.maxlinear
        if 0.0 < self.v and self.v < self.minlinear:
            self.msg_vel.linear.x = self.minlinear
        if self.v > self.maxang:
            self.msg_vel.linear.x = self.maxang
        if 0.0 < self.v and self.v < self.minang:
            self.msg_vel.linear.x = self.minang
        self.vel_publisher.publish(self.msg_vel)
        
        self.get_logger().info('Velocities: {}'.format(self.msg_vel))
        self.get_logger().info('Distance: {}'.format(self.distance))
        self.get_logger().info('Angle: {}'.format(self.psi))
        self.get_logger().info('Punto: {}'.format(self.puntos[self.count]))
        self.get_logger().info('theta: {}'.format(thetaT))
        self.get_logger().info('error_d: {}'.format(error_d))

    # Update left motor angular velocity
    def odom_callback(self, msg):
        self.theta = msg.theta
        self.x = msg.x
        self.y = msg.y
        self.get_logger().info('Odom: {}'.format(msg))

# Run node
def main(args=None):
    rclpy.init(args=args)
    ctr = Controller()
    rclpy.spin(ctr)
    ctr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    