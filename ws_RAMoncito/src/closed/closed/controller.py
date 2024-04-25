import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Pose2D, Point
import numpy as np
import math

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
        self.point_subscriber = self.create_subscription(Point, '/trajectory', self.point_callback, qos_profile=qos_profile_sub) # odometry topic subscriber

        # Velocity publisher
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10) # velocity topic publisher
        self.vel_period = 0.05 # velocity publishing period (seconds)
        self.vel_timer = self.create_timer(self.vel_period, self.vel_callback) # velocity publishing timer
        self.msg_vel = Twist() # velocity message
        self.variablesInit()

        
        self.get_logger().info('Controller node successfully initialized!!!')


    # def generate_polygon(self, num_points, distance):
    #         polygon = []
    #         angle = 2 * math.pi / num_points
    #         for i in range(num_points):
    #             x = distance * math.cos(i * angle)
    #             y = distance * math.sin(i * angle)
    #             polygon.append([x, y])
    #         return polygon

    # Variables inicialization


    def point_callback(self,msg):
        for i in range (msg)
        self.puntos

    def variablesInit(self):
        
        # Twist message
        self.msg_vel.linear.x = 0.0
        self.msg_vel.linear.y = 0.0
        self.msg_vel.linear.z = 0.0
        self.msg_vel.angular.x = 0.0
        self.msg_vel.angular.y = 0.0
        self.msg_vel.angular.z = 0.0
        
        # Robot constants
        self.Kv = 0.25 # proportional linear velocity constant
        self.Kw = 0.25 # proportional angular velocity constant

        # Robot pose variables
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0

        # Robot velocity variables
        self.v = 0.0
        self.w = 0.0

        # Robot distance between points
        self.distance = 0
        self.distance_t = 0

        # Flags when angVelocity and Linear Velocity end
        self.vfinish = True
        self.afinish = True

        # How many points have been crossed
        self.count = 0

        # Points
        self.puntos = self.generate_polygon(4,0.3)        
        
        self.xT = self.puntos[self.count][0]
        self.yT = self.puntos[self.count][1]

        self.TARGET_TOLERANCE = 0.02


        # Max & min velocities
        self.maxang = 1.25
        self.maxlinear = 0.3
        self.minlinear = 0.05
        self.minang = 0.4


    def getVelocity(self):

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
        self.error_d = np.sqrt((deltax)**2 + (deltay)**2)
        self.thetaT = np.arctan2(deltay, deltax)
        errorDer = 0
    
        #Obtain angular error and its complementary error, and see which is the lowest
        errorIzq = self.thetaT - self.theta
        if(errorIzq < 0): errorDer = errorIzq + 2*np.pi
        elif(errorIzq > 0): errorDer = errorIzq - 2*np.pi

        if(abs(errorIzq) < abs(errorDer)): self.error_ang = errorIzq
        else: self.error_ang = errorDer
        
        #First, let robot turn until self.error_ang is diminutive
        if (abs(self.error_ang) >= 0.05):
            self.w = self.Kw * self.error_ang
        else:
            self.w = 0.0
            self.afinish = True

        #If robot turn is finish, let robot have linear velocity
        if self.afinish:
            if self.error_d > 0.05 : 
                self.v = self.Kv * self.error_d
            else: 
                self.vfinish = True
        else:
            self.v = 0.0

        #if self.finish:
        #    self.v = 0.0
        #    self.w = 0.0

        # self.distance_t = math.sqrt((self.xT - self.x)**2 + (self.yT - self.y)**2)
        
        # deltay = self.yT - self.y
        # deltax = self.xT - self.x
        
        # # abs_delta_y = abs(self.yT) - abs(self.y)
        # # abs_delta_x = abs(self.xT) - abs(self.x)

        # if (self.distance_t < self.TARGET_TOLERANCE):
        #     self.count+=1
        #     if self.count >= len(self.puntos):
        #         self.count=0
        #     self.xT = self.puntos[self.count][0]
        #     self.yT = self.puntos[self.count][1]
        

        # # self.distance = np.sqrt((self.x)**2 + (self.y)**2)
        
        # self.error_d = np.sqrt((deltax)**2 + (deltay)**2)
        # self.thetaT = np.arctan2(deltay, deltax)
        # errorDer = 0
        
        # #Obtain angular error and its complementary error, and see which is the lowest
        # errorIzq = self.thetaT * self.theta 
        # if(errorIzq < 0): errorDer = errorIzq + 2*np.pi
        # elif(errorIzq > 0): errorDer = errorIzq - 2*np.pi

        # if(abs(errorIzq) < abs(errorDer)): self.error_ang = errorIzq
        # else: self.error_ang = errorDer

        # if self.error_ang > 0.05:
        #     self.w = self.Kw * self.error_ang
        #     self.v = 0.0
        # else:
        #     self.v = self.Kv * self.distance_t
        #     self.w = 0.0



        # #Publish velocities
        
        
        
        if abs(self.v) > self.maxlinear:
            self.msg_vel.linear.x = (abs(self.v) / self.v) * self.maxlinear
        elif 0.0 < abs(self.v) and self.v < self.minlinear:
            self.msg_vel.linear.x = (abs(self.v) / self.v) * self.minlinear
        else:
            self.msg_vel.linear.x = self.v

        if abs(self.w) > self.maxang:
            self.msg_vel.angular.z = (abs(self.w) / self.w) * self.maxang
        elif 0.0 < abs(self.w) and self.w < self.minang:
            self.msg_vel.angular.z = (abs(self.w) / self.w) * self.minang
        else:
            self.msg_vel.angular.z = self.w

    # Odometry callback
    def vel_callback(self):

        self.getVelocity()
        self.vel_publisher.publish(self.msg_vel)
        

        # print(self.msg_vel, end='')
        # print(self.distance, end='')
        # print(self.theta, end='')
        # print(self.puntos[self.count], end='')
        # print(self.thetaT, end='')
        print("Distance error: ", self.error_d)
        print("Angle error: ", self.error_ang)
        print("Local car distance: ",self.distance)
        print("Target distance: ", self.distance_t)
        print("Difference between local car distance and target distance: ", self.distance_t-self.distance)
        # self.get_logger().info('Velocities: {}'.format(self.msg_vel))
        # self.get_logger().info('Distance: {}'.format(self.distance))
        # self.get_logger().info('Angle: {}'.format(self.theta))
        #self.get_logger().info('Punto: {}'.format(self.puntos[self.count]))
        # self.get_logger().info('theta: {}'.format(self.thetaT))
        # self.get_logger().info('error_d: {}'.format(self.error_d))
        # self.get_logger().info('error_a: {}'.format(self.error_ang))

    # Update left motor angular velocity
    def odom_callback(self, msg):
        self.theta = msg.theta
        self.x = msg.x
        self.y = msg.y
        #self.get_logger().info('Odom: {}'.format(msg))

# Run node
def main(args=None):
    rclpy.init(args=args)
    ctr = Controller()
    rclpy.spin(ctr)
    ctr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    