#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Pose2D, Point, PoseArray, Pose
from std_msgs.msg import Int16, Float32
import numpy as np
import math
from car_interfaces.msg import Detection, Inference, InferenceArray

USING_POSE = False
USING_LINE = True
USING_DETECTIONS = True

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
        if USING_POSE:
            self.odom_subscriber = self.create_subscription(Pose2D, 'odom', self.odom_callback, qos_profile=qos_profile_sub) # odometry topic subscriber
            self.point_subscriber = self.create_subscription(PoseArray, '/trajectory', self.get_puntos_callback, 10) # pose topic subscriber

        # Velocity publisherself.msg_vel = Twist() # velocity message
        self.vel_period = 0.05 # velocity publishing period (seconds)
        self.msg_vel = Twist() # velocity message
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10) # velocity topic publisher
        
        if USING_POSE:
            self.point_publisher = self.create_publisher(Int16, '/number_of_points', 10)
        
        if USING_LINE:
            self.road_publisher = self.create_subscription(Float32, '/roadError', self.line_callback, 1)
        
        if USING_DETECTIONS:
            self.detection_subscriber = self.create_subscription(InferenceArray, '/carInferences', self.get_vel_multipler_inference, 10)
            self.velocity_multiplier_suscriber = self.create_subscription(Detection, '/carDetections', self.get_vel_multipler, 10)
        
        self.vel_timer = self.create_timer(self.vel_period, self.vel_callback) # velocity publishing timer

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
        self.Kv = 0.25 # proportional linear velocity constant
        self.Kw = 0.25 # proportional angular velocity constant
        self.KwL = 0.01 # proportional angular velocity constant

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
        self.finish_config = True

        # How many points have been crossed
        self.count = 0

        # Points
        self.puntos = []     
        
        self.xT = 0
        self.yT = 0

        self.TARGET_TOLERANCE = 0.02


        # Max & min velocities
        self.maxang = 1.25
        self.maxlinear = 0.3
        self.minlinear = 0.05
        self.minang = 0.4
        self.velocity_multiplier = 1.0

    def get_puntos_callback(self,msg):
        if(self.finish_config == False):
            for pose in msg.poses:
                self.punto = []
                self.punto.append(pose.position.x)
                self.punto.append(pose.position.y)
                self.puntos.append(self.punto)
            
            self.finish_config = True


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

    #Velocity multiplier callback
    def get_vel_multipler_inference(self, msg):
        # Iterate through InferenceArray msg
        # uint8 FORWARD = 0
        # uint8 GIVEAWAY = 1
        # uint8 ROADWORK = 2
        # uint8 ROUNDABOUT = 3
        # uint8 STOP = 4
        # uint8 TURN_RIGHT = 5
        # uint8 TURN_LEFT = 6

        for i in range(len(msg.detections)):
            if(msg.detections[i].class_id == msg.detections[i].FORWARD):
                self.velocity_multiplier = 1.0 # TODO
            elif(msg.detections[i].class_id == msg.detections[i].GIVEAWAY):
                self.velocity_multiplier = 0.5
            elif(msg.detections[i].class_id == msg.detections[i].ROADWORK):
                self.velocity_multiplier = 0.0
            elif(msg.detections[i].class_id == msg.detections[i].ROUNDABOUT):
                self.velocity_multiplier = 0.0 # TODO
            elif(msg.detections[i].class_id == msg.detections[i].STOP):
                self.velocity_multiplier = 0.0
            elif(msg.detections[i].class_id == msg.detections[i].TURN_RIGHT):
                self.velocity_multiplier = 0.0 # TODO
            elif(msg.detections[i].class_id == msg.detections[i].TURN_LEFT):
                self.velocity_multiplier = 0.0 # TODO
            else:
                self.velocity_multiplier = 1.0
    
    def get_vel_multipler(self, msg):
        if(msg.detection_type == msg.GREEN_CIRCLE):
            self.velocity_multiplier = 1.0
        elif(msg.detection_type == msg.YELLOW_CIRCLE):
            self.velocity_multiplier = 0.5
        elif(msg.detection_type == msg.RED_CIRCLE):
            self.velocity_multiplier = 0.0
        else:
            self.velocity_multiplier = 0.0


    # Odometry callback
    def vel_callback(self):
        if self.finish_config:
            #self.getVelocity()
            self.msg_vel.linear.x *= self.velocity_multiplier
            self.msg_vel.angular.z *= self.velocity_multiplier
            self.vel_publisher.publish(self.msg_vel)
            

            # print(self.msg_vel, end='')
            # print(self.distance, end='')
            # print(self.theta, end='')
            # print(self.puntos[self.count], end='')
            # print(self.thetaT, end='')
            # print("Distance error: ", self.error_d)
            # print("Angle error: ", self.error_ang)
            # print("Local car distance: ",self.distance)
            # print("Target distance: ", self.distance_t)
            # print("Difference between local car distance and target distance: ", self.distance_t-self.distance)
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

    # Line follower callback
    def line_callback(self, msg):
        self.error_ang = msg.data
        
        #First, let robot turn until self.error_ang is diminutive
        if (abs(self.error_ang) >= 10.0):
            self.w = self.KwL * self.error_ang 
        else:
            self.w = 0.0

        self.v = self.maxlinear * 0.3
    
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

        print (self.v, self.w)

# Run node
def main(args=None):
    rclpy.init(args=args)
    ctr = Controller()
    rclpy.spin(ctr)
    ctr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    