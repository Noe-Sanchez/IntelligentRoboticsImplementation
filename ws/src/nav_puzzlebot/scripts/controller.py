#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Pose2D, Point, PoseArray, Pose
from std_msgs.msg import Int16, Float32
import numpy as np
import math
from car_interfaces.msg import Detection, Inference, InferenceArray
from enum import IntEnum
from std_srvs.srv import Empty
import time
import copy

USING_POSE = True
USING_LINE = True
USING_DETECTIONS = True

class States(IntEnum):
    IDLE = 0
    INFERENCE = 1
    FOLLOW_LINE = 2
    STOP = 3

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

        # Inference variables
        self.inference_list = []
        self.inference = Inference()
        self.inference_timeout = 3
        self.last_inference_time = 0

        # Detection variables
        self.detection = Detection()
        self.detection.detection_type = self.detection.NO_DETECTION

        # Intersection id
        self.intersection_id = 999.0

        # Robot constants
        self.Kv = 0.25 # proportional linear velocity constant
        self.Kw = 0.25 # proportional angular velocity constant
        self.KwL =  0.00125 # proportional angular velocity constant
        self.KwLT = 0.0055 # proportional angular velocity constant
        self.KdwL = 0.0

        # Velocity publisherself.msg_vel = Twist() # velocity message
        self.vel_period = 0.01 # velocity publishing period (seconds)
        self.msg_vel = Twist() # velocity message
        # Twist message
        self.msg_vel.linear.x = 0.0
        self.msg_vel.linear.y = 0.0
        self.msg_vel.linear.z = 0.0
        self.msg_vel.angular.x = 0.0
        self.msg_vel.angular.y = 0.0
        self.msg_vel.angular.z = 0.0

        # Line follower variables
        self.line_v = 0.0
        self.line_w = 0.0
        self.intersection_flag = False
        
        # Robot pose variables
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.odom_flag = False

        # Flags when angVelocity and Linear Velocity end
        self.vfinish = True
        self.afinish = True
        
        self.xT = 0
        self.yT = 0

        # Robot distance between points
        self.distance = 0
        self.distance_t = 0

        # Points
        self.puntos = []   
        self.TARGET_TOLERANCE = 0.02  

        
        self.forward_points = [(0.30, 0.0)]
        self.turn_left_points = [(0.25, 0.0), (0.25, 0.1)]
        self.turn_right_points = [(0.27, 0.0), (0.27, -0.1)]
        self.roundabout_points = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]

        # Max & min velocities
        self.maxang = 1
        self.maxlinear = 0.3
        self.minlinear = 0.05
        self.minang = 0.4
        self.velocity_multiplier = 1.0
        self.lasterror_ang = 0.0


        # Robot state
        self.robot_state = States.IDLE

        # Angular velocity subscribers
        if USING_POSE:
            self.odom_subscriber = self.create_subscription(Pose2D, 'odom', self.odom_callback, qos_profile=qos_profile_sub) # odometry topic subscriber
            self.odom_reset_client = self.create_client(Empty, 'reset_odom')
        
        if USING_LINE:
            self.line_subscriber = self.create_subscription(Float32, '/roadError', self.line_callback, 1)
        
        if USING_DETECTIONS:
            self.circle_detection_timeout=0.5
            self.last_circle_detection_time = 0
            self.detection_subscriber = self.create_subscription(InferenceArray, '/carInferences', self.getInferences, 10)
            self.velocity_multiplier_suscriber = self.create_subscription(Detection, '/carDetections', self.getDetections, 10)
        
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10) # velocity topic publisher

        self.vel_timer = self.create_timer(self.vel_period, self.state_machine) # velocity publishing timer
        
        self.get_logger().info('Controller node successfully initialized!!!')


    # Update left motor angular velocity
    def odom_callback(self, msg):
        self.odom_flag = True
        self.theta = msg.theta
        self.x = msg.x
        self.y = msg.y
        #self.get_logger().info('Odom: {}'.format(msg))
    
    # Line follower callback
    def line_callback(self, msg):
        self.error_ang_line = msg.data
        
        if self.error_ang_line == self.intersection_id:
            self.line_v = 0.0
            self.line_w = 0.0
            self.intersection_flag = True
            return

        #First, let robot turn until self.error_ang_line is diminutive
        if (abs(self.error_ang_line) >= 70.0):
            w = self.KwLT * self.error_ang_line # + self.KdwL * (self.error_ang_line - self.lasterror_ang) * self.vel_period 
            print('TURN')
        elif(abs(self.error_ang_line) >= 20.0):
             w = self.KwL * self.error_ang_line
             print('FIX') 
        else:
            print("STRAIGHT")
            w = 0.0

        v = self.maxlinear * 0.3 * (1 - abs(self.error_ang_line) / 90.0)
        self.lasterror_ang = self.error_ang_line
        if abs(v) > self.maxlinear:
            v = (abs(v) / v) * self.maxlinear
        elif 0.0 < abs(v) and v < self.minlinear:
            v = (abs(v) / v) * self.minlinear

        if abs(w) > self.maxang:
            w = (abs(w) / w) * self.maxang
        #elif 0.0 < abs(w) and w < self.minang:
        #    w = (abs(w) / w) * self.minang
        
        print(w)
        print(self.error_ang_line)

        self.line_v = v
        self.line_w = w
    

    def getVelocity(self):
        # After one part of the trajectory is ready, update points
        if self.vfinish and self.afinish:
            self.distance = 0
            self.vfinish = False
            self.afinish = False

            if len(self.puntos) > 0:
                print("---------------------- POPPING POINTS ----------------------------")
                self.xT, self.yT = self.puntos.pop(0)
                self.error_d = 0
                self.thetaT = 0
            else:
                self.xT, self.yT, self.error_d, self.thetaT, self.error_ang = 0, 0, 0, 0
                return 0.0, 0.0

            # print(self.xT, se lf.yT)
            # print(self.x, sel f.y)
        
        #print(self.xT, self.yT)
        #print(self.x, self.y)
            
        # Obtain distance and angle between points
        print(f"Deltas: y: {self.yT} - {self.y} x: {self.xT} - {self.x}")
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

        print(f"Waypoint error ang: {self.error_ang}")
        #First, let robot turn until self.error_ang is diminutive
        if (abs(self.error_ang) >= 0.15):
            w = self.Kw * self.error_ang
        else:
            w = 0.0
            self.afinish = True

        #If robot turn is finish, let robot have linear velocity
        if self.afinish:
            if self.error_d > 0.01 : 
                v = self.Kv * self.error_d
            else: 
                self.vfinish = True
                print("Reached")
                v = 0.0
                
        else:
            v = 0.0
    
        if abs(v) > self.maxlinear:
            v = (abs(v) / v) * self.maxlinear
        elif 0.0 < abs(v) and v < self.minlinear:
            v = (abs(v) / v) * self.minlinear

        if abs(w) > self.maxang:
            w = (abs(w) / w) * self.maxang
        elif 0.0 < abs(w) and w < self.minang:
            w = (abs(w) / w) * self.minang

        return v, w
       
    # Inferences callback
    def getInferences(self, msg):
        # Iterate through InferenceArray msg
        self.inference_list = []
        for i in range(len(msg.detections)):
            #self.get_logger().info('Inference: {}'.format(msg.detections[i].class_id))
            if(msg.detections[i].class_id not in [msg.detections[i].FORWARD, msg.detections[i].TURN_LEFT, msg.detections[i].TURN_RIGHT, msg.detections[i].ROUNDABOUT, msg.detections[i].STOP]):
                self.inference_list.append(msg.detections[i])
            else:
                if (self.intersection_flag):
                    self.robot_state = States.INFERENCE
                self.last_inference_time = time.time()
                self.inference = msg.detections[i]
                break

    #Velocity multiplier callback
    def get_vel_multipler(self):
        #self.get_logger().info('Inference length: {}'.format(len(self.inference_list)))
        if len(self.inference_list) > 0:
            while len(self.inference_list) > 0:
                inference = self.inference_list.pop()
                #self.get_logger().info('Inference: {}'.format(inference.class_id))
                if(inference.class_id == inference.GIVEAWAY):
                    self.velocity_multiplier = 0.2
                elif(inference.class_id == inference.ROADWORK):
                    self.velocity_multiplier = 0.2
        else: 
            self.velocity_multiplier = 1.0
        

        if(self.detection.detection_type == self.detection.GREEN_CIRCLE):
            self.velocity_multiplier = 1.0
        elif(self.detection.detection_type == self.detection.YELLOW_CIRCLE):
            self.velocity_multiplier = 0.5
        elif(self.detection.detection_type == self.detection.RED_CIRCLE):
            self.velocity_multiplier = 0.0
    
    def getDetections(self, msg):
        if (time.time() - self.last_circle_detection_time ) < self.circle_detection_timeout:
            return
        self.last_circle_detection_time = time.time()
        if msg.detection_type != msg.NO_DETECTION:
            print()
            print("-"*20)
            print("DETECTED", msg)
            print("-"*20)
            print()
        self.detection = msg

    # Odometry callback
    def state_machine(self):
        
        # delete inference if time has passed
        if (time.time()-self.last_inference_time > self.inference_timeout):
            # print(">>>>>>>>RESET INFERENCE")
            self.inference = Inference()
        
        # self.get_logger().info('State: {}'.format(self.robot_state))
        if self.robot_state == States.IDLE:
            self.msg_vel.linear.x = 0.0
            self.msg_vel.angular.z = 0.0
            self.robot_state = States.FOLLOW_LINE
        elif self.robot_state == States.FOLLOW_LINE:
            if USING_LINE:
                
                self.msg_vel.linear.x = self.line_v
                self.msg_vel.angular.z = self.line_w

                if self.intersection_flag:
                    self.msg_vel.linear.x = self.maxlinear * 0.2
            else:
                self.msg_vel.linear.x = self.maxlinear * 0.6
                self.msg_vel.angular.z = 0.0
        elif self.robot_state == States.INFERENCE:
            if not self.odom_flag: 
                return 

            if len(self.puntos) == 0:
                print(self.inference.class_id)
                self.vfinish = True
                self.afinish = True
                self.yT = 0
                self.xT = 0
                if self.inference.class_id == self.inference.FORWARD:
                    self.puntos = copy.deepcopy(self.forward_points)
                elif self.inference.class_id == self.inference.TURN_RIGHT:
                    print("RIGHT")
                    self.puntos = copy.deepcopy(self.turn_left_points)
                elif self.inference.class_id == self.inference.TURN_LEFT:
                    print("LEFT")
                    self.puntos = copy.deepcopy(self.turn_right_points)
                elif self.inference.class_id == self.inference.ROUNDABOUT:
                    self.puntos = copy.deepcopy(self.turn_right_points)
                elif self.inference.class_id == self.inference.STOP:
                    self.robot_state = States.STOP
                    return
                else:
                    return
                
                
                self.reset_odometry()
                
                for i in range(len(self.puntos)):
                    self.puntos[i] = ((self.puntos[i][0] * math.cos(self.theta) - self.puntos[i][1] * math.sin(self.theta)) + self.x, (self.puntos[i][0] * math.sin(self.theta) + self.puntos[i][1] * math.cos(self.theta)) + self.y)

                self.puntos.append(self.puntos[-1])

                print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                print(self.x, self.y, self.theta)

                print(self.puntos)


            self.msg_vel.linear.x, self.msg_vel.angular.z = self.getVelocity()  
              
            if (len(self.puntos) == 0):
                self.robot_state = States.FOLLOW_LINE
                self.intersection_flag = False
        elif self.robot_state == States.STOP:
            self.msg_vel.linear.x = 0.0
            self.msg_vel.angular.z = 0.0
            print("STOP")


        self.get_vel_multipler()
        
        self.msg_vel.linear.x *= self.velocity_multiplier
        self.msg_vel.angular.z *= self.velocity_multiplier
        if self.msg_vel.linear.x > self.maxlinear:
            self.msg_vel.linear.x = self.maxlinear
        self.vel_publisher.publish(self.msg_vel)
        
    def reset_odometry(self):
        print()
        print("-"*20)
        print("RESETTING ODOMETRY")
        print("-"*20)
        print()
        self.reset_odom_response = self.odom_reset_client.call_async(Empty.Request())
        self.stop(1.0)
        print("ODOMETRY RESET")
        # while not reset_odom_response.done():
        #     print("WAITING FOR RESET ODOM")
        #     pass
        
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0
    
    def stop(self, stop_time):
        stop_msg_vel = Twist()
        stop_msg_vel.linear.x = 0.0
        stop_msg_vel.angular.z = 0.0
        self.vel_publisher.publish(stop_msg_vel)
        time.sleep(stop_time)
        
        

# Run node
def main(args=None):
    rclpy.init(args=args)
    ctr = Controller()
    rclpy.spin(ctr)
    ctr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
