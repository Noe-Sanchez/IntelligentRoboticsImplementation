import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class ControlV(Node):
    def __init__(self):
        super().__init__('open_loop_node')
        self.msg_v = Twist()
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.variablesInit()
        self.getParameters = self.create_subscription(PoseArray, 'pose', self.getParams_callback, 10)
        self.v_period = 0.05
        self.velocity_timer = self.create_timer(self.v_period, self.velocity_callback)
        self.get_logger().info('Open loop node successfully initialized!!!')
        
        
    
    def variablesInit(self):
        #Set variables

        #Twist message
        self.msg_v.linear.y = 0.0
        self.msg_v.linear.z = 0.0
        self.msg_v.angular.x = 0.0
        self.msg_v.angular.y = 0.0

        #Angle and robot distance between points
        self.angle = 0
        self.time = 0
        self.distance = 0
        self.maxerror = 0

        #Flags when angVelocity and Linear Velocity end
        self.vfinish = True
        self.afinish = True

        #Flag when trajectory ends
        self.finish = False

        #How many points have been crossed
        self.count = 0

        #Max velocities
        self.maxang = 1.25
        self.maxlinear = 0.3

        #Min velocities
        self.minlinear = 0.05
        self.minang = 0.4

        #Car velocities
        self.linearv = 0
        self.angw = 0
        self.finishConfig = False
        self.puntos =[]


    def getParams_callback(self, msg):
        self.get_logger().info("Mensaje recibido: {}".format(msg))
        if self.finishConfig == False:
            for pose in msg.poses:
                self.punto = []
                self.linearv = pose.orientation.x
                self.angw = pose.orientation.y

                #If linear or angular velocities are below the minimum or above the maximum, use average between maximum and minimum velocities
                if self.linearv > self.maxlinear or self.linearv < self.minlinear: self.linearv = (self.minlinear + self.maxlinear)/2
                if self.angw > self.maxang or self.angw < self.minang: self.angw = (self.maxang + self.minang)/2
                self.punto.append(pose.position.x)
                self.punto.append(pose.position.y)
                #Update points
                self.puntos.append(self.punto)


            self.finishConfig = True

    
    def velocity_callback(self):
        #After configuring path and velocities, do this
        if self.finishConfig:

            #After one part of the trajectory is ready, update points
            if self.vfinish and self.afinish:
                self.distance = 0
                self.vfinish = False
                self.afinish = False
                if self.count < len(self.puntos) - 1: 
                    self.x1 = self.puntos[self.count][0]
                    self.y1 = self.puntos[self.count][1]
                    self.x2 = self.puntos[self.count+1][0]
                    self.y2 = self.puntos[self.count+1][1]
                    self.count +=1
                else:
                    self.x1 = self.puntos[-1][0]
                    self.y1 = self.puntos[-1][1]
                    self.count = 0
                    self.x2 = self.puntos[self.count][0]
                    self.y2 = self.puntos[self.count][1]
                    #self.finish = True
                
            #Obtain distance and angle between points
            deltay = self.y2 - self.y1
            deltax = self.x2 - self.x1
            d = np.sqrt((deltax)**2 + (deltay)**2)
            calculatedAngle = np.arctan2(deltay, deltax)
            errorDer = 0

            #Narrow robot angle between [0, 2pi]
            if(self.angle > 2*np.pi): self.angle -= 2*np.pi
            elif(self.angle < 0): self.angle += 2*np.pi
        
            #Obtain angular error and its complementary error, and see which is the lowest
            errorIzq = self.angle - calculatedAngle
            if(errorIzq < 0): errorDer = errorIzq + 2*np.pi
            elif(errorIzq > 0): errorDer = errorIzq - 2*np.pi

            if(abs(errorIzq) < abs(errorDer)): error = errorIzq
            else: error = errorDer

            #Estimate maximum angular error and time between points
            if(abs(error) > self.maxerror): self.maxerror = abs(error)
            self.timeEstimated = d / self.linearv + self.maxerror / self.angw

            #First, let robot turn until error is diminutive
            if (error <= -0.05 or error >= 0.05):
                if(error > 0): self.msg_v.angular.z = -self.angw
                elif(error < 0): self.msg_v.angular.z = self.angw             
            else:
                self.msg_v.angular.z = 0.0
                self.afinish = True

            #If robot turn is finish, let robot have linear velocity
            if self.afinish:
                if self.distance < d : self.msg_v.linear.x = self.linearv
                else: self.vfinish = True
            else:
                self.msg_v.linear.x = 0.0

            if self.finish:
                self.msg_v.linear.x = 0.0
                self.msg_v.angular.z = 0.0
            
            v = self.msg_v.linear.x
            w = self.msg_v.angular.z

            #Publish velocities
            self.publisher.publish(self.msg_v)

            #Obtain actual robot distance and angle of the robot
            self.distance = self.distance + v * self.v_period
            self.angle = self.angle + w * self.v_period
            
            
            self.get_logger().info('Velocities: {}'.format(self.msg_v))
            self.get_logger().info('Distance: {}'.format(self.distance))
            self.get_logger().info('Angle: {}'.format(self.angle))
            self.get_logger().info('Punto: {}'.format(self.puntos[self.count - 1]))
            self.get_logger().info('Punto2: {}'.format(self.puntos[self.count]))
            self.get_logger().info('theta: {}'.format(calculatedAngle))
            self.get_logger().info('d: {}'.format(d))
            self.get_logger().info("Estimated time: {}".format(self.timeEstimated))
        


def main(args=None):
    rclpy.init(args=args)
    m_s = ControlV()
    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    