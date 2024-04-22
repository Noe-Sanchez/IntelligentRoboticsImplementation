import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class ControlV(Node):
    def __init__(self):
        super().__init__('open_loop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.v_period = 0.1 
        self.velocity_timer = self.create_timer(self.v_period, self.velocity_callback)
        self.get_logger().info('Open loop node successfully initialized!!!')
        self.msg_v = Twist()
        self.variablesInit()
        self.puntos = np.array( [ [0,0], [1,1], [2,0]] )
        
        
        
    
    def variablesInit(self):
        #Twist message
        self.msg_v.linear.y = 0.0
        self.msg_v.linear.z = 0.0
        self.msg_v.angular.x = 0.0
        self.msg_v.angular.y = 0.0

        #Angle and robot distance between points
        self.angle = 0
        self.time = 0
        self.distance = 0

        #Flags when angVelocity and Linear Velocity end
        self.vfinish = True
        self.afinish = True

        #Flag when trajectory ends
        self.finish = False

        #How many points have been crossed
        self.count = 0
        self.maxang = 0.5
        self.maxlinear = 0.1

    
    def velocity_callback(self):
       
        if self.vfinish and self.afinish:
            self.distance = 0
            self.vfinish = False
            self.afinish = False
            if self.count < self.puntos.shape[0] - 1: 
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
            

        deltay = self.y2 - self.y1
        deltax = self.x2 - self.x1
        d = np.sqrt((deltax)**2 + (deltay)**2)
        calculatedAngle = np.arctan2(deltay, deltax)
        errorDer = 0

        if(self.angle > 2*np.pi): self.angle -= 2*np.pi
        elif(self.angle < 0): self.angle += 2*np.pi
    
        errorIzq = self.angle - calculatedAngle
        if(errorIzq < 0): errorDer = errorIzq + 2*np.pi
        elif(errorIzq > 0): errorDer = errorIzq - 2*np.pi

        if(abs(errorIzq) < abs(errorDer)): error = errorIzq
        else: error = errorDer

        if (error <= -0.1 or error >= 0.1):
            if(error > 0): self.msg_v.angular.z = -self.maxang
            elif(error < 0): self.msg_v.angular.z = self.maxang             
        else:
            self.msg_v.angular.z = 0.0
            self.afinish = True

        if self.afinish:
            if self.distance < d : self.msg_v.linear.x = self.maxlinear
            else: self.vfinish = True
        else:
            self.msg_v.linear.x = 0.0

        if self.finish:
            self.msg_v.linear.x = 0.0
            self.msg_v.angular.z = 0.0
        
        v = self.msg_v.linear.x
        w = self.msg_v.angular.z


        self.publisher.publish(self.msg_v)
        self.distance = self.distance + v * self.v_period
        self.angle = self.angle + w * self.v_period
        self.timeEstimated = d / self.maxlinear + calculatedAngle / self.maxang
        
        self.get_logger().info('Velocities: {}'.format(self.msg_v))
        self.get_logger().info('Distance: {}'.format(self.distance))
        self.get_logger().info('Angle: {}'.format(self.angle))
        self.get_logger().info('Punto: {}'.format(self.puntos[self.count - 1]))
        self.get_logger().info('Punto2: {}'.format(self.puntos[self.count]))
        self.get_logger().info('theta: {}'.format(calculatedAngle))
        self.get_logger().info('d: {}'.format(d))
        self.get_logger().info("Estimated time: {}".format(self.timeEstimated))

        self.time += self.v_period
        


def main(args=None):
    rclpy.init(args=args)
    m_s = ControlV()
    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    