#!/usr/bin/env python3

from __future__ import print_function
from xarm_msgs.srv import *
import rospkg
import importlib
import sys
import time
import math as m
import copy
import rospy
from arm_scripts import xarm
from geometry_msgs.msg import Twist, Point
import threading

def cmd_vel_handle(twist):
    gain = 0.9
    multiplier = 1 * gain
    velocities = [0.0] * 6
    velocities[0] = twist.linear.x*multiplier
    velocities[1] = twist.linear.y*multiplier
    velocities[2] = twist.linear.z*multiplier
    #velocities[2] = 0.0
    velocities[3] = twist.angular.x*multiplier
    velocities[4] = twist.angular.y*multiplier
    velocities[5] = twist.angular.z*multiplier
    print(velocities)
    xarm.move_by_velocity(velocities)

def pose_handle(msg):
    print(msg.x*10,msg.y*10,msg.z*10)
    x = (msg.x*10.0)
    y = (msg.y*10.0)
    z = (msg.z*10.0)
    print(x,y,z)
    xarm.move_to_goal([x,y,z,0,0,0])

def check_arm_limits_continuously():
    while not rospy.is_shutdown():
        xarm.check_arm_limits()
        time.sleep(0.1)  # Adjust the sleep time as needed

if __name__ == "__main__":
    rospy.init_node('cartesian_server')
    xarm = xarm()
    xarm.set_arm()
    # Start a new thread to continuously check arm limits
    arm_limits_thread = threading.Thread(target=check_arm_limits_continuously)
    arm_limits_thread.daemon = True
    arm_limits_thread.start()
    rospy.Subscriber('/hand_velocity', Twist, cmd_vel_handle)
    print("Ready to move the arm")
    
    rospy.spin()
