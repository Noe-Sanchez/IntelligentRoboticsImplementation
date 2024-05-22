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
from geometry_msgs.msg import Twist
import threading

def cmd_vel_handle(twist):
    velocities = [0.0] * 6
    velocities[0] = twist.linear.x
    velocities[1] = twist.linear.y
    velocities[2] = twist.linear.z
    velocities[3] = twist.angular.x
    velocities[4] = twist.angular.y
    velocities[5] = twist.angular.z
    xarm.move_by_velocity(velocities)

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

    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_handle)
    print("Ready to move the arm")
    
    rospy.spin()
