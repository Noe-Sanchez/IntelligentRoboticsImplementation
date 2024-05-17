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

def cmd_vel_handle(twist):
	velocities = [0.0] * 6
	velocities[0] = twist.linear.x
	velocities[1] = twist.linear.y
	velocities[2] = twist.linear.z
	velocities[3] = twist.angular.x
	velocities[4] = twist.angular.y
	velocities[5] = twist.angular.z
	xarm.move_by_velocity(velocities)


if __name__ == "__main__":
	rospy.init_node('cartesian_server')
	xarm = xarm()
	xarm.set_arm()
	rospy.Subscriber('/goal_pose', Twist, cmd_vel_handle)
	xarm.move_to_goal([169.788757, 79.443832, 525.949951, -3.141593, 0.000000, 0.000000])
	rospy.spin()