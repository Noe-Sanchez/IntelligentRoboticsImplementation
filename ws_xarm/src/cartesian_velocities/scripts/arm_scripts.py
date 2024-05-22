#!/usr/bin/env python3
# license removed for brevity
import rospy
import time
from std_msgs.msg import *
from geometry_msgs.msg import Twist
from xarm_msgs.srv import *
import copy
import math as m

class xarm:

	def initialize_xarm(self):
		self.motion = 0
		self.mode = 0
		self.state = 0
		self.current_position = [0.0] * 6
		self.goal_position = [0.0] * 6
		

	def set_arm(self):
		rospy.wait_for_service('xarm/set_mode')
		rospy.wait_for_service('xarm/set_state')
		try:
			set_mode = rospy.ServiceProxy('xarm/set_mode', SetInt16)
			set_state = rospy.ServiceProxy('xarm/set_state', SetInt16)
			set_motion_control = rospy.ServiceProxy('xarm/motion_ctrl', SetAxis)
			set_motion_control(8,1)
			set_mode(5)
			set_state(0)
			self.mode = 5
			self.state = 0
			self.motion = 1

		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

	def move_by_velocity_timed(self,velocities):
		rospy.wait_for_service('xarm/velo_move_line_timed')
		try:
			move_line = rospy.ServiceProxy('xarm/velo_move_line_timed', MoveVelocity)
			req = MoveVelocityRequest()
			req.speeds = velocities
			req.duration = 1.0
			req.is_sync = False
			req.is_tool_coord = False
			move_line(req)
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

	def move_by_velocity(self,velocities):
		rospy.wait_for_service('xarm/velo_move_line')
		try:
			move_line = rospy.ServiceProxy('xarm/velo_move_line', MoveVelo)
			req = MoveVeloRequest()
			# for i in range(2):
			# 	if velocities[i] > 20:
			# 		velocities[i] = 20
			# 	elif velocities[i] < -20:
			# 		velocities[i] = -20
			vel_ = velocities
			vel_[3] = 0.0
			vel_[4] = 0.0
			vel_[5] = 0.0
			req.velocities = velocities
			req.jnt_sync = 0
			req.coord = 0
			if(self.check_arm_limits()):
				move_line(req)
			else:
				self.stop_arm()
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

	def stop_arm(self):
		rospy.wait_for_service('xarm/velo_move_line')
		try:
			move_line = rospy.ServiceProxy('xarm/velo_move_line', MoveVelo)
			req = MoveVeloRequest()
			req.velocities = [0.0] * 6
			req.jnt_sync = 0
			req.coord = 0
			move_line(req)
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

	def get_current_position(self):
		get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
		actual_pose = list(get_position().datas)
		return actual_pose

	def get_difference_between_poses(self,pose1, pose2):
		diff = [pose2[i] - pose1[i] for i in range(6)]
		return diff
	
	def calculate_distance(self, pose_1, pose_2):
		diff = self.get_difference_between_poses(pose_1, pose_2)
		distance = sum([abs(v) for v in diff])
		return distance

	def move_to_goal(self,goal_pose):
		current_pose = self.get_current_position()
		diff = self.get_difference_between_poses(current_pose, goal_pose)
		calculated_distance = self.calculate_distance(current_pose, goal_pose)
		while(calculated_distance > 0.1):
			self.move_by_velocity(diff)
			current_pose = self.get_current_position()
			diff = self.get_difference_between_poses(current_pose, goal_pose)
			calculated_distance = self.calculate_distance(current_pose, goal_pose)
			time.sleep(0.1)
		self.stop_arm()

	def check_arm_limits(self):
		current_position = self.get_current_position()
		distancie_from_the_center_in_x_y = m.sqrt(sum([v**2 for v in current_position[:2]]))
		print(distancie_from_the_center_in_x_y)
		if distancie_from_the_center_in_x_y < 170:
			print("The arm is out of the limits")
			self.stop_arm()
			return False
		if distancie_from_the_center_in_x_y > 500:
			print("The arm is out of the limits")
			self.stop_arm()
			return False
		if current_position[2] < 0:
			print("The arm is out of the limits")
			self.stop_arm()
			return False
		if current_position[2] > 600:
			print("The arm is out of the limits")
			self.stop_arm()
			return False
		else:
			print("The arm is within the limits")
			return True
