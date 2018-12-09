#!/usr/bin/env python

import rospy
import sys
import numpy as np
import itertools

from intera_interface import gripper as robot_gripper
from intera_interface import Limb

from intera_interface.limb import Point, Quaternion

from moveit_commander import MoveGroupCommander

from moveit_msgs.msg import OrientationConstraint, RobotTrajectory, Constraints
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse

_start_joint_angles = {'right_j6': 0.062576171875, 'right_j5': 1.8633359375, 'right_j4': 0.9105380859375, 'right_j3': -0.165474609375, 'right_j2': -0.9086630859375, 'right_j1': -0.2302421875, 'right_j0': -0.1539775390625}

_start_pose = {'position': Point(x=0.83580435659716, y=0.14724066320869522, z=0.5248728070777701), 'orientation': Quaternion(x=0.7366681513777151, y=-0.6758459333695754, z=0.023326822897651703, w=0.002858046019378824)}


class SimpleController:
	def __init__(self):
		self._pixel_meter_ratio = 0.001
		self._threshold = 5
		self.harmonic_repeat = float(2)
		print('1')
		rospy.wait_for_service('compute_ik')
		print('2')
		rospy.init_node('service_query')
		print('3')
		#Create the function used to call the service
		self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
		print('4')
		self.right_gripper = robot_gripper.Gripper('right_gripper')
		print('5')
		self.limb = Limb()
		print('6')

	def move_to(self, pos_x, pos_y, pos_z, orien_x, orien_y, orien_z, orien_w, ik, orien_const = None):
		#Construct the request
		request = GetPositionIKRequest()
		request.ik_request.group_name = "right_arm"
		request.ik_request.ik_link_name = "right_gripper"
		request.ik_request.attempts = 20
		request.ik_request.pose_stamped.header.frame_id = "base"

		#Set the desired orientation for the end effector HERE
		request.ik_request.pose_stamped.pose.position.x = pos_x
		request.ik_request.pose_stamped.pose.position.y = pos_y
		request.ik_request.pose_stamped.pose.position.z = pos_z        
		request.ik_request.pose_stamped.pose.orientation.x = orien_x
		request.ik_request.pose_stamped.pose.orientation.y = orien_y
		request.ik_request.pose_stamped.pose.orientation.z = orien_z
		request.ik_request.pose_stamped.pose.orientation.w = orien_w


		try:
			#Send the request to the service
			response = ik(request)
			
			#Print the response HERE
			print(response)
			group = MoveGroupCommander("right_arm")

			if orien_const is not None:
				constraints = Constraints()
				constraints.orientation_constraints = orien_const
				group.set_path_constraints(constraints)

			# Setting position and orientation target
			group.set_pose_target(request.ik_request.pose_stamped)

			# TRY THIS
			# Setting just the position without specifying the orientation
			# group.set_position_target([0.5, 0.5, 0.0])

			# Plan IK and execute
			group.go()
			
		except rospy.ServiceException, e:
			print("Service call failed: %s")

if __name__ == '__main__':
	ctrler = SimpleController()

	ctrler.move_to(_start_pose["position"].x, _start_pose["position"].y, _start_pose["position"].z,
				   _start_pose["orientation"].x, _start_pose["orientation"].y, _start_pose["orientation"].z, _start_pose["orientation"].w,
				   ctrler.compute_ik)
