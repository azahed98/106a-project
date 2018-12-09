#!/usr/bin/env python

import rospy
import sys
import numpy as np
import itertools

from intera_interface import gripper as robot_gripper
from intera_interface import Limb

from intera_interface.limb import Point, Quaternion

from moveit_commander import MoveGroupCommander

from moveit_msgs.msg import OrientationConstraint, RobotTrajectory, Constraints, CollisionObject
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

from kinect_interface import get_grip_target, get_center, get_angle, process_depth_map, aquire_depth_map


class Controller:
	def __init__(self):
		self._start_pose = {'position': Point(x=0.83580435659716, y=0.14724066320869522, z=0.4548728070777701), 'orientation': Quaternion(x=0.7366681513777151, y=-0.6758459333695754, z=0.023326822897651703, w=0.002858046019378824)}

		self._pixel_meter_ratio = 0.001
		self._threshold = 4
		self.harmonic_repeat = float(2)
		rospy.wait_for_service('compute_ik')
		rospy.init_node('service_query')

		#Create the function used to call the service
		self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
		self.right_gripper = robot_gripper.Gripper('right_gripper')
		self.limb = Limb()
		self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

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

	def add_box_obstacle(self, size, name, pose):
		"""
		Adds a rectangular prism obstacle to the planning scene

		Inputs:
		size: 3x' ndarray; (x, y, z) size of the box (in the box's body frame)
		name: unique name of the obstacle (used for adding and removing)
		pose: geometry_msgs/PoseStamped object for the CoM of the box in relation to some frame
		"""    

		# Create a CollisionObject, which will be added to the planning scene
		co = CollisionObject()
		co.operation = CollisionObject.ADD
		co.id = name
		co.header = pose.header

		# Create a box primitive, which will be inside the CollisionObject
		box = SolidPrimitive()
		box.type = SolidPrimitive.BOX
		box.dimensions = size

		# Fill the collision object with primitive(s)
		co.primitives = [box]
		co.primitive_poses = [pose.pose]

		# Publish the object
		self._planning_scene_publisher.publish(co)

	def remove_obstacle(self, name):
		"""
		Removes an obstacle from the planning scene

		Inputs:
		name: unique name of the obstacle
		"""

		co = CollisionObject()
		co.operation = CollisionObject.REMOVE
		co.id = name

		self._planning_scene_publisher.publish(co)

	def pixels_to_meters(self, pixel):
		return pixel * self._pixel_meter_ratio

	def quaternion_multiply(self, quaternion1, quaternion0):
		w0, x0, y0, z0 = quaternion0
		w1, x1, y1, z1 = quaternion1
		return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
						 x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
						 -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
						 x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

	def add_collision_box(self):
		size = (1.0, 2.0, .1)
		pose = PoseStamped()
		pose.pose.position.x = 0.75
		pose.pose.position.y = 0.0
		pose.pose.position.z = -0.17
		pose.pose.orientation.x = 0.0
		pose.pose.orientation.y = 0.0
		pose.pose.orientation.z = 0.0
		pose.pose.orientation.w = 1.0
		pose.header.frame_id = "base"
		name = 'prevent-collision-box'
		self.add_box_obstacle(size, name, pose)

	def move_ontop_target(self):
		dy, dx = get_grip_target(plot=False)

		dxm = self.pixels_to_meters(dx)
		dym = self.pixels_to_meters(dy)

		# kinect_sensor_diff_y = self.pixels_to_meters(170)
		# kinect_sensor_diff_x = self.pixels_to_meters(350)
		kinect_sensor_diff_y = 0.075 + 0.03
		kinect_sensor_diff_x = 0.12 - 0.1025
		# kinect_sensor_diff_x = 0.12
		dym = dym + kinect_sensor_diff_y
		dxm = dxm + kinect_sensor_diff_x

		inp = None
		iter_num = 1
		while dxm**2 + dym**2 > self.pixels_to_meters(self._threshold)**2 and inp is not 'q':

			cur_pose = self.limb.endpoint_pose()
			gripper_to_endpoint = 0.135
			cur_pos = cur_pose['position']
			cur_orien = cur_pose['orientation']
			move_delta_max = 0.1
			
			# decay_weight = (1.0/np.ceil(iter_num/self.harmonic_repeat))
			decay_weight = 1.0
			delta_x = decay_weight * (np.sign(dxm)*min(abs(dxm), move_delta_max))
			delta_y = -1 * decay_weight * np.sign(dym)* min(abs(dym), move_delta_max)
			self.move_to(cur_pos.x + delta_x, \
		 				 cur_pos.y + delta_y, \
		 				 cur_pos.z - gripper_to_endpoint, \
		 				 cur_orien.x, cur_orien.y, cur_orien.z, cur_orien.w, \
						 self.compute_ik)

			# self.move_to(cur_pos.x + (np.sign(dxm)*min(abs(dxm), move_delta_max)), \
		 # 				 cur_pos.y, \
		 # 				 cur_pos.z - gripper_to_endpoint, \
		 # 				 cur_orien.x, cur_orien.y, cur_orien.z, cur_orien.w, \
			# 			 self.compute_ik)
			
			rospy.sleep(0.5)
			

			found_box = False
			while not found_box:
				try:
					dy, dx = get_grip_target(plot=False)
					dxm = self.pixels_to_meters(dx)
					dym = self.pixels_to_meters(dy)

					dym = dym + kinect_sensor_diff_y 
					dxm = dxm + kinect_sensor_diff_x

					found_box = True
				except ValueError, e:
					print("BOX NOT FOUND, CHANGING Z")
					self.move_to(ctrler._start_pose["position"].x, ctrler._start_pose["position"].y,ctrler. _start_pose["position"].z,
							     ctrler._start_pose["orientation"].x, ctrler._start_pose["orientation"].y, ctrler._start_pose["orientation"].z, ctrler._start_pose["orientation"].w,
							     ctrler.compute_ik)
					self.right_gripper.open()
			
			if iter_num >= 30:
				inp = raw_input('type [ q ] to quit: ')
			iter_num += 1

		print("HIT THRESHOLD")

		rospy.sleep(1.0)
		# maybe average rot from trials above to get final rot
		
		# FOR TESTING --> UNCOMMENT OTHERWISE
		# rot = np.pi/6
		depth_map = process_depth_map(aquire_depth_map(), plot=False)
		centerx, centery = get_center(depth_map, plot=False)
		rot = -1.0 * get_angle(depth_map, np.array([centerx, centery]), plot=False)
		print("ROTATION: ", rot)
		# rot = rot * 2.5

		cur_pose = self.limb.endpoint_pose()
		cur_pos = cur_pose['position']
		cur_orien = cur_pose['orientation']
		quat_x, quat_y, quat_z, quat_w = 0.0, 0.0, np.sin(rot/2), np.cos(rot/2)
		quat_1 = (quat_w, quat_x, quat_y, quat_z)
		quat_2 = (cur_orien.w, cur_orien.x, cur_orien.y, cur_orien.z)
		quat_final = self.quaternion_multiply(quat_1, quat_2)
		final_w, final_x, final_y, final_z = quat_final

		gripper_to_endpoint = 0.15

		self.move_to(cur_pos.x, \
					 cur_pos.y, \
					 cur_pos.z - gripper_to_endpoint, \
					 final_x, final_y, final_z, final_w, \
					 self.compute_ik)
		
		print("ROTATED TO BOX ROTATION")

		rospy.sleep(.5)

		cur_pose = self.limb.endpoint_pose()
		cur_pos = cur_pose['position']
		cur_orien = cur_pose['orientation']

		gripper_box_height_diff = 0.87
		percent_no_constr = .7
		first_drop = gripper_box_height_diff * percent_no_constr
		second_drop = gripper_box_height_diff * (1.0-percent_no_constr)

		self.move_to(cur_pos.x, \
					 cur_pos.y, \
					 cur_pos.z - first_drop, \
					 cur_orien.x, cur_orien.y, cur_orien.z, cur_orien.w, \
					 self.compute_ik)

		rospy.sleep(1.0)

		self.remove_obstacle('prevent-collision-box')

		cur_pose = self.limb.endpoint_pose()
		cur_pos = cur_pose['position']
		cur_orien = cur_pose['orientation']


		orien_const = OrientationConstraint()
		orien_const.link_name = "right_gripper"
		orien_const.header.frame_id = "base"
		orien_const.orientation.x = cur_orien.x
		orien_const.orientation.y = cur_orien.y
		orien_const.orientation.z = cur_orien.z
		orien_const.absolute_x_axis_tolerance = 0.1
		orien_const.absolute_y_axis_tolerance = 0.1
		orien_const.absolute_z_axis_tolerance = 0.1
		orien_const.weight = 1.0

		self.move_to(cur_pos.x, \
					 cur_pos.y, \
					 cur_pos.z - second_drop, \
					 cur_orien.x, cur_orien.y, cur_orien.z, cur_orien.w, \
					 self.compute_ik,
					 [orien_const])

		self.right_gripper.close()
		rospy.sleep(3.0)

		self.move_to(ctrler._start_pose["position"].x, ctrler._start_pose["position"].y,ctrler. _start_pose["position"].z,
				   ctrler._start_pose["orientation"].x, ctrler._start_pose["orientation"].y, ctrler._start_pose["orientation"].z, ctrler._start_pose["orientation"].w,
				   ctrler.compute_ik)

		rospy.sleep(3.0)

		self.right_gripper.open()


if __name__ == '__main__':
	ctrler = Controller()
	ctrler.add_collision_box()
	print(ctrler.limb.endpoint_pose()['position'])
	ctrler.move_to(ctrler._start_pose["position"].x, ctrler._start_pose["position"].y,ctrler. _start_pose["position"].z,
				   ctrler._start_pose["orientation"].x, ctrler._start_pose["orientation"].y, ctrler._start_pose["orientation"].z, ctrler._start_pose["orientation"].w,
				   ctrler.compute_ik)
	ctrler.right_gripper.calibrate()
	ctrler.right_gripper.open()
	rospy.sleep(1.0)
	ctrler.move_ontop_target()
	print(get_grip_target(plot=False)[0:3])
