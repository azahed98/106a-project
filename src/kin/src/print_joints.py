#!/usr/bin/env python

import rospy

import intera_interface

import intera_external_devices
from intera_interface import CHECK_VERSION

from intera_interface import Limb


rospy.init_node("sdk_joint_position_keyboard")
limb = intera_interface.Limb("right")
rs = intera_interface.RobotEnable(CHECK_VERSION)
rs.enable()
print('Initial angles')
print(limb.joint_angles())

print(limb.endpoint_pose())


# controller.move_to(_start_pose.position.x, _start_pose.position.y, _start_pose.position.z,
# 					_start_pose.orientation.x, _start_pose.orientation.y, _start_pose.orientation.z, _start_pose.orientation.w)
# for key, value in _start_joint_angles.iteritems():
# 	limb.set_joint_positions({key: value})
# 	rospy.sleep(2.0)

# print(limb.joint_angles())
# rospy.sleep(5.0)
