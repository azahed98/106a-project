#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from intera_interface import gripper as robot_gripper
import numpy as np
from numpy import linalg

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    right_gripper = robot_gripper.Gripper('right_gripper')

    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        print('Calibrating gripper...')
        right_gripper.calibrate()
        rospy.sleep(2.0)
        
        move_to(pos_x=0.676, pos_y=0.120, pos_z=0.146, \
         orien_x=0, orien_y=1, orien_z=0, orien_w=0, ik=compute_ik)

        right_gripper.close()
        rospy.sleep(1.0)

        move_to(pos_x=0.676, pos_y=0.120, pos_z=-0.146, \
         orien_x=0, orien_y=1, orien_z=0, orien_w=0, ik=compute_ik)



        right_gripper.open()
        rospy.sleep(1.0)

def move_to(pos_x, pos_y, pos_z, orien_x, orien_y, orien_z, orien_w, ik):
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

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # TRY THIS
        # Setting just the position without specifying the orientation
        # group.set_position_target([0.5, 0.5, 0.0])

        # Plan IK and execute
        group.go()
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()

