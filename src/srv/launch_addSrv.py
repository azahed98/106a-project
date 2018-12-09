#!/usr/bin/env python
import sys
import rospy
from srv import AddTwoInts


def add_two_ints(req):
	return srv.AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
	rospy.init_node('add_two_ints_server')
	s = rospy.Service('add_two_ints', AddTwoInts, add_two_ints)
	rospy.spin()

def main():
	add_two_ints_server()
	


if __name__ == "__main__":
    main()