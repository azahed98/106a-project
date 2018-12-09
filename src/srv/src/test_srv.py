#!/usr/bin/env python

import rospy


def main():
	add_two_ints = rospy.ServiceProxy('add_two_ints', srv.srv.AddTwoInts)
	result = add_two_ints(1, 2)
	print(result)

if __name__ == "__main__":
    main()