#!/usr/bin/env python

import sys
import rospy
from stero_mobile_init.srv import Positioning


def callThaServer(position):
	rospy.wait_for_service('set_position')
	try:
		handler = rospy.ServiceProxy('set_position', Positioning)
		response = handler(position)
		return response.status
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e


def countPose2D(x, y, theta):
	position = Pose2D()
	position.x = x
	position.y = y
	position.theta = theta
	return position


def usage():
	return "Usage is:\n	%s [x y]" %sys.argv[0]


if __name__ == "__main__":
	if len(sys.argv) == 4:
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		theta = float(sys.argv[3])
	else:
		print usage()
		sys.exit(1)
	print "Requesting: %s those numbers: %s & %s" %(x, y, theta)
	print "Response status is: ", callThaServer(countPose2D(x, y, theta))
