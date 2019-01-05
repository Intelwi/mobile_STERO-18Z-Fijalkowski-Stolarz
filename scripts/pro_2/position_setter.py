#!/usr/bin/env python

import sys
import rospy
import PyKDL
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from stero_mobile_init.srv import Positioning
from stero_mobile_init.srv import Positioning2


def callThaServer(position):
	rospy.wait_for_service('set_position')
	try:
		handler = rospy.ServiceProxy('set_position', Positioning) # or Positioning2
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


def countPoseStamped(x, y, theta):
	position = PoseStamped()

	target.header.frame_id = "map"
	target.header.stamp = rospy.Time(0) # ???

	target.pose.position.x = x
	target.pose.position.y = y

	target.pose.orientation.x = qaternion[0]
	target.pose.orientation.y = qaternion[1]
	target.pose.orientation.z = qaternion[2]
	target.pose.orientation.w = qaternion[3]

	return position


def usage():
	return "Usage is:\n	%s [x y theta]" %sys.argv[0]


if __name__ == "__main__":
	if len(sys.argv) == 4:
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		theta = float(sys.argv[3])
	else:
		print usage()
		sys.exit(1)
	print "Requesting: %s those numbers: %s & %s" %(x, y, theta)
	print "Response status is: ", callThaServer(countPose2D(x, y, theta)) # or countPoseStamped(x, y, theta)
