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
	print "Requesting target position: (%s, %s) and orientation: %s rad..." %(x, y, theta)
	
	while True : # Czy da sie otrzymac kilka odpowiedzi?
		response = callThaServer(countPose2D(x, y, theta)) # or countPoseStamped(x, y, theta)
		if response == 0 :
			print "All went good. [%s]" %(response)
		elif response == -1 :
			print "Robot failed to reach set postion. [%s]" %(response)
		elif response == -2 :
			print "Robot failed to count the way. [%s]" %(response)
		elif response == -66 :
			print "Robot is executing other task now. Try again later. [%s]" %(response)
		elif response == 13 :
			print "Robot failed but retrying... [%s]" %(response)
			continue
		else :
			print "Unknown response [%s]" %(response)
			
		break
