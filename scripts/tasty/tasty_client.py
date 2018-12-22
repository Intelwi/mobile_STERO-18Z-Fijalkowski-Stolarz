#!/usr/bin/env python

import sys
import rospy
from stero_mobile_init.srv import CallItAsUWantIt

def callThaServer(desc, a, b):
	rospy.wait_for_service('tasty_pictalk')
	try:
		handler = rospy.ServiceProxy('tasty_pictalk', CallItAsUWantIt)
		response = handler(desc, a, b)
		return response.status
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e

def usage():
	return "Usage is:\n	%s [x y]" %sys.argv[0]

if __name__ == "__main__":
	if len(sys.argv) == 4:
		desc = sys.argv[1]
		a = float(sys.argv[2])
		b = float(sys.argv[3])
	else:
		print usage()
		sys.exit(1)
	print "Requesting: %s those numbers: %s & %s" %(desc, a, b)
	print "Response status is: ", callThaServer(desc, a, b)
