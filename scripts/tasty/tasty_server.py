#!/usr/bin/env python

import rospy
from stero_mobile_init.srv import CallItAsUWantIt


def handle_req(req):
	print "Received request description is: ", req.desc
	reqSum = req.a + req.b
	print "Returning sign of [%s + %s = %s]"%(req.a, req.b, reqSum)
	return reqSum and (1, -1)[reqSum < 0] # status: znak sumy


def listener():
	# handling srv:
	rospy.init_node('tasty_server')
	s = rospy.Service('tasty_pictalk', CallItAsUWantIt, handle_req)

	print "READY TO WORK"
	rospy.spin() # simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
	listener()
