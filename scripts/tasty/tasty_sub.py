#!/usr/bin/env python

import rospy
from stero_mobile_init.msg import Whisper


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s, %s" %(data.desc, data.x, data.y))


def listener():
	# handling msg:
	rospy.init_node('tasty_listener', anonymous=True)
	rospy.Subscriber('tasty_topic', Whisper, callback)

	print "READY TO WORK"
	rospy.spin() # simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
	listener()
