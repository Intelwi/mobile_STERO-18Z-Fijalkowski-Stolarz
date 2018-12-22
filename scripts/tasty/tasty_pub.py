#!/usr/bin/env python

import rospy
from stero_mobile_init.msg import Whisper


def talker():
	pub = rospy.Publisher('tasty_topic', Whisper, queue_size=10)
	rospy.init_node('tasty_talker', anonymous=True)
	rate = rospy.Rate(5) # 5hz

	i = 0
	j = 3.14
	while not rospy.is_shutdown():
		whisky = Whisper()
		whisky.desc = "elo ziom"
		whisky.x = i
		whisky.y = j
		rospy.loginfo(whisky)
		pub.publish(whisky)
		i=i+1
		j=j-0.3
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
