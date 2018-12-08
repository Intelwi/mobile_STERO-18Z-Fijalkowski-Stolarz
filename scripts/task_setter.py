#!/usr/bin/env python
# license removed for brevity
import rospy


def talker():
	pub = rospy.Publisher('chatter', Whisper, queue_size=10)  # TOPIC: chatter
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1) # 1hz
	while not rospy.is_shutdown():
		pose = Whisper()
		pose.desc = "to_point"
		pose.x = 2
		pose.y = 2
		rospy.loginfo(pose)
		pub.publish(pose)
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
