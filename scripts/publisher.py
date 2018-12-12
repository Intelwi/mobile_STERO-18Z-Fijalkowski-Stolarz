#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose

def talker():
	pub = rospy.Publisher('chatter', Pose, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1) # 1hz
	while not rospy.is_shutdown():
		pose = Pose()
		pose.position.x = 0
		pose.position.y = 0
		pose.position.z = 1
		rospy.loginfo(pose)
		pub.publish(pose)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
