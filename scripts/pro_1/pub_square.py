#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose

def talker():
	pub = rospy.Publisher('chatter', Pose, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	
	pose = Pose()
	
	""" Rozpoczecie z punktu (0, 0) """
	pose.position.x = 0
	pose.position.y = 0
	pose.position.z = 1
	rospy.loginfo(pose)
	pub.publish(pose)
	
	""" Punkt 1 """
	pose.position.x = 1
	pose.position.y = 0
	pose.position.z = 1
	rospy.loginfo(pose)
	pub.publish(pose)
	
	""" Punkt 2 """
	pose.position.x = 1
	pose.position.y = -1
	pose.position.z = 1
	rospy.loginfo(pose)
	pub.publish(pose)
	
	""" Punkt 3 """
	pose.position.x = 0
	pose.position.y = -1
	pose.position.z = 1
	rospy.loginfo(pose)
	pub.publish(pose)
	
	""" Punkt 4 - powrot do bazy """
	pose.position.x = 0
	pose.position.y = 0
	pose.position.z = 1
	rospy.loginfo(pose)
	pub.publish(pose)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
