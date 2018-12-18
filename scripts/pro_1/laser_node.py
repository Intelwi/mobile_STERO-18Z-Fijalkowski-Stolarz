#!/usr/bin/env python
import rospy
import math
import PyKDL
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from decimal import Decimal


real_x = 0
real_y = 0
real_theta = 0

def listener():
	"""
	Uruchomienie node'a i nasluchiwanie na topicach:
	'/chatter'
	'/elektron/mobile_base_controller/odom'
	'/pose2D'
	
	"""
	rospy.init_node('odom_node', anonymous=True)
	rospy.Subscriber('/pose2D', Pose2D, getLaserNav) #do badania laser
	rospy.Subscriber('/gazebo_odom', Odometry, getGazeboNav)
	

	print "READY TO DO A JOB"


def getLaserNav(data):
	"""
	Odbiera wiadomosci z lasera - aktualnej pozycji robota
	
	"""
	global real_x
	global real_y
	global real_theta
	
	fake_x = data.x
	fake_y = data.y

	error_x = abs(fake_x - real_x)
	error_y = abs(fake_y - real_y)

	fake_theta = data.theta
	error_theta = abs(fake_theta - real_theta)

	pub = rospy.Publisher('/error', Pose2D, queue_size=10) # TOPIC: /error
	error_msg = Pose2D()
	error_msg.x = error_x
	error_msg.y = error_y
	error_msg.theta = error_theta

	print "error_x: ", error_x,"	error_y: ",error_y, "	error_theta: ",error_theta
	
	pub.publish(error_msg)	
	

def getGazeboNav(data):
	"""
	Odbiera wiadomosci z lasera - aktualnej pozycji robota
	
	"""	
	global real_x
	global real_y
	global real_theta
	real_x = data.pose.pose.position.x
	real_y = data.pose.pose.position.y
				
	rot = PyKDL.Rotation.Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
	[roll,pitch,yaw] = rot.GetRot()
	real_theta = yaw;

###------------------------------------------v--MAIN--v------------------------------------------###

if __name__ == '__main__':
	listener()
	rospy.spin()
	#while not rospy.is_shutdown():
		
