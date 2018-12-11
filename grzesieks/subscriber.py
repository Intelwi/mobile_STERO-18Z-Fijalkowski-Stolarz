#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

omega = 2*math.pi/5 # pelny obrot na 5s
veloc = 1 # do ustalenia doswiadczalnie

def calcs(destination):
	x = destination.x
	y = destination.y
	fi = math.atan(abs(y)/x)
	s = math.sqrt(x*x+y*y)
	t_obr = fi/omega
	t_jazd = s/veloc

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)
	calcs(data.position)
    
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("chatter", Pose, callback)
	#talker()
	#rospy.spin()

def sendMess():
	vel_msg = Twist()
	vel_msg.linear.y = 0
	vel_msg.linear.x = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = omega
	return vel_msg
	

def talker():
	pub = rospy.Publisher('/mux_vel_nav/cmd_vel',Twist, queue_size=10)
	'''rate = rospy.Rate(1) # 10hz
    	while not rospy.is_shutdown():
		print "nie ma LIPY"
		vel_msg = sendMess()
		rospy.loginfo(vel_msg)
		pub.publish(vel_msg)
        	rate.sleep()'''
	vel_msg = sendMess()
	rospy.loginfo(vel_msg)
	pub.publish(vel_msg)
	rospy.sleep(10)
	vel_msg = sendMess()
	rospy.loginfo(vel_msg)
	pub.publish(vel_msg)

if __name__ == '__main__':
	listener()
	talker()
