#!/usr/bin/env python
# license removed for brevity
import sys
import rospy
from geometry_msgs.msg import Pose

wentThrough = False
overRange = False


def usage(argNum):
	if (argNum == 2 and sys.argv[1] == "help") or overRange :
		print "Avaliable modes: "
		for i in range(6) :
			print " %s - %s" %(i, modeDesc(i))
		return
	
	if argNum < 4 :
		print "Not enough arguments given. Usage is like:"
	else :
		print "Too much arguments given. Usage is like:"
	print "	rosrun stero_mobile_init publisher.py x y mode"
	print "For avaliable modes type in:"
	print "	rosrun stero_mobile_init publisher.py help"


def modeDesc(mode):
	global x, y
	
	if not wentThrough :
		x = "x"
		y = "y"
	
	if mode == 0 :
		return "move to position (%s, %s) without feedback correction" %(x,y)
	elif mode == 1 :
		return "move to position (%s, %s) with feedback correction" %(x,y)
	elif mode == 2 :
		return "go there and back"
	elif mode == 3 :
		return "rotate in place about 360 dagrees"
	elif mode == 4 :
		return "go square, first corner in position (%s, %s)" %(x,y)
	elif mode == 5 :
		return "go square, first corner in position (%s, %s) -> pre-counting" %(x,y)


def talker():
	pub = rospy.Publisher('chatter', Pose, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1) # 1hz
	
	pose = Pose()
	pose.position.x = x
	pose.position.y = y
	pose.position.z = mode
	
	#rospy.loginfo(pose)
	
	for i in range(2): # dla pewnosci 2 wiadomosci
		pub.publish(pose)
		rate.sleep()


if __name__ == '__main__':
	global x, y, mode
	
	argNum = len(sys.argv)
	
	if argNum == 4:
		x = float(sys.argv[1])
		y = float(sys.argv[2])
		mode = int(sys.argv[3])
	else:
		usage(argNum)
		sys.exit(1)
		
	if mode not in range(6) :
		overRange = True
		print "Non-existent mode request"
		usage(argNum)
		sys.exit(1)
	
	wentThrough = True
	
	print "--------------------"
	print "Requesting mode: (%s) %s" %(mode, modeDesc(mode))
	print "--------------------"
	
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
