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
		for i in range(9) :
			print " %s - %s" %(i, modeDesc(i))
		return
	
	if argNum < 4 :
		print "Not enough arguments given. Usage is like:"
	else :
		print "Too much arguments given. Usage is like:"
	print "	rosrun stero_mobile_init problisher.py x y mode"
	print "For avaliable modes type in:"
	print "	rosrun stero_mobile_init problisher.py help"


def modeDesc(mode):
	global x, y
	
	if not wentThrough :
		x = "x"
		y = "y"
	
	if mode == 0 :
		return "move to position (%s, %s) [with time control]" %(x,y)
	elif mode == 1 :
		return "go there and back [with time control]"
	elif mode == 2 :
		return "rotate in place about 360 dagrees [with time control]"
	elif mode == 3 :
		return "go square, first corner in position (%s, %s) -> pre-counting\n     [with time control]" %(x,y)
	elif mode == 4 :
		return "move to position (%s, %s) [with feedback correction]" %(x,y)
	elif mode == 5 :
		return "go there and back [with feedback correction]"
	elif mode == 6 :
		return "rotate in place about 360 dagrees [with feedback correction]"
	elif mode == 7 :
		return "go square, first corner in position (%s, %s)\n     [with feedback correction]" %(x,y)
	elif mode == 8 :
		return "go square, first corner in position (%s, %s) -> pre-counting\n     [with feedback correction]" %(x,y)
	else :
		return "DESCRIPTION NOT AVALIABLE YET"


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
		
	if mode not in range(9) and mode not in [-8,-7,-6,-3,-2]:
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
