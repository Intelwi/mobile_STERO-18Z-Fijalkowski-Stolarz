#!/usr/bin/env python
import rospy
import math
import PyKDL
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from decimal import Decimal

OMEGA = 2*math.pi/40 # pelny obrot na 5s
VELOC = 0.8 # do ustalenia doswiadczalnie
HZ = 50

class Position:
	"""
	Pozycja robota na plaszczyznie i kat obrotu (wzgledem pozycji poczatkowej)
	
	"""
	# class attributes (only used for setRoPo's)
	sign = 1 # kierunek obrotu
	tRot = 0 # czas obrotu
	tGo = 0 # czas jazdy

	# instance attributes
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta # lewo: (0, PI), prawo: (0, -PI)


cuRoPo = Position(0,0,0) # current Robot Position (x,y,theta)
setRoPo = Position(0,0,0) # zadana Robot Position (x,y,theta)


###-----------------------------------------v--CALCS--v-----------------------------------------###

def calcToPoint(destination):
	"""
	Wyznacza czas i kierunek obrotu oraz czas jazdy, aby robot osiagnal zadany punkt
	
	Parameters
	----------
	destination : Pose (or anything that has x and y)
		punkt do ktorego ma dojechac robot
	
	Returns
	-------
	setRoPo values
	
	"""
	setRoPo.x = destination.x
	setRoPo.y = destination.y
	
	dx = destination.x - cuRoPo.x
	dy = destination.y - cuRoPo.y
	
	if(dx == 0 and dy == 0):
		setRoPo.tRot = 0
		setRoPo.tGo = 0
		return
	
	""" Bezwzgledny kat obrotu theta """
	if(dx != 0):
		fi = math.atan(dy/abs(dx))
		if(dx > 0):
			theta = fi
		else:
			if(dy > 0):
				theta = math.pi - fi
			else:
				theta = -math.pi - fi
	else:
		theta = abs(dy)/dy * math.pi/2
		
	setRoPo.theta = theta
	print "Theta:", theta/(math.pi*2)*360, "stopni"
	
	""" Wzgledny kat obrotu fi """
	fi = theta - cuRoPo.theta # uwzglednienie aktualnego kata obrotu
	if(abs(fi) > math.pi): # zawsze obracaj sie tak jak masz blizej
		fi = -(2*math.pi - abs(fi))
	setRoPo.tRot = abs(fi)/OMEGA
	setRoPo.sign = abs(fi)/fi
	print "Fi:", fi/(math.pi*2)*360, "stopni"
	
	""" Odleglosc w linii prostej do punktu """
	s = math.sqrt(dx*dx + dy*dy)
	setRoPo.tGo = s/VELOC
	print "Do przejechania:", s, "m"


###-------------------------------------v--MESS-HANDLERS--v-------------------------------------###

def listener():
	"""
	Uruchomienie node'a i nasluchiwanie na topicu 'chatter'
	
	"""
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('chatter', Pose, callback) # TOPIC: chatter
	rospy.Subscriber('/elektron/mobile_base_controller/odom',Odometry,get_navigation)
	print "READY TO DO A JOB"

def get_navigation(data):
	#rospy.loginfo(rospy.get_caller_id() + "I got location: %s", data)
	rot = PyKDL.Rotation.Quaternion(data.pose.pose.orientation.x ,data.pose.pose.orientation.y ,data.pose.pose.orientation.z ,data.pose.pose.orientation.w )
	[roll,pitch,yaw] = rot.GetRot()
	cuRoPo.theta = yaw
	cuRoPo.x = data.pose.pose.position.x
	cuRoPo.y = data.pose.pose.position.y	

def callback(data):
	"""
	Odbiera wiadomosci i wywoluje funkcje zarzadzajaca
	
	"""
	#rospy.loginfo(rospy.get_caller_id() + "I heard an order: %s", data.position.z)
	manager(data.position)


def manager(data):
	"""
	Na podstawie stringa 'desc' otrzymanej wiadomosci decyduje jaka akcje wykonac
	
	"""
	if(data.z == 0):
		calcToPoint(data)
		if(setRoPo.tRot != 0 or setRoPo.tGo != 0):
			print "JEST OK"
			talkerToPoint(data)
	'''elif(data.desc == "there_and_back"):
		#TO DO
	elif(data.desc == "rotating"):
		#TO DO
	elif(data.desc == "square"):
		#TO DO'''


###-------------------------------------v--MESS-BUILDERS--v-------------------------------------###

def buildMess(go, rot, sign):
	"""
	Buduje wiadomosc przekazywana do robota

	Parameters
	----------
	go : bool
		if True - zadajemy predkosc postepowa
	rot : bool
		if True - zadajemy predkosc obrotowa
	sign : 1/-1
		okresla kierunek obrotu (lewo/prawo) i jazdy (przod/tyl)

	Returns
	-------
	Twist()
	
	"""
	vel_msg = Twist()
	vel_msg.linear.y = 0
	vel_msg.linear.x = VELOC*go*sign
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = OMEGA*rot*sign
	#print "predkosc obrotowa:", vel_msg.angular.z
	return vel_msg


###----------------------------------------v--TALKERS--v----------------------------------------###

def talkerToPoint(data):
	"""
	Publikuje polecenia do robota,
	majace na celu jego przemieszczenie do zadanego punktu
	
	"""
	pub = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10) # TOPIC: /mux_vel_nav/cmd_vel
	rate = rospy.Rate(HZ) # 50hz


	""" Obrot w strone punktu docelowego """
	message = buildMess(False, True, setRoPo.sign)
	rospy.loginfo(message)
	i = 0
	while round(setRoPo.theta,3) != round(cuRoPo.theta,3):#korekcja z enkoderami
		calcToPoint(data)
		message = buildMess(False, True, setRoPo.sign)
		pub.publish(message)
		print "dest angle: ",setRoPo.theta,"	current angle: ",cuRoPo.theta
		rate.sleep()
	
	""" Zatrzymanie robota w punkcie docelowym """
	i=0
	while i<=HZ*2 :
		message = buildMess(False, False, 1)
		pub.publish(message)
		i=i+1
		rate.sleep()
	
	""" Jazda w strone punktu docelowego """
	'''i=0
	while i<=HZ*setRoPo.tGo:
		message = buildMess(True, False, 1) # jedziemy zawsze przodem
		if(i==1):
			rospy.loginfo(message)

		pub.publish(message)
		i=i+1
		rate.sleep()'''
	
	""" Zatrzymanie robota w punkcie docelowym """
	'''i=0
	while i<=HZ*2 :
		message = buildMess(False, False, 1)
		pub.publish(message)
		cuRoPo.x = setRoPo.x
		cuRoPo.y = setRoPo.y
		i=i+1
		rate.sleep()'''


###------------------------------------------v--MAIN--v------------------------------------------###

if __name__ == '__main__':
	listener()
	rospy.spin()
	#while not rospy.is_shutdown():
		
