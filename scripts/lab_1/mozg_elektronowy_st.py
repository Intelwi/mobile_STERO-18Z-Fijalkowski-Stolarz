#!/usr/bin/env python
import rospy
import math
import PyKDL
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from decimal import Decimal

OMEGA = 2*math.pi/20 # pelny obrot na 15s
VELOC = 0.1 # predkosc postepowa - zmniejszyc???
HZ = 50 # czestotliwsc wysylania wiadomosci
working = False # True -> robot podczas pracy -> nie zadasz nowego punktu
STATE = 1

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
	global working
	working = True
	
	dx = destination.x - cuRoPo.x
	dy = destination.y - cuRoPo.y
	
	if(abs(dx) < 0.1 and abs(dy) < 0.1): # zmniejszyc maksymalnie!!!
		print "ALREADY IN POINT"
		working = False
		return
	
	setRoPo.x = destination.x
	setRoPo.y = destination.y
	
	""" Bezwzgledny kat obrotu theta """
	if(dx != 0):
		alfa = math.atan(dy/abs(dx))
		if(dx > 0):
			theta = alfa
		else:
			if(dy > 0):
				theta = math.pi - alfa
			else:
				theta = -math.pi - alfa
	else:
		theta = abs(dy)/dy * math.pi/2
	
	setRoPo.theta = theta
	print "Theta:", setRoPo.theta/(math.pi*2)*360, "stopni"
	
	""" Wzgledny kat obrotu fi -> czas i kierunek obrotu """
	fi = setRoPo.theta - cuRoPo.theta # uwzglednienie aktualnego kata obrotu
	if(abs(fi) > math.pi): # zawsze obracaj sie tak jak masz blizej
		fi = -(abs(fi)/fi)*(2*math.pi - abs(fi))
	setRoPo.tRot = abs(fi)/OMEGA
	setRoPo.sign = abs(fi)/fi
	print "Fi:", fi/(math.pi*2)*360, "stopni"
	
	if destination.z == 0 : # zeby nie liczyl bez sensu w przypadku z odometria
		""" Odleglosc w linii prostej do punktu """
		s = math.sqrt(dx*dx + dy*dy)
		setRoPo.tGo = s/VELOC
		print "Do przejechania:", s, "m"


###-------------------------------------v--MESS-HANDLERS--v-------------------------------------###

def listener():
	"""
	Uruchomienie node'a i nasluchiwanie na topicach:
	'chatter'
	'/elektron/mobile_base_controller/odom'
	
	"""
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('chatter', Pose, callback) # TOPIC: chatter
	rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, get_navigation)
	print "READY TO DO A JOB"


def get_navigation(data):
	"""
	Odbiera wiadomosci z odometrii - aktualnej pozycji robota
	i wywoluje funkcje zarzadzajaca
	
	"""
	#rospy.loginfo(rospy.get_caller_id() + "I got location: %s", data)
	if STATE != 0 :
		rot = PyKDL.Rotation.Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
		[roll,pitch,yaw] = rot.GetRot()
		cuRoPo.theta = yaw
		cuRoPo.x = data.pose.pose.position.x
		cuRoPo.y = data.pose.pose.position.y
		'''print "x = ", data.pose.pose.position.x
		print "y = ", data.pose.pose.position.y
		print "theta = ", yaw'''


def callback(data):
	"""
	Odbiera wiadomosci z punktem zadanym
	i wywoluje funkcje zarzadzajaca
	
	"""
	rospy.loginfo(rospy.get_caller_id() + "I heard an order: %s", data.position.z)
	manager(data.position)


def manager(data):
	"""
	Na podstawie wartosci 'z' otrzymanej wiadomosci decyduje jaka akcje wykonac
	
	"""
	global STATE
	
	if(data.z == 0):
		STATE = 0
		if(working == False): # rob cos tylko jak aktualnie nic nie robisz
			calcToPoint(data) # jesli po wykonaniu tej funkcji working==True, tzn ze przyjal te robote
			if(working):
				print "ZACZYNAM RUCH DO PKT"
				talkerToPoint()
	elif(data.z == 1):
		STATE = 1
		if(working == False): # rob cos tylko jak aktualnie nic nie robisz
			calcToPoint(data) # jesli po wykonaniu tej funkcji working==True, tzn ze przyjal te robote
			if(working):
				print "ZACZYNAM RUCH DO PKT #ODOM"
				talkerToPointOdom()
	'''elif(data.desc == "rotating"):
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

def talkerToPoint():
	"""
	Publikuje polecenia do robota,
	majace na celu jego przemieszczenie do zadanego punktu
	nie uwzglednia odometrii
	
	"""
	global working
	
	pub = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10) # TOPIC: /mux_vel_nav/cmd_vel
	rate = rospy.Rate(HZ)
	
	""" Obrot w strone punktu docelowego """
	message = buildMess(False, True, setRoPo.sign)
	#rospy.loginfo(message)
	i=0
	while i <= HZ*setRoPo.tRot:
		pub.publish(message)
		i=i+1
		rate.sleep()
	
	""" Zatrzymanie robota w punkcie docelowym """
	message = buildMess(False, False, 1)
	#rospy.loginfo(message)
	i=0
	while i<=HZ :
		pub.publish(message)
		i=i+1
		rate.sleep()
		
	cuRoPo.theta = setRoPo.theta
	
	""" Jazda w strone punktu docelowego """
	message = buildMess(True, False, 1) # jedziemy zawsze przodem
	#rospy.loginfo(message)
	i=0
	while i<=HZ*setRoPo.tGo:
		pub.publish(message)
		i=i+1
		rate.sleep()
	
	""" Zatrzymanie robota w punkcie docelowym """
	message = buildMess(False, False, 1)
	#rospy.loginfo(message)
	i=0
	while i<=HZ :
		pub.publish(message)
		i=i+1
		rate.sleep()
	
	cuRoPo.x = setRoPo.x
	cuRoPo.y = setRoPo.y
	working = False


def talkerToPointOdom():
	"""
	Publikuje polecenia do robota,
	majace na celu jego przemieszczenie do zadanego punktu
	uwzglednia odometrie
	
	"""
	global working
	
	pub = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10) # TOPIC: /mux_vel_nav/cmd_vel
	rate = rospy.Rate(HZ)
	
	""" Obrot w strone punktu docelowego """
	message = buildMess(False, True, setRoPo.sign)
	#rospy.loginfo(message)
	while round(setRoPo.theta, 2) != round(cuRoPo.theta, 2): #korekcja z enkoderami
		pub.publish(message)
		print "dest angle: ", setRoPo.theta, " current angle: ", cuRoPo.theta
		rate.sleep()
	
	""" Zatrzymanie obrotu robota """
	message = buildMess(False, False, 1)
	i=0
	while i<=HZ :
		pub.publish(message)
		i=i+1
		rate.sleep()
	
	""" Jazda w strone punktu docelowego """
	message = buildMess(True, False, 1) # jedziemy zawsze przodem
	while round(setRoPo.x, 2) != round(cuRoPo.x, 2): #korekcja z enkoderami
		pub.publish(message)
		print "dest x: ", setRoPo.x, "  current x: ", cuRoPo.x
		print "dest y: ", setRoPo.y, "  current y: ", cuRoPo.y
		rate.sleep()
	
	""" Zatrzymanie obrotu robota """
	message = buildMess(False, False, 1)
	i=0
	while i<=HZ :
		pub.publish(message)
		i=i+1
		rate.sleep()
	
	working = False


###------------------------------------------v--MAIN--v------------------------------------------###

if __name__ == '__main__':
	listener()
	rospy.spin()
	#while not rospy.is_shutdown():
		
