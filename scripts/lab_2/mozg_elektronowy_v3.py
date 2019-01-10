#!/usr/bin/env python
import sys
import rospy
import math
import PyKDL
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from decimal import Decimal
from nav_msgs.srv import *
from stero_mobile_init.srv import Positioning

PERIOD = 10
OMEGA = 2*math.pi/PERIOD # pelny obrot na 30s
VELOC = 0.7 # predkosc postepowa 0.1
HZ = 50 # czestotliwsc wysylania wiadomosci
STATE = 1 # okresla zadanie jakie wykonuje robot (!=0 -> aktualna pozycja z odometrii/lasera)
erBreFa = VELOC*0.06 # early breaking factor
working = False # True -> robot podczas pracy -> nie zadasz nowego punktu
lefty = False # True -> kwadrat w lewo / obrot w lewo


class Position:
	"""
	Pozycja robota na plaszczyznie i kat obrotu (wzgledem pozycji poczatkowej)
	
	"""
	# class attributes (only used for setRoPo's)
	sign = 1 # kierunek obrotu
	tRot = 0 # czas obrotu
	tGo = 0 # czas jazdy
	s = 0 # droga do przebycia

	# instance attributes
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta # lewo: (0, PI), prawo: (0, -PI)
	
	# instance method
	def countRoute(self, s):
		self.s = s
		self.tGo = s/VELOC


cuRoPo = Position(0,0,0) # current Robot Position (x,y,theta) by odometry or laser 
setRoPo = Position(0,0,0) # zadana Robot Position (x,y,theta)
hopeRoPo = Position(0,0,0) # hoped Robot Position (x,y,theta) when no feedback 


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
	
	if STATE != 0 :
		dx = destination.x - cuRoPo.x
		dy = destination.y - cuRoPo.y
	else :
		dx = destination.x - hopeRoPo.x
		dy = destination.y - hopeRoPo.y
	
	if(abs(dx) < 0.1 and abs(dy) < 0.1): # zmniejszyc maksymalnie!!!
		print "ALREADY IN POINT"
		working = False
		return
	
	setRoPo.x = round(destination.x, 3)
	setRoPo.y = round(destination.y, 3)
	
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
	
	setRoPo.theta = round(theta, 3)
	print "Theta:", setRoPo.theta/(math.pi*2)*360, "stopni"
	
	""" Wzgledny kat obrotu fi -> czas i kierunek obrotu """
	if STATE != 0 :
		fi = setRoPo.theta - cuRoPo.theta # uwzglednienie aktualnego kata obrotu
	else :
		fi = setRoPo.theta - hopeRoPo.theta
	if(abs(fi) > math.pi): # zawsze obracaj sie tak jak masz blizej
		fi = -(abs(fi)/fi)*(2*math.pi - abs(fi))
	setRoPo.tRot = abs(fi)/OMEGA
	setRoPo.sign = fi and (1, -1)[fi < 0] # nie wiem jak to dziala, wzielem z internetu
	print "Fi:", fi/(math.pi*2)*360, "stopni"
	
	#if STATE == 0 or STATE == 4: # zeby nie liczyl bez sensu w przypadku z odometria
	""" Odleglosc w linii prostej do punktu """
	setRoPo.countRoute(math.sqrt(dx*dx + dy*dy))
	print "Do przejechania:", setRoPo.s, "m"


###-------------------------------------v--MESS-HANDLERS--v-------------------------------------###

def pathFollow(plan):
	for i in range(0,len(plan.plan.poses),4):
		manager(plan.plan.poses[i].pose.position)
	talkerToTwistOdom()


def pathFind(req):
	global setTheta
	rospy.wait_for_service('/global_planner/planner/make_plan')
	try:
		print req
		setTheta = req.position.theta
		#start point
		path_getter = rospy.ServiceProxy('/global_planner/planner/make_plan', GetPlan)
		start = PoseStamped()
		start.header.stamp = rospy.Time(0)
		start.header.frame_id = "map"
		start.pose.position.x = cuRoPo.x
		start.pose.position.y = cuRoPo.y
		start.pose.position.z = 0;
		
		#finish point
		finish = PoseStamped()
		finish.header.stamp = rospy.Time(0)
		finish.header.frame_id = "map"
		finish.pose.position.x = req.position.x
		finish.pose.position.y = req.position.y
		finish.pose.position.z = 0
		
		#pobranie planu jazdy
		plan = path_getter(start, finish,0.2)

		#jesli lista jest pusta
		if len(plan.plan.poses) == 0:
			rospy.logwarn("Empty plan calulated.")
			return -1
		else:
			pathFollow(plan)
			return 0		
		
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		rospy.logerr("Service call failed: %s"%e)
		return -1


def listener():
	"""
	Uruchomienie node'a i nasluchiwanie na topicach:
	'/chatter'
	'/elektron/mobile_base_controller/odom'
	'/pose2D'
	
	"""
	rospy.init_node('path_following', anonymous=True)
	rospy.Service('set_position', Positioning, pathFind)
	rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, getOdomNav)
	print "READY TO DO A JOB"


def getOdomNav(data):
	"""
	Odbiera wiadomosci z odometrii - aktualnej pozycji robota
	
	"""
	rot = PyKDL.Rotation.Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
	[roll,pitch,yaw] = rot.GetRot()
	cuRoPo.theta = round(yaw, 3)
	cuRoPo.x = round(data.pose.pose.position.x, 3)
	cuRoPo.y = round(data.pose.pose.position.y, 3)


def callback(data):
	"""
	Odbiera wiadomosci z punktem zadanym
	i wywoluje funkcje zarzadzajaca
	
	"""
	rospy.loginfo(rospy.get_caller_id() + "I heard an order: %s", data.position.z)
	if(not working): manager(data.position)


def manager(data):
	"""
	Na podstawie wartosci 'z' otrzymanej wiadomosci decyduje jaka akcje wykonac
	
	"""
	global STATE
	global working
	
	STATE = 1
	calcToPoint(data) # jesli po wykonaniu tej funkcji working==True, tzn ze przyjal te robote
	if(working):
		print "ZACZYNAM RUCH DO PKT #ODOM"
		talkerToPointOdom()
		working = False


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
	print "talkerToPoint STARTED"
	pub = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10) # TOPIC: /mux_vel_nav/cmd_vel
	rate = rospy.Rate(HZ)
	
	""" Obrot w strone punktu docelowego """
	message = buildMess(False, True, setRoPo.sign)
	#rospy.loginfo(message)
	i=0
	while i <= HZ*setRoPo.tRot:
		pub.publish(message)
		print "\rdest angle: ", format(setRoPo.theta,'.3f'), " current angle: ", format(cuRoPo.theta,'.3f'),
		sys.stdout.flush()
		i=i+1
		rate.sleep()
	
	""" Zatrzymanie robota obroconego do punktu docelowego """
	message = buildMess(False, False, 1)
	#rospy.loginfo(message)
	i=0
	while i<=HZ :
		pub.publish(message)
		i=i+1
		rate.sleep()
		
	hopeRoPo.theta = setRoPo.theta
	
	""" Jazda w strone punktu docelowego """
	message = buildMess(True, False, 1) # jedziemy zawsze przodem
	#rospy.loginfo(message)
	i=0
	while i<=HZ*setRoPo.tGo:
		pub.publish(message)
		print "dest x: ", format(setRoPo.x,'.3f'), "  current x: ", format(cuRoPo.x,'.3f')
		print "dest y: ", format(setRoPo.y,'.3f'), "  current y: ", format(cuRoPo.y,'.3f')
		print "--------"
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
	
	hopeRoPo.x = setRoPo.x
	hopeRoPo.y = setRoPo.y
	
	print "\ntalkerToPoint FINISHED with:"
	print "	dest angle: ", format(hopeRoPo.theta,'.3f'), " current angle: ", format(cuRoPo.theta,'.3f')
	print "	dest x: ", format(hopeRoPo.x,'.3f'), "  current x: ", format(cuRoPo.x,'.3f')
	print "	dest y: ", format(hopeRoPo.y,'.3f'), "  current y: ", format(cuRoPo.y,'.3f')
	print "\n"


def talkerToPointOdom():
	"""
	Publikuje polecenia do robota,
	majace na celu jego przemieszczenie do zadanego punktu
	uwzglednia odometrie
	
	Parameters ?
	----------
	setRoPo : Position()
		pozycja zadana dla robota
	
	"""
	print "talkerToPointOdom STARTED"
	pub = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10) # TOPIC: /mux_vel_nav/cmd_vel
	rate = rospy.Rate(HZ)
	
	""" Wyznaczenie warunku dla konca jazdy robota """
	if abs(setRoPo.x - cuRoPo.x) > abs(setRoPo.y - cuRoPo.y) :
		if cuRoPo.x < setRoPo.x :
			condition = lambda ebf: cuRoPo.x + ebf < setRoPo.x
		else :
			condition = lambda ebf: cuRoPo.x - ebf > setRoPo.x
	else :
		if cuRoPo.y < setRoPo.y :
			condition = lambda ebf: cuRoPo.y + ebf < setRoPo.y
		else :
			condition = lambda ebf: cuRoPo.y - ebf > setRoPo.y
	
	""" Obrot w strone punktu docelowego """
	message = buildMess(False, True, setRoPo.sign)
	while abs(setRoPo.theta - cuRoPo.theta) >= OMEGA*0.02 :
		pub.publish(message)
		print "\rdest angle: ", format(setRoPo.theta,'.3f'), " current angle: ", format(cuRoPo.theta,'.3f'),
		sys.stdout.flush()
		rate.sleep()
	
	""" Zatrzymanie robota """
	message = buildMess(False, False, 0)
	i=0
	while i<=HZ :
		pub.publish(message)
		i=i+1
		rate.sleep()
	
	""" Jazda w strone punktu docelowego """
	message = buildMess(True, False, 1) # jedziemy zawsze przodem
	while condition(erBreFa) : #sprawdzamy osiagniecie tylko jednej ze wspolrzednych docelowych
		pub.publish(message)
		print "dest x: ", format(setRoPo.x,'.3f'), "  current x: ", format(cuRoPo.x,'.3f')
		print "dest y: ", format(setRoPo.y,'.3f'), "  current y: ", format(cuRoPo.y,'.3f')
		print "--------"
		rate.sleep()
	
	""" Zatrzymanie robota """
	message = buildMess(False, False, 0)
	i=0
	while i<=HZ :
		pub.publish(message)
		i=i+1
		rate.sleep()
	
	print "\ntalkerToPointOdom FINISHED with:"
	print "	dest angle: ", format(setRoPo.theta,'.3f'), " current angle: ", format(cuRoPo.theta,'.3f')
	print "	dest x: ", format(setRoPo.x,'.3f'), "  current x: ", format(cuRoPo.x,'.3f')
	print "	dest y: ", format(setRoPo.y,'.3f'), "  current y: ", format(cuRoPo.y,'.3f')
	print "\n"


def talkerToTwistOdom():
	"""
	Publikuje polecenia do robota,
	majace na celu jego obrocenie sie w miejscu o 360 stopni
	uwzglednia odometrie
	
	Parameters ?
	----------
	setRoPo : Position()
		pozycja zadana dla robota
	
	"""
	print "talkerToTwistOdom STARTED"
	pub = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10) # TOPIC: /mux_vel_nav/cmd_vel
	rate = rospy.Rate(HZ)
	
	message = buildMess(False, True, lefty*2-1) # obrot w lewo gdy lewo==True
	
	""" Obrot dojezdzajacy do setTheta """
	while abs(cuRoPo.theta - setTheta) >= 0.07 :
		pub.publish(message)
		print "\rdest angle: ", format(setRoPo.theta,'.3f'), " current angle: ", format(cuRoPo.theta,'.3f'),
		sys.stdout.flush()
		rate.sleep()
	
	""" Zatrzymanie robota """
	message = buildMess(False, False, 0)
	i=0
	while i <= HZ :
		pub.publish(message)
		i=i+1
		rate.sleep()
		
	print "\ntalkerToTwistOdom FINISHED with:"
	print "	dest angle: ", format(setRoPo.theta,'.3f'), " current angle: ", format(cuRoPo.theta,'.3f')
	print "	dest x: ", format(setRoPo.x,'.3f'), "  current x: ", format(cuRoPo.x,'.3f')
	print "	dest y: ", format(setRoPo.y,'.3f'), "  current y: ", format(cuRoPo.y,'.3f')
	print "\n"

###------------------------------------------v--MAIN--v------------------------------------------###

if __name__ == '__main__':
	listener()
	rospy.spin()
	#while not rospy.is_shutdown():
		
