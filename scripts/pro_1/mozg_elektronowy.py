#!/usr/bin/env python
import rospy
import math
import PyKDL
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from decimal import Decimal

OMEGA = 2*math.pi/30 # pelny obrot na 20s
VELOC = 0.1 # predkosc postepowa
HZ = 50 # czestotliwsc wysylania wiadomosci
STATE = 1 # okresla zadanie jakie wykonuje robot (!=0 -> aktualna pozycja z odometrii/lasera)
working = False # True -> robot podczas pracy -> nie zadasz nowego punktu
withLaser = False # True -> nawigacja z uzyciem lasera

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
	fi = setRoPo.theta - cuRoPo.theta # uwzglednienie aktualnego kata obrotu
	if(abs(fi) > math.pi): # zawsze obracaj sie tak jak masz blizej
		fi = -(abs(fi)/fi)*(2*math.pi - abs(fi))
	setRoPo.tRot = abs(fi)/OMEGA
	setRoPo.sign = fi and (1, -1)[fi < 0] # nie wiem jak to dziala, wzielem z internetu
	print "Fi:", fi/(math.pi*2)*360, "stopni"
	
	#if STATE == 0 or STATE == 4: # zeby nie liczyl bez sensu w przypadku z odometria
	""" Odleglosc w linii prostej do punktu """
	setRoPo.countRoute(math.sqrt(dx*dx + dy*dy))
	print "Do przejechania:", setRoPo.s, "m"


def calcNextSquarePoint():
	"""
	Wyznacza nastepny punkt ruchu po kwadracie
	na podstawie aktualnej pozycji cuRoPo
	
	Returns
	-------
	Position()
	
	"""
	vecRo = PyKDL.Vector(cuRoPo.x, cuRoPo.y, 0)
	twiRo = PyKDL.Rotation.RPY(0, 0, cuRoPo.theta)
	mxJednoRo = PyKDL.Frame(twiRo, vecRo)
	vecDestRel = PyKDL.Vector(0, setRoPo.s, 0)
	vecDest = mxJednoRo * vecDestRel
	nextSqPnt = Position(vecDest[0], vecDest[1], 0)
	print "nextSqPnt.x:", nextSqPnt.x, "  nextSqPnt.y:", nextSqPnt.y
	return nextSqPnt


def calcAllSquarePoints():
	"""
	Wyznacza wszystkie kolejne punkty ruchu po kwadracie
	na podstawie zadanej pozycji setRoPo wyliczonej wczesniej przez calcToPoint()
	oraz aktualnej pozycji cuRoPo
	
	Returns
	-------
	Position() x3
	
	"""
	print "squarePoint1.x:", setRoPo.x, "  squarePoint1.y:", setRoPo.y
	
	""" Tworzenie macierzy jednorodnej robota w setRoPo """
	vecRo = PyKDL.Vector(setRoPo.x, setRoPo.y, 0)
	twiRo = PyKDL.Rotation.RPY(0, 0, setRoPo.theta)
	mxJednoRo = PyKDL.Frame(twiRo, vecRo)
	
	vecDestRel = PyKDL.Vector(0, setRoPo.s, 0)
	vecDest = mxJednoRo * vecDestRel
	squarePoint2 = Position(vecDest[0], vecDest[1], 0)
	print "squarePoint2.x:", squarePoint2.x, "  squarePoint2.y:", squarePoint2.y
	
	vecDestRel = PyKDL.Vector(-setRoPo.s, setRoPo.s, 0)
	vecDest = mxJednoRo * vecDestRel
	squarePoint3 = Position(vecDest[0], vecDest[1], 0)
	print "squarePoint3.x:", squarePoint3.x, "  squarePoint3.y:", squarePoint3.y
	
	squarePoint4 = Position(cuRoPo.x, cuRoPo.y, 0)
	print "squarePoint4.x:", squarePoint4.x, "  squarePoint4.y:", squarePoint4.y
	
	return squarePoint2, squarePoint3, squarePoint4


###-------------------------------------v--MESS-HANDLERS--v-------------------------------------###

def listener():
	"""
	Uruchomienie node'a i nasluchiwanie na topicach:
	'/chatter'
	'/elektron/mobile_base_controller/odom'
	'/pose2D'
	
	"""
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('/chatter', Pose, callback)
	rospy.Subscriber('/elektron/mobile_base_controller/odom', Odometry, getOdomNav)
	rospy.Subscriber('/pose2D', Pose2D, callback)
	print "READY TO DO A JOB"


def getOdomNav(data):
	"""
	Odbiera wiadomosci z odometrii - aktualnej pozycji robota
	i wywoluje funkcje zarzadzajaca
	
	"""
	#rospy.loginfo(rospy.get_caller_id() + "I got location: %s", data)
	if STATE != 0 and not withLaser :
		rot = PyKDL.Rotation.Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
		[roll,pitch,yaw] = rot.GetRot()
		cuRoPo.theta = round(yaw, 3)
		cuRoPo.x = round(data.pose.pose.position.x, 3)
		cuRoPo.y = round(data.pose.pose.position.y, 3)


def getLaserNav(data):
	"""
	Odbiera wiadomosci z lasera - aktualnej pozycji robota
	i wywoluje funkcje zarzadzajaca
	
	"""
	#rospy.loginfo(rospy.get_caller_id() + "I got location: %s", data)
	if STATE != 0 and withLaser :
		cuRoPo.theta = round(data.theta, 3)
		cuRoPo.x = round(data.x, 3)
		cuRoPo.y = round(data.y, 3)


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
	
	if(data.z == 0):
		STATE = 0
		calcToPoint(data) # jesli po wykonaniu tej funkcji working==True, tzn ze przyjal te robote
		if(working):
			print "ZACZYNAM RUCH DO PKT"
			talkerToPoint()
			working = False
			
	elif(data.z == 1):
		STATE = 1
		calcToPoint(data) # jesli po wykonaniu tej funkcji working==True, tzn ze przyjal te robote
		if(working):
			print "ZACZYNAM RUCH DO PKT #ODOM"
			talkerToPointOdom()
			working = False
			
	elif(data.z == 2):
		STATE = 2
		calcToPoint(data) # jesli po wykonaniu tej funkcji working==True, tzn ze przyjal te robote
		if(working):
			print "ZACZYNAM TEST ~PRZOD TYL~ #ODOM"
			#TO DO
			talkerToPointOdom()
			working = False
			
	elif(data.z == 3):
		STATE = 3
		print "ZACZYNAM TEST ~OBROT~ #ODOM"
		talkerToTwist()
		working = False
		
	elif(data.z == 4):
		STATE = 4
		calcToPoint(data) # jesli po wykonaniu tej funkcji working==True, tzn ze przyjal te robote
		if(working):
			print "ZACZYNAM TEST ~RUCH PO KWADRACIE~ #ODOM"
			talkerToPointOdom()
			for i in range(3):
				nextPoint = calcNextSquarePoint()
				calcToPoint(nextPoint)
				talkerToPointOdom()
			working = False
			
	elif(data.z == 5):
		STATE = 5
		calcToPoint(data) # jesli po wykonaniu tej funkcji working==True, tzn ze przyjal te robote
		if(working):
			print "ZACZYNAM TEST ~RUCH PO KWADRACIE v2~ #ODOM"
			[sqPnt2, sqPnt3, sqPnt4] = calcAllSquarePoints()
			talkerToPointOdom()
			
			calcToPoint(sqPnt2)
			talkerToPointOdom()
			
			calcToPoint(sqPnt3)
			talkerToPointOdom()
			
			calcToPoint(sqPnt4)
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
	
	''' Wyznaczenie warunku dla konca jazdy robota '''
	if abs(setRoPo.x - cuRoPo.x) > abs(setRoPo.y - cuRoPo.y) :
		condition = lambda : abs(setRoPo.x - cuRoPo.x) >= 0.002
	else :
		condition = lambda : abs(setRoPo.y - cuRoPo.y) >= 0.002
	
	""" Obrot w strone punktu docelowego """
	message = buildMess(False, True, setRoPo.sign)
	while abs(setRoPo.theta - cuRoPo.theta) >= 0.007 :
		pub.publish(message)
		print "dest angle: ", format(setRoPo.theta,'.3f'), " current angle: ", format(cuRoPo.theta,'.3f')
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
	while condition() : #sprawdzamy osiagniecie tylko jednaj ze wspolrzednych docelowych
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


def talkerToTwist():
	"""
	Publikuje polecenia do robota,
	majace na celu jego obrocenie sie w miejscu o 360 stopni
	uwzglednia odometrie
	
	Parameters ?
	----------
	setRoPo : Position()
		pozycja zadana dla robota
	
	"""
	print "talkerToTwist STARTED"
	pub = rospy.Publisher('/mux_vel_nav/cmd_vel', Twist, queue_size=10) # TOPIC: /mux_vel_nav/cmd_vel
	rate = rospy.Rate(HZ)
	
	""" Obrot odjezdzajacy od 0 """
	message = buildMess(False, True, 1) # obrot w lewo
	while cuRoPo.theta < 1 :
		pub.publish(message)
		print "dest angle: ", format(setRoPo.theta,'.3f'), " current angle: ", format(cuRoPo.theta,'.3f')
		rate.sleep()
	
	""" Obrot dojezdzajacy do 0 """
	while abs(cuRoPo.theta) >= 0.007 :
		pub.publish(message)
		print "dest angle: ", format(setRoPo.theta,'.3f'), " current angle: ", format(cuRoPo.theta,'.3f')
		rate.sleep()
	
	""" Zatrzymanie robota """
	message = buildMess(False, False, 0)
	i=0
	while i<=HZ :
		pub.publish(message)
		i=i+1
		rate.sleep()
		
	print "\ntalkerToTwist FINISHED with:"
	print "	dest angle: ", format(setRoPo.theta,'.3f'), " current angle: ", format(cuRoPo.theta,'.3f')
	print "	dest x: ", format(setRoPo.x,'.3f'), "  current x: ", format(cuRoPo.x,'.3f')
	print "	dest y: ", format(setRoPo.y,'.3f'), "  current y: ", format(cuRoPo.y,'.3f')
	print "\n"


###------------------------------------------v--MAIN--v------------------------------------------###

if __name__ == '__main__':
	listener()
	rospy.spin()
	#while not rospy.is_shutdown():
		
