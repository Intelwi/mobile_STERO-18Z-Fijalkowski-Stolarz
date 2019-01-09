#include "ros/ros.h"
#include "stero_mobile_init/Positioning.h"
#include "stero_mobile_init/Positioning2.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <cmath>
//#include <math.h>
#include "costmap_2d/costmap_2d_ros.h"
#include "tf2_ros/buffer.h"
#include "stdbool.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/utils.h>
#include <tf/transform_listener.h>
#include <global_planner/planner_core.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <vector>
#include <nav_core/recovery_behavior.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <rotate_recovery/rotate_recovery.h>

#define HZ 10
#define XX 50

/* Funkcja konwertująca pozycję (x,y,theta) na PoseStamped */
geometry_msgs::PoseStamped convertToPoseStamped(double, double, double);

int planAndExecute();
void rotate360();
double countAverageSpeed(double *);
void pushNextSpeed(double *, double);
int whatToDo(int);
bool openGate = false;

/* Współrzędne celu */
double targX, targY, targTheta;

//flaga czy planery zrobiły plany
bool isPlanComputed = false;

//flaga czy robot wyznaczył lokalną trajektorię
bool isTrajectoryComputed = false;

//flaga czy rozpoczeto planowanie
bool isStarted = false;

//flaga czy zainicjowano local ekipe
bool localCrewInitiated = false;

//zaplanowana sciezka jazdy
std::vector<geometry_msgs::PoseStamped> plan;

//punkt startowy - bedzie pobierane z odometrii to jest wersja próbna
geometry_msgs::PoseStamped start;

//punkt koncowy
geometry_msgs::PoseStamped target;

//wyliczone predkosci przez planer lokalny
geometry_msgs::Twist velocities;


bool reqHandler(stero_mobile_init::Positioning::Request  &req,
                stero_mobile_init::Positioning::Response  &res)
{
	if(isStarted)
	{
		res.status = -66;
		ROS_INFO("sending back response: [%d]", res.status);
		return true;
	}
	int status;
	targX = req.position.x;
	targY = req.position.y;
	targTheta = req.position.theta;
	
	ROS_INFO("I got co-ordinates: (%f, %f) and orientation: %f rad", targX, targY, targTheta);
	
	do {
		ros::spinOnce(); // odświeżenie odometrii w razie replanningu
		status = 0;
		status = planAndExecute();
		ROS_INFO("Got response: [%d]", status);
	} while(status == 13);
	
	isStarted = false;
	res.status = status;
	ROS_INFO("sending back response: [%d]", res.status);
	
	return true;
}


/* Odbiera aktualną pozycję robota i wstawia do zmiennej "start".
 * Robi to tylko wtedy, gdy robot nie wykonuje zadania */
void getOdomNav(const nav_msgs::Odometry::ConstPtr&  msg)
{
	if(!isStarted || openGate)
	{
		//std::cout<<"elo from getOdomNav"<<std::endl;
		start.header.frame_id = "map";
		start.header.stamp = ros::Time(0);

		start.pose.position.x = msg->pose.pose.position.x;
		start.pose.position.y = msg->pose.pose.position.y;
		
		start.pose.orientation.x = msg->pose.pose.orientation.x;
		start.pose.orientation.y = msg->pose.pose.orientation.y;
		start.pose.orientation.z = msg->pose.pose.orientation.z;
		start.pose.orientation.w = msg->pose.pose.orientation.w;
	}
}

costmap_2d::Costmap2DROS *costmap;
global_planner::GlobalPlanner *elektron_global_planner;

ros::Rate *loop_rate;
ros::Publisher velocity_pub;

int main(int argc, char **argv)
{
	//inicjalizacja glownego node'a
	ros::init(argc, argv, "alles_server");
	ros::NodeHandle n;
	
	//inicjalizacja globalnego planera i jego mapy kosztów
	tf2_ros::Buffer  buffer(ros::Duration(10),true);
	tf2_ros::TransformListener  tf(buffer);
	costmap = new costmap_2d::Costmap2DROS("costmap", buffer);
	(*costmap).start();
	elektron_global_planner = new global_planner::GlobalPlanner("elektron_global_planner",(*costmap).getCostmap(),"map");

	//inicjalizacja servisu odbierającego położenie docelowe
	ros::ServiceServer service = n.advertiseService("set_position", reqHandler);
	//inicjalizacja publikowania prędkości do robota
	velocity_pub = n.advertise<geometry_msgs::Twist>("/mux_vel_raw/cmd_vel", 500);
	//inicjalizacja odbierania aktualnej pozycji z odometrii
	ros::Subscriber odometry_get = n.subscribe("/elektron/mobile_base_controller/odom", 10, getOdomNav); // second arg: buffer size (in tutorial = 1000)

	loop_rate = new ros::Rate(HZ);
	
	ROS_INFO("READY TO GET TARGET POSITION");

	ros::spin();
	return 0;
}


// Poniższe obiekty możnaby chyba dać lokalne static,
// ale niech już tak będzie
tf2_ros::TransformListener *local_tf;
tf2_ros::Buffer *local_buffer;
costmap_2d::Costmap2DROS *local_costmap;

int planAndExecute()
{
	isStarted = true;
	bool isGreat, recoBeha = false;
	double lastLinearVel = 0, lastAngularVel = 0, averageSpeed = 0;
	double speeds[XX];
	int status, stopCounter = 0; // licznik zatrzymań
	static base_local_planner::TrajectoryPlannerROS elektron_local_planner;
	//static clear_costmap_recovery::ClearCostmapRecovery ccr;
	//static rotate_recovery::RotateRecovery rr;
	
	for(int i=0; i<XX; i++) speeds[i] = 100;
	
	//inicjalizacja lokalnego planera i jego mapy kosztów
	if(!localCrewInitiated)
	{
		local_buffer = new tf2_ros::Buffer(ros::Duration(10),true);
		local_tf = new tf2_ros::TransformListener(*local_buffer);
		local_costmap = new costmap_2d::Costmap2DROS("local_costmap", *local_buffer);
		(*local_costmap).start();
		elektron_local_planner.initialize("elektron_local_planner", local_buffer, local_costmap);
		/*
		//inicjalizacja clear_costmap_recovery
		tf2_ros::Buffer  ccr_buffer(ros::Duration(10),true);
		tf2_ros::TransformListener  ccr_tf(ccr_buffer);
		ccr.initialize("my_clear_costmap_recovery", &ccr_buffer, costmap, local_costmap);
		
		//inicjalizacja rotate_recovery
		tf2_ros::Buffer  rr_buffer(ros::Duration(10),true);
		tf2_ros::TransformListener  rr_tf(rr_buffer);
		rr.initialize("my_rotate_recovery", &rr_buffer, costmap, local_costmap);
		*/
		localCrewInitiated = true;
	}
	
	//punkt startowy jest pobierany z odometrii

	//punkt docelowy
	target = convertToPoseStamped(targX, targY, targTheta);
	
	// wyliczanie ścieżki przez planer globalny
	(*elektron_global_planner).makePlan(start, target, plan);
	elektron_local_planner.setPlan(plan);
	
	while(ros::ok())
	{
		if(isStarted)
		{
			(*elektron_global_planner).publishPlan(plan);//zeby se zobaczyc sciezke w rviz
			(*local_costmap).updateMap();
			
			if(recoBeha) // Recovery Behaviors
			{
				(*local_costmap).resetLayers();
				rotate360();
				//ccr.runBehavior();
				//rr.runBehavior();
				recoBeha = false;
			}
			
			isGreat = elektron_local_planner.computeVelocityCommands(velocities); //isGreat mowi nam ze robot wyznaczyl jakos dobra sciezke lokalna
			std::cout<<"v_lin: "<<velocities.linear.x<<"  |  v_ang: "<<velocities.angular.z<<std::endl;
			
			if(isGreat == false)
			{
				velocities.linear.x = 0;
				velocities.angular.z = 0;
				velocity_pub.publish(velocities);
				ROS_ERROR("CANT CALCULATE WAY");
				(*local_costmap).resetLayers();
				status = whatToDo(-2);
				if(status != 25) return status;
				else {recoBeha = true; continue;}
			}
			
			if(velocities.linear.x == 0 && velocities.angular.z == 0) // czy już skończył jazde
			{
				if(stopCounter++ == 5) // bo czasem stawał w połowie drogi
				{
					ros::spinOnce(); // czekamy na aktualizację odometrii
					(*local_costmap).resetLayers();
					double dx = start.pose.position.x - targX;
					double dy = start.pose.position.y - targY;
					double howFar = sqrt(dx*dx + dy*dy);
					std::cout<<"Odleglosc od celu: "<<howFar<<std::endl;
					// nie sprawdzamy obrotu
					// bo trzebaby najpierw z quaternionów na RPY
					// a potem różnicę sinusów
					if(howFar < 0.3)
						return 0;
					else
						return -1;
				}
			}
			else
				stopCounter = 0;
			
			averageSpeed = countAverageSpeed(speeds);
			std::cout<<"Average speed: "<<averageSpeed<<std::endl;
			
			if(lastAngularVel*velocities.angular.z < -0.1 || averageSpeed <= 0.1) // jak odwala coś dziwnego
			{
				openGate = true;
				velocities.linear.x = 0;
				velocities.angular.z = 0;
				velocity_pub.publish(velocities);
				velocity_pub.publish(velocities);
				(*local_costmap).resetLayers();
				status = whatToDo(-8);
				if(status != 25) return status;
				else {recoBeha = true; continue;}
			}
			
			velocity_pub.publish(velocities); // Publikujemy otrzymane prędkości
			
			lastLinearVel = velocities.linear.x;
			lastAngularVel = velocities.angular.z;
			pushNextSpeed(speeds, lastLinearVel);
		}
		ros::spinOnce();
		(*loop_rate).sleep();
	}
	return -100;
}


int whatToDo(int status)
{
	std::string whatToDo = "";
	std::cout<<"Co robić?:\n\
				[r] Recalculate path\n\
				[b] recovery Behaviour\n\
				[s] Stop this sh..";
	std::cin>>whatToDo;
	if(whatToDo == "r")
		return 13;
	else if(whatToDo == "b")
		return 25;
	else if(whatToDo == "s")
		return status;
	else
	{
		std::cout<<"Nie ma takiego numeru."<<std::endl;
		return -127;
	}
}


void rotate360()
{
	int i = 0;
	KDL::Rotation r1;
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0.5;
	
	openGate = true;
	ros::spinOnce(); // czekamy na aktualizację odometrii
	
	double yaw = tf2::getYaw(start.pose.orientation);
	std::cout<<"Oto yaw: "<<yaw<<std::endl;
	
	// Obrot odjezdzajacy od 0
	while(i <= 2*HZ)
	{
		velocity_pub.publish(vel_msg);
		ros::spinOnce();
		std::cout<<"Oto kat: "<<tf2::getYaw(start.pose.orientation)<<std::endl;
		i++;
		(*loop_rate).sleep();
	}
	
	// Obrot dojezdzajacy do 360
	while(abs(tf2::getYaw(start.pose.orientation) - yaw) > 0.07)
	{
		velocity_pub.publish(vel_msg);
		ros::spinOnce();
		std::cout<<"Oto kat: "<<tf2::getYaw(start.pose.orientation)<<std::endl;
		(*loop_rate).sleep();
	}
	
	// Zatrzymanie robota
	vel_msg.angular.z = 0;
	while(i <= HZ)
	{
		velocity_pub.publish(vel_msg);
		i++;
		(*loop_rate).sleep();
	}
	
	openGate = false;
}


double countAverageSpeed(double *speeds)
{
	double sum = 0;
	int i = 0;
	while(i < XX)
	{
		//std::cout<<*speeds<<" ";
		if(*speeds == 100) return 1;
		sum += *speeds;
		speeds++;
		i++;
	}
	return sum/XX;
}

void pushNextSpeed(double *speeds, double nextSpeed)
{
	static int i = 0;
	//std::cout<<i<<std::endl;
	if(i == XX) i = 0;
	speeds[i++] = nextSpeed;
}

geometry_msgs::PoseStamped convertToPoseStamped(double x, double y, double theta)
{
	geometry_msgs::PoseStamped result;

	result.header.frame_id = "map";
	result.header.stamp = ros::Time(0);

	result.pose.position.x = x;
	result.pose.position.y = y;
	//msg.pose.position.z = 0;

	KDL::Rotation r1 = KDL::Rotation::RPY(0,0,theta);
	r1.GetQuaternion
	(
		result.pose.orientation.x,
		result.pose.orientation.y,
		result.pose.orientation.z,
		result.pose.orientation.w
	);

	return result;
}
