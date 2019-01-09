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
#include <ctime>

#define HZ 10
#define XX 200
#define avgSpeed 0.05

/* Funkcja konwertująca pozycję (x,y,theta) na PoseStamped */
geometry_msgs::PoseStamped convertToPoseStamped(double, double, double);

int planAndExecute();
void rotate360();



/* Współrzędne celu */
double targX, targY, targTheta;

//flaga czy planery zrobiły plany
bool isPlanComputed = false;

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
	
	
	status = 0;
	status = planAndExecute();
	ROS_INFO("Got response: [%d]", status);
	
	
	isStarted = false;
	res.status = status;
	ROS_INFO("sending back response: [%d]", res.status);
	
	return true;
}


/* Odbiera aktualną pozycję robota i wstawia do zmiennej "start".
 * Robi to tylko wtedy, gdy robot nie wykonuje zadania */
void getOdomNav(const nav_msgs::Odometry::ConstPtr&  msg)
{
		start.header.frame_id = "map";
		start.header.stamp = ros::Time(0);

		start.pose.position.x = msg->pose.pose.position.x;
		start.pose.position.y = msg->pose.pose.position.y;
		
		start.pose.orientation.x = msg->pose.pose.orientation.x;
		start.pose.orientation.y = msg->pose.pose.orientation.y;
		start.pose.orientation.z = msg->pose.pose.orientation.z;
		start.pose.orientation.w = msg->pose.pose.orientation.w;
	
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
	int failCounter = 0; //licznik porazek
	int stopCounter = 0; //licznik zatrzyman
	bool planMade = true;
	bool isGreat = true;
	bool isStoppedForNoReason = false;
	double dx, dy, howFar;
	static base_local_planner::TrajectoryPlannerROS elektron_local_planner;
	std::clock_t beginn;
	std::clock_t end;
	
	//inicjalizacja lokalnego planera i jego mapy kosztów
	if(!localCrewInitiated)
	{
		local_buffer = new tf2_ros::Buffer(ros::Duration(10),true);
		local_tf = new tf2_ros::TransformListener(*local_buffer);
		local_costmap = new costmap_2d::Costmap2DROS("local_costmap", *local_buffer);
		(*local_costmap).start();
		elektron_local_planner.initialize("elektron_local_planner", local_buffer, local_costmap);
		
		localCrewInitiated = true;
	}
	
	//punkt startowy jest pobierany z odometrii

	//punkt docelowy
	target = convertToPoseStamped(targX, targY, targTheta);
	
	// wyliczanie ścieżki przez planer globalny
	planMade = (*elektron_global_planner).makePlan(start, target, plan);
	if(!planMade) return -1;
	elektron_local_planner.setPlan(plan);
	
	while(ros::ok())
	{
		if(isStarted)
		{
			(*elektron_global_planner).publishPlan(plan);//zeby se zobaczyc sciezke w rviz
			(*local_costmap).updateMap();
			
			isGreat = elektron_local_planner.computeVelocityCommands(velocities); //isGreat mowi nam ze robot wyznaczyl jakos dobra sciezke lokalna
			//std::cout<<"v_lin: "<<velocities.linear.x<<"  |  v_ang: "<<velocities.angular.z<<std::endl;
			
			if(velocities.linear.x == 0 && velocities.angular.z == 0 && isGreat) // jak się zatrzymał i wyznaczy dobre predkosci
			{
				if(stopCounter++ > 10){
					ros::spinOnce(); // czekamy na aktualizację odometrii
					velocity_pub.publish(velocities);
					(*local_costmap).resetLayers();
					dx = start.pose.position.x - targX;
					dy = start.pose.position.y - targY;
					howFar = sqrt(dx*dx + dy*dy);
					if(howFar < 0.3) return 0;
					else isStoppedForNoReason = true;
				}
			}
			else stopCounter = 0;

			
			if(isGreat && !isStoppedForNoReason) {
				end = clock();
				if(double(end-beginn)>60*CLOCKS_PER_SEC) failCounter=0;
			}
			else
			{
				velocities.linear.x = 0;
				velocities.angular.z = 0;
				velocity_pub.publish(velocities);
				if(failCounter == 0){
					ROS_INFO("RECOVERY BEHAVIOUR 1: CLEAR AND ROTATE");
					(*local_costmap).resetLayers(); 
					rotate360();
					ROS_INFO("RECOVERY BEHAVIOUR 1 DONE: CLEAR AND ROTATE");
					failCounter=1;
					beginn = clock();
				}
				else if(failCounter==1){
					ROS_INFO("RECOVERY BEHAVIOUR 2: MAKE NEW PLAN");
					(*elektron_global_planner).makePlan(start, target, plan);
					elektron_local_planner.setPlan(plan);
					ROS_INFO("RECOVERY BEHAVIOUR 2 DONE: MAKE NEW PLAN");
					failCounter=2;
					beginn = clock();
				}
				else if(failCounter==2){
					ROS_ERROR("FAILED TO REACH GOAL");
					return -1;				
				}
				isStoppedForNoReason = false;
				
				
			}
			
		
			velocity_pub.publish(velocities); // Publikujemy otrzymane prędkości
		}
		ros::spinOnce();
		(*loop_rate).sleep();
	}
	return -100;
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
	
	double yaw = tf2::getYaw(start.pose.orientation);
	
	// Obrot odjezdzajacy od 0
	while(i <= HZ)
	{
		velocity_pub.publish(vel_msg);
		ros::spinOnce();
		i++;
		(*loop_rate).sleep();
	}
	
	// Obrot dojezdzajacy do 360
	while(abs(tf2::getYaw(start.pose.orientation) - yaw) > 0.07)
	{
		velocity_pub.publish(vel_msg);
		ros::spinOnce();
		(*loop_rate).sleep();
	}
}

//-----------------------------------------------------------------------^^^^^

geometry_msgs::PoseStamped convertToPoseStamped(double x, double y, double theta)
{
	geometry_msgs::PoseStamped result;

	result.header.frame_id = "map";
	result.header.stamp = ros::Time(0);

	result.pose.position.x = x;
	result.pose.position.y = y;

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
