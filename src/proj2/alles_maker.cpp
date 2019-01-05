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
#include <global_planner/planner_core.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <vector>

/* Funkcja konwertująca pozycję (x,y,theta) na PoseStamped */
geometry_msgs::PoseStamped convertToPoseStamped(double, double, double);

int planExecutor();

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
	targX = req.position.x;
	targY = req.position.y;
	targTheta = req.position.theta;
	
	ROS_INFO("I got co-ordinates: (%f, %f) and orientation: %f rad", targX, targY, targTheta);

	res.status = planExecutor();
	ROS_INFO("sending back response: [%d]", res.status);
	
	return true;
}


/* Odbiera aktualną pozycję robota i wstawia do zmiennej "start".
 * Robi to tylko wtedy, gdy robot nie wykonuje zadania */
void getOdomNav(const nav_msgs::Odometry::ConstPtr&  msg)
{
	if(!isStarted)
	{
		std::cout<<"elo from getOdomNav"<<std::endl;
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
	costmap_2d::Costmap2DROS  costmap("costmap", buffer);
	costmap.start();
	elektron_global_planner = new global_planner::GlobalPlanner("elektron_global_planner",costmap.getCostmap(),"map");

	//inicjalizacja lokalnego planera i jego mapy kosztów
	/*local_buffer = new tf2_ros::Buffer(ros::Duration(10),true);
	local_tf = new tf2_ros::TransformListener(*local_buffer);
	local_costmap = new costmap_2d::Costmap2DROS("local_costmap", *local_buffer);
	(*local_costmap).start();
	base_local_planner::TrajectoryPlannerROS elektron_local_planner;
	elektron_local_planner.initialize("elektron_local_planner",local_buffer,local_costmap);*/
	
	//inicjalizacja servisu odbierającego położenie docelowe
	ros::ServiceServer service = n.advertiseService("set_position", reqHandler);
	//inicjalizacja publikowania prędkości do robota
	velocity_pub = n.advertise<geometry_msgs::Twist>("/mux_vel_raw/cmd_vel", 500);
	//inicjalizacja odbierania aktualnej pozycji z odometrii
	ros::Subscriber odometry_get = n.subscribe("/elektron/mobile_base_controller/odom", 10, getOdomNav); // second arg: buffer size (in tutorial = 1000)

	loop_rate = new ros::Rate(10);
	
	ROS_INFO("READY TO GET TARGET POSITION");

	/*while(ros::ok())
	{
		if(isStarted)
		{
			if(!isPlanComputed)
			{
				(*elektron_global_planner).makePlan(start, target, plan);
				elektron_local_planner.setPlan(plan);
				isPlanComputed = true;
			}
			(*elektron_global_planner).publishPlan(plan);//zeby se zobaczyc sciezke w rviz
			//isTrajectoryComputed = elektron_local_planner.checkTrajectory(0.01, 0.01, 0.001, true);
			(*local_costmap).updateMap();
			elektron_local_planner.computeVelocityCommands(velocities);
			
			std::cout<<velocities<<std::endl;
			velocity_pub.publish(velocities);
			
			/*if(velocities.linear.x == 0 && velocities.angular.z == 0)
			{
				isStarted = false;
				isPlanComputed = false;
			}
		}
		ros::spinOnce();
		(*loop_rate).sleep();
	}*/
	ros::spin();
	return 0;
}


// Poniższe obiekty możnaby chyba dać lokalne static,
// ale niech już tak będzie
tf2_ros::TransformListener *local_tf;
tf2_ros::Buffer *local_buffer;
costmap_2d::Costmap2DROS *local_costmap;

int planExecutor()
{
	int stopCounter = 0; // licznik zatrzymań
	static base_local_planner::TrajectoryPlannerROS elektron_local_planner;
	
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

	isStarted = true;
	
	while(ros::ok())
	{
		if(isStarted)
		{
			if(!isPlanComputed)
			{
				(*elektron_global_planner).makePlan(start, target, plan);
				elektron_local_planner.setPlan(plan);
				isPlanComputed = true;
			}
			(*elektron_global_planner).publishPlan(plan);//zeby se zobaczyc sciezke w rviz
			//isTrajectoryComputed = elektron_local_planner.checkTrajectory(0.01, 0.01, 0.001, true);
			(*local_costmap).updateMap();
			elektron_local_planner.computeVelocityCommands(velocities);
			
			std::cout<<velocities<<std::endl;
			velocity_pub.publish(velocities);
			
			if(velocities.linear.x == 0 && velocities.angular.z == 0)//czy już skończył jazde
			{
				if(stopCounter++ == 5) // bo czasem stawał w połowie drogi
				{
					isStarted = false;
					isPlanComputed = false;
					ros::spinOnce(); // czekamy na aktualizację odometrii
					double dx = start.pose.position.x - targX;
					double dy = start.pose.position.y - targY;
					double howFar = sqrt(dx*dx + dy*dy);
					// nie sprawdzamy obrotu
					// bo trzebaby najpierw z quaternionów na RPY
					// a potem różnicę sinusów
					if(howFar < 0.2)
						return 0;
					else
						return -1;
				}
			}
			else
				stopCounter = 0;
		}
		ros::spinOnce();
		(*loop_rate).sleep();
	}
	return 0;
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
