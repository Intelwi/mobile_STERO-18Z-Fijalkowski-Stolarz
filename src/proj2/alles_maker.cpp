#include "ros/ros.h"
#include "stero_mobile_init/Positioning.h" // mobile_STERO-18Z-Fijalkowski-Stolarz/ LUB stero_mobile_init LUB nic
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <cmath>
#include "costmap_2d/costmap_2d_ros.h"
#include "tf2_ros/buffer.h"
#include "stdbool.h"
#include "tf2_ros/transform_listener.h"
#include <global_planner/planner_core.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <vector>

float x, y, theta;
double qaternion[4];

//flaga czy robot wyznaczył lokalną trajektorię
bool isTrajectoryComputed = false;

//flaga czy rozpoczeto planowanie
bool isStarted = false;

//zaplanowana sciezka jazdy
std::vector<geometry_msgs::PoseStamped> plan;

//punkt startowy -  bedzie pobierane z odometrii to jest wersja próbna
geometry_msgs::PoseStamped start;

//punkt koncowy
geometry_msgs::PoseStamped target;

//wyliczone predkosci przez planer lokalny
geometry_msgs::Twist velocities;

int theFunction()
{


	KDL::Rotation r1=KDL::Rotation::RPY(0,0,theta);
	r1.GetQuaternion(qaternion[0],qaternion[1],qaternion[2],qaternion[3]);



	//start bedzie pobierane z odometrii to jest wersja prbna
	start.header.frame_id="costmap";
	start.header.stamp = ros::Time(0);

	start.pose.position.x = 0;
	start.pose.position.y = 0;
	//msg.pose.position.z = 0;

	start.pose.orientation.x = 0;
	start.pose.orientation.y = 0;
	start.pose.orientation.z = 0;
	start.pose.orientation.w = 1;



	//punkt docelowy
	target.header.frame_id="costmap";
	target.header.stamp = ros::Time(0);

	target.pose.position.x = x;
	target.pose.position.y = y;
	//msg.pose.position.z = 0;

	target.pose.orientation.x = qaternion[0];
	target.pose.orientation.y = qaternion[1];
	target.pose.orientation.z = qaternion[2];
	target.pose.orientation.w = qaternion[3];
	
	isStarted = true;

	return 13;
}


bool reqHandler(stero_mobile_init::Positioning::Request  &req,
                stero_mobile_init::Positioning::Response  &res)
{
	x = req.position.x;
	y = req.position.y;
	theta = req.position.theta;
	
	ROS_INFO("I got co-ordinates: (%f, %f) and orientation: %f rad", x, y, theta);

	res.status = theFunction();
	ROS_INFO("sending back response: [%d]", res.status);
	
	return true;
}


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "alles_server");
	ros::NodeHandle n;
	
	//inicjalizacja globalnego planera i jego mapy kosztów
	tf2_ros::Buffer buffer(ros::Duration(10),true);
	tf2_ros::TransformListener tf(buffer);	
	costmap_2d::Costmap2DROS costmap("costmap", buffer);
	global_planner::GlobalPlanner elektron_global_planner("global_planner",costmap.getCostmap(),"costmap");

	//inicjalizacja lokalnego planera i jego mapy kosztów
	tf2_ros::Buffer local_buffer(ros::Duration(10),true);
	tf2_ros::TransformListener local_tf(local_buffer);	
	costmap_2d::Costmap2DROS local_costmap("local_costmap", local_buffer);
	base_local_planner::TrajectoryPlannerROS elektron_local_planner;
	elektron_local_planner.initialize("local_planner",&local_buffer,&local_costmap);
	
	ros::ServiceServer service = n.advertiseService("set_position", reqHandler);

	ROS_INFO("READY TO GET TARGET POSITION");

	ros::Rate loop_rate(2);

	bool isPlanNotComputed = true;

	while(ros::ok()){
		if(isStarted)
		{
			if(isPlanNotComputed)
			{
				elektron_global_planner.makePlan(start,target,plan);
				elektron_local_planner.setPlan(plan);
				isPlanNotComputed = false;
			}
			
			elektron_global_planner.publishPlan(plan);//zeby se zobaczyc sciezke w rviz
			isTrajectoryComputed = elektron_local_planner.checkTrajectory(0.01, 0.01, 0.001,true);
			elektron_local_planner.computeVelocityCommands(velocities);//z jakiegoś powodu ten kawałek kodu wywala [ERROR] [1546524878.017346415, 155.898000000]: No Transform available Error: "costmap" passed to lookupTransform argument source_frame does not exist. 

		}
		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
