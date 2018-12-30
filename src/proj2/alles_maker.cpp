#include "ros/ros.h"
#include "stero_mobile_init/Positioning.h" // mobile_STERO-18Z-Fijalkowski-Stolarz/ LUB stero_mobile_init LUB nic
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <cmath>
#include "costmap_2d/costmap_2d_ros.h"
#include "tf2_ros/buffer.h"
#include "stdbool.h"
#include "tf2_ros/transform_listener.h"
#include <global_planner/planner_core.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <vector>

float x, y, theta;
double qaternion[4];

//zaplanowana sciezka jazdy
std::vector<geometry_msgs::PoseStamped> plan;
//punkt startowy -  bedzie pobierane z odometrii to jest wersja prbna
geometry_msgs::PoseStamped start;
//punkt koncowy
geometry_msgs::PoseStamped target;

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

	tf2_ros::Buffer buffer(ros::Duration(10),true);
	tf2_ros::TransformListener tf(buffer);	
	costmap_2d::Costmap2DROS costmap("costmap", buffer);
	global_planner::GlobalPlanner elektron_global_planner("global_planner",costmap.getCostmap(),"costmap");
	
	ros::ServiceServer service = n.advertiseService("set_position", reqHandler);

	ROS_INFO("READY TO GET TARGET POSITION");

	ros::Rate loop_rate(0.5);
	while(ros::ok()){

		elektron_global_planner.makePlan(start,target,plan);
		elektron_global_planner.publishPlan(plan);//zeby se zobaczyc sciezke w rviz
		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
