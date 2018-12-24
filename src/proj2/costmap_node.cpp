#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <cmath>
#include "tf/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "tf2_ros/buffer.h"
#include "stdbool.h"

int main(int argc, char** argv){

	ros::init(argc,argv,"publish_multiple_goals");
	ros::NodeHandle n;

	//tf::TransformListener tf(ros::Duration(10));
	tf2_ros::Buffer buffer(ros::Duration(10),true);
	buffer.setUsingDedicatedThread(true);//bo tf jest publikowany przez inny watek
	costmap_2d::Costmap2DROS costmap("costmap_node", buffer);
	std::cout<<"IT WORKS!"<<std::endl;

    return 0;
}
