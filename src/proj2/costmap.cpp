#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <cmath>
#include "costmap_2d/costmap_2d_ros.h"
#include "tf2_ros/buffer.h"
#include "stdbool.h"

#include "move_base/move_base.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char** argv){

	std::cout<<"IT WORKS !"<<std::endl;
	ros::init(argc,argv,"publish_costmap");
	ros::NodeHandle n;
	std::cout<<"IT WORKS 0!"<<std::endl;
	tf2_ros::Buffer buffer(ros::Duration(10),true);
	tf2_ros::TransformListener tf(buffer, n);
	std::cout<<"IT WORKS 1!"<<std::endl;
	buffer.setUsingDedicatedThread(true);//bo tf jest publikowany przez inny watek
	std::cout<<"IT WORKS 2!"<<std::endl;
	costmap_2d::Costmap2DROS costmap("costmap", buffer);
	costmap.start();
	costmap.updateMap();


	ros::spin();


	return 0;
}
