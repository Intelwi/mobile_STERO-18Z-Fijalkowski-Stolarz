#include "ros/ros.h"
#include "stero_mobile_init/Positioning.h" // mobile_STERO-18Z-Fijalkowski-Stolarz/ LUB stero_mobile_init LUB nic

float x, y, theta;


int theFunction()
{
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

	ros::ServiceServer service = n.advertiseService("set_position", reqHandler);
	ROS_INFO("READY TO GET TARGET POSITION");
	ros::spin();

	return 0;
}
