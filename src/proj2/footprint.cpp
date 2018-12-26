#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Polygon.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "footprint");

  ros::NodeHandle n;

  ros::Publisher polygon_pub = n.advertise<geometry_msgs::Polygon>("costmap/footprint", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Polygon polygon;

    polygon.points.reserve(5);

    geometry_msgs::Point32 p1;
    p1.x = -0.325;
    p1.y = -0.325;
    polygon.points.push_back(p1);

    geometry_msgs::Point32 p2;
    p2.x = -0.325;
    p2.y = 0.325;
    polygon.points.push_back(p2);

    geometry_msgs::Point32 p3;
    p3.x = 0.325;
    p3.y = 0.325;
    polygon.points.push_back(p3);

    geometry_msgs::Point32 p4;
    p4.x = 0.46;
    p4.y = 0.0;
    polygon.points.push_back(p4);

    geometry_msgs::Point32 p5;
    p5.x = 0.325;
    p5.y = -0.325;
    polygon.points.push_back(p5);

    polygon_pub.publish(polygon);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
