
#include "controllers/vision_out.h"
#include "ros/ros.h"

void chatterIMU(const controllers::vision_out::ConstPtr& msg)
{
  ROS_INFO("x coordinate: [%d]", msg-> x);
  ROS_INFO("y coordinate: [%d]", msg-> y);
  //ROS_INFO("starting a crack: [%s]", msg-> crack);

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("vision_publisher", 1000, chatterIMU);

  ros::spin();

  return 0;
}