#include "ros/ros.h"
#include "webots_ros/get_float.h"
#include "std_msgs/Float32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_time_client");
  ros::NodeHandle n;

  // Create a client for the "/fivebarTrailer/robot/get_time" service
  ros::service::waitForService("/fivebarTrailer/robot/get_time");
  ros::Duration(0.1).sleep();
  ros::ServiceClient client = n.serviceClient<webots_ros::get_float>("/fivebarTrailer/robot/get_time");

  // Create a publisher to the "webots_time" topic with message type std_msgs::Float32
  ros::Publisher time_pub = n.advertise<std_msgs::Float32>("webots_time", 1000);

  ros::Rate loop_rate(5); // 10 Hz

  while (ros::ok())
  {
    // Call the service and store the response in a variable
    webots_ros::get_float srv;
    if (client.call(srv))
    {
      // Publish the response from the service to the "webots_time" topic
      std_msgs::Float32 msg;
      msg.data = srv.response.value;
      time_pub.publish(msg);
    }
    else
    {
      ROS_ERROR("Failed to call service /fivebarTrailer/robot/get_time");
    }

    loop_rate.sleep();
  }

  return 0;
}