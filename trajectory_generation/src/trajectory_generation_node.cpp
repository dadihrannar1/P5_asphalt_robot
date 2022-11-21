#include <armadillo>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_generation/vision_out.h>


void coordinate_callback(const trajectory_generation::vision_out::ConstPtr& coordinates){
  ROS_INFO("x coordinate: [%d]", coordinates-> x);
  ROS_INFO("y coordinate: [%d]", coordinates-> y);
  //ROS_INFO("starting a crack: [%s]", msg-> crack);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("crack_coordinates", 1000, coordinate_callback);

  ros::spin();
  return 0;
}