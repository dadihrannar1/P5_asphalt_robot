// Documentation for nodes and services: https://cyberbotics.com/doc/reference/nodes-and-api-functions

// Name of robot: fivebarTrailer_2841
/* Nodes:
    1: ros::service /fivebarTrailer/Motor_L/set_position <position>
    2: ros::service /fivebarTrailer/Motor_R/set_position <position>
    3: ros::service /fivebarTrailer/PosR/enable <sampling rate>
        3.1: ros::topic /fivebarTrailer/PosR/value
    4: ros::service /fivebarTrailer/PosL/enable <sampling rate>
        4.1: ros::topic /fivebarTrailer/PosL/value
*/ 

#include "ros/ros.h"

#include <webots_ros/Int32Stamped.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/robot_get_device_list.h>

#include <std_msgs/String.h>

#include <signal.h>
#include <stdio.h>

#define TIME_STEP 32

static double lposition = 0;
static double rposition = 0;

ros::ServiceClient leftMotorClient;
webots_ros::set_float leftMotorSrv;

ros::ServiceClient rightMotorClient;
webots_ros::set_float rightMotorSrv;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;


int main(int argc, char **argv) {
  // create a node named 'test_movement' on ROS network
  ros::init(argc, argv, "test_movement", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::Rate r(1000);

  // Wait for the `ros` controller.
  ros::service::waitForService("/fivebarTrailer/robot/time_step");
  ros::spinOnce();

  // These are all the services found in the default ros controller
  leftMotorClient = n.serviceClient<webots_ros::set_float>("/fivebarTrailer/MotorL/set_position");
  rightMotorClient = n.serviceClient<webots_ros::set_float>("/fivebarTrailer/MotorR/set_position");
  timeStepClient = n.serviceClient<webots_ros::set_int>("/fivebarTrailer/robot/time_step");

  timeStepSrv.request.value = TIME_STEP;

  // main loop
    while (ros::ok()) {
      //ros::spinOnce();
      for (int i = 1; i <= 5; ++i) {
        lposition = lposition+i*0.2;
        ROS_INFO("Setting left motor to position: %f",lposition);
        leftMotorSrv.request.value = lposition;
        leftMotorClient.call(leftMotorSrv);
        ros::spinOnce();
        r.sleep();
      }

      for (int i = 1; i <= 5; ++i) {
        lposition = lposition-i*0.2;
        ROS_INFO("Setting left motor to position: %f",lposition);
        leftMotorSrv.request.value = lposition;
        leftMotorClient.call(leftMotorSrv);
        ros::spinOnce();
        r.sleep();
      }
      for (int i = 1; i <= 5; ++i) {
        rposition = rposition-i*0.2;
        ROS_INFO("Setting right motor to position: %f",rposition);
        rightMotorSrv.request.value = rposition;
        rightMotorClient.call(rightMotorSrv);
        ros::spinOnce();
        r.sleep();
      }
      for (int i = 1; i <= 5; ++i) {
        rposition = rposition+i*0.2;
        ROS_INFO("Setting right motor to position: %f",rposition);
        rightMotorSrv.request.value = rposition;
        rightMotorClient.call(rightMotorSrv);
        ros::spinOnce();
        r.sleep();
      }
    }
}