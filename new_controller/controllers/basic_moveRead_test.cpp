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
#include <webots_ros/Float64Stamped.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/robot_get_device_list.h>

#include <std_msgs/String.h>

#include <signal.h>
#include <stdio.h>

#define TIME_STEP 32

// standard callback function for subscribers
void chatterCallbackL(const webots_ros::Float64Stamped::ConstPtr &value){
  ROS_INFO("PosL sensor sent value %f (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
}
void chatterCallbackR(const webots_ros::Float64Stamped::ConstPtr &value){
  ROS_INFO("PosR sensor sent value %f (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
}


int main(int argc, char **argv) {
  // set variables
  static double lposition = 0;
  static double rposition = 0;
  // Motor services
  ros::ServiceClient leftMotorClient;
  webots_ros::set_float leftMotorSrv;

  ros::ServiceClient rightMotorClient;
  webots_ros::set_float rightMotorSrv;

  // Encoder topics and services
  ros::ServiceClient encoderClient;
  ros::Subscriber PosL;
  ros::Subscriber PosR;

  // Time step for motor control
  ros::ServiceClient timeStepClient;
  webots_ros::set_int timeStepSrv;


  // create a node named 'test_movement' on ROS network
  ros::init(argc, argv, "test_movement", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::Rate r(100);

  // Wait for the `ros` controller.
  ros::service::waitForService("/fivebarTrailer/robot/time_step");
  ros::spinOnce();

  // Time step
  timeStepSrv.request.value = TIME_STEP;

  // These are all the services found in the default ros controller
  leftMotorClient = n.serviceClient<webots_ros::set_float>("/fivebarTrailer/MotorL/set_position");
  rightMotorClient = n.serviceClient<webots_ros::set_float>("/fivebarTrailer/MotorR/set_position");
  timeStepClient = n.serviceClient<webots_ros::set_int>("/fivebarTrailer/robot/time_step");

  // Encoders have to be enabled before their topics are available
  encoderClient = n.serviceClient<webots_ros::set_int>("/fivebarTrailer/PosL/enable");
  encoderClient.call(timeStepSrv);
  encoderClient = n.serviceClient<webots_ros::set_int>("/fivebarTrailer/PosR/enable");
  encoderClient.call(timeStepSrv);
  // Set function to read out encoder postion
  PosL = n.subscribe("/fivebarTrailer/PosL/value", 1, chatterCallbackL);
  PosR = n.subscribe("/fivebarTrailer/PosR/value", 1, chatterCallbackR);

  // main loop
    while (ros::ok()) {
      // Run through random motor positions
      
      for (int i = 1; i <= 100; ++i) {
        lposition = i*0.02;
        ROS_INFO("%d, Setting left motor to position: %f", i,lposition);
        leftMotorSrv.request.value = lposition;
        leftMotorClient.call(leftMotorSrv);
        ros::spinOnce();
        r.sleep();
      }

      for (int i = 100; i >= 0; --i) {
        lposition = i*0.02;
        ROS_INFO("%d, Setting left motor to position: %f", i,lposition);
        leftMotorSrv.request.value = lposition;
        leftMotorClient.call(leftMotorSrv);
        ros::spinOnce();
        r.sleep();
      }
      for (int i = 1; i <= 100; ++i) {
        rposition = i*0.02;
        ROS_INFO("%d, Setting right motor to position: %f", i, rposition);
        rightMotorSrv.request.value = rposition;
        rightMotorClient.call(rightMotorSrv);
        ros::spinOnce();
        r.sleep();
      }
      for (int i = 100; i >= 0; --i) {
        rposition = i*0.02;
        ROS_INFO("%d, Setting right motor to position: %f", i, rposition);
        rightMotorSrv.request.value = rposition;
        rightMotorClient.call(rightMotorSrv);
        ros::spinOnce();
        r.sleep();
      }
    }
}