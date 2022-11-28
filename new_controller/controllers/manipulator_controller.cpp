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

// Robot zero positions
/*
  MotorR
        Theta = -p+1.8675
  MotorL
        Theta = p-2.33874
*/


#include "ros/ros.h"

#include <webots_ros/Int32Stamped.h>
#include <webots_ros/Float64Stamped.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/robot_get_device_list.h>

#include <new_controller/set_pos.h>
#include <new_controller/motor_pos.h>

#include <std_msgs/String.h>
#include <cmath>

#include <signal.h>
#include <stdio.h>

// Time step for the webots simulation
#define TIME_STEP 32

ros::ServiceClient leftMotorClient;
ros::ServiceClient rightMotorClient;

class robotController{
  public:

  double theta_1;
  double theta_2;
  double simTime;

  // standard callback function for subscribers
  void posCallbackL(const webots_ros::Float64Stamped::ConstPtr &value){
    theta_1 = value->data-2.33874;
    simTime = value->header.stamp.sec;
    //ROS_INFO("PosL sensor sent value %f (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
  }
  void posCallbackR(const webots_ros::Float64Stamped::ConstPtr &value){
    theta_2 = -value->data+1.8675;
  }
};

// Function for setting the motors in a desired position
bool setPos(new_controller::set_pos::Request &msg,
            new_controller::set_pos::Response &nah){
  double posL = msg.theta_1+2.33874;
  double posR = msg.theta_2-1.8675;

  // define and publish
  webots_ros::set_float leftMotorSrv; webots_ros::set_float rightMotorSrv;
  leftMotorSrv.request.value = posL;
  rightMotorSrv.request.value = posR;
  leftMotorClient.call(leftMotorSrv);
  rightMotorClient.call(rightMotorSrv);

  return true;
            }

int main(int argc, char **argv) {
  // create a node named 'test_movement' on ROS network
  ros::init(argc, argv, "manipulator_controller");
  ros::NodeHandle n;

  // Wait for the simulation controller.
  ros::service::waitForService("/fivebarTrailer/robot/time_step");
  ros::spinOnce();

  // Time step
  webots_ros::set_int timeStepSrv;
  timeStepSrv.request.value = TIME_STEP;

  // Encoders have to be enabled before their topics are available
  ros::ServiceClient encoderClient = n.serviceClient<webots_ros::set_int>("/fivebarTrailer/PosL/enable");
  encoderClient.call(timeStepSrv);
  encoderClient = n.serviceClient<webots_ros::set_int>("/fivebarTrailer/PosR/enable");
  encoderClient.call(timeStepSrv);

  // Clients for the set motor position
  leftMotorClient = n.serviceClient<webots_ros::set_float>("/fivebarTrailer/MotorL/set_position");
  rightMotorClient = n.serviceClient<webots_ros::set_float>("/fivebarTrailer/MotorR/set_position");

  // Set Service
  ros::ServiceServer motorService = n.advertiseService("motorSetPos", setPos);
  //new_controller::set_pos motorSrv;

  // Set function to read out encoder postion
  robotController rc;
  ros::Subscriber posL = n.subscribe("/fivebarTrailer/PosL/value", 1, &robotController::posCallbackL, &rc);
  ros::Subscriber posR = n.subscribe("/fivebarTrailer/PosR/value", 1, &robotController::posCallbackR, &rc);

  // Set publisher
  ros::Publisher encoderPub = n.advertise<new_controller::motor_pos>("encoderData", 10);
  // message for the encoder
  new_controller::motor_pos encoderMsg;
  
  // main loop
    while (ros::ok()) {
      encoderMsg.theta_1 = rc.theta_1;
      encoderMsg.theta_2 = rc.theta_2;
      encoderMsg.simuTime = rc.simTime;
      encoderPub.publish(encoderMsg);
      ros::spinOnce();
    }
}