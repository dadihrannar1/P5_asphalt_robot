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
#include <istream>

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

// Robot lengths
int L0 = 176;
int L1 = 573;
int L2 = 714;

// Global variables for calculatred angles
float angl1;
float angl2;

// Function declarations
void invKin(float xPos, float yPos);
bool setPos(new_controller::set_pos::Request &msg,new_controller::set_pos::Response &nah);
bool setMotorPos(new_controller::set_pos::Request &msg,new_controller::set_pos::Response &nah);

class robotController{
  public:

  double theta_1;
  double theta_2;
  double simTime;

  // standard callback function for subscribers
  void posCallbackL(const webots_ros::Float64Stamped::ConstPtr &value){
    theta_1 = value->data+(M_PI-0.3364441);
    simTime = value->header.stamp.sec;
    //ROS_INFO("PosL sensor sent value %f (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
  }
  void posCallbackR(const webots_ros::Float64Stamped::ConstPtr &value){
    theta_2 = -value->data-(M_PI-0.2931572);
  }
};



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
  ros::ServiceServer motorService = n.advertiseService("motorSetPos", setMotorPos);
  ros::ServiceServer posService = n.advertiseService("manipulatorSetPos", setPos);
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

// Inverse kinematics
void invKin(float xPos, float yPos)
{
  // for an explenation look in the report
  float alpha1 = atan2(yPos,xPos);
  float alpha2 = atan2(yPos,L0-xPos);
  float D1 = sqrt(pow(xPos,2)+pow(yPos,2));
  float D2 = sqrt(pow(L0-xPos,2)+pow(yPos,2));
  float beta1 = acos((pow(L1,2)+pow(D1,2)-pow(L2,2))/(2*L1*D1));
  float beta2 = acos((pow(L1,2)+pow(D2,2)-pow(L2,2))/(2*L1*D2));

  float Theta1 = alpha1+beta1;
  float Theta2 = alpha2+beta2;

  angl1 = Theta1;
  angl2 = Theta2;
}


// Function for setting the ee in a desired position
bool setPos(new_controller::set_pos::Request &msg,
            new_controller::set_pos::Response &nah){
  // input
  float posx = msg.x;
  float posy = msg.y;
  new_controller::set_pos posMsg;

  invKin(posx, posy); // get the motor positions
  // package it and send it towards the motors
  posMsg.request.theta_1 = angl1;
  posMsg.request.theta_2 = angl2;
  setMotorPos(posMsg.request,posMsg.response);

  return true;
}

// Function for setting the motors in a desired position
bool setMotorPos(new_controller::set_pos::Request &msg,
                 new_controller::set_pos::Response &nah){
  //ROS_INFO("theta1 %f theta2 %f" ,msg.theta_1, msg.theta_2);

  double posL = msg.theta_1-(M_PI-0.3364441);
  double posR = -msg.theta_2+(M_PI-0.2931572);

  // incase a position is called that requires the robot to spin wrap it back down to values between [-pi,pi]
  // fx. theta1 = 500 deg, returns 140 degs
  posL = fmod(posL+M_PI, 2*M_PI);
  posR = fmod(posR + M_PI, 2*M_PI);
  if(posL < 0)
  {
    posL += 2*M_PI;
  }
  if(posR < 0)
  {
    posR += 2*M_PI;
  }
  posL -= M_PI;
  posR -= M_PI;
  
  // define and publish
  webots_ros::set_float leftMotorSrv; webots_ros::set_float rightMotorSrv;
  leftMotorSrv.request.value = posL;
  rightMotorSrv.request.value = posR;
  //ROS_INFO("posL %f posR %f", posL, posR);
  leftMotorClient.call(leftMotorSrv);
  rightMotorClient.call(rightMotorSrv);
  
  return true;
  }
