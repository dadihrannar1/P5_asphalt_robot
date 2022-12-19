// Documentation for nodes and services: https://cyberbotics.com/doc/reference/nodes-and-api-functions

// Name of robot: fivebarTrailer
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
#include <webots_ros/get_float.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/robot_get_device_list.h>

#include <new_controller/set_pos.h>
#include <new_controller/motor_pos.h>
#include <new_controller/trajectory_polynomial.h>

#include <std_msgs/String.h>
#include <cmath>

#include <signal.h>
#include <stdio.h>

// Time step for the webots simulation
#define TIME_STEP 4

ros::ServiceClient leftMotorClient;
ros::ServiceClient rightMotorClient;
ros::ServiceClient time_client;


// Global variables for calculatred angles
float angl1;
float angl2;

// Function declarations
void invKin(float xPos, float yPos);
bool setPos(new_controller::set_pos::Request &posmsg,new_controller::set_pos::Response &nt);
bool setMotorPos(new_controller::set_pos::Request &motormsg,new_controller::set_pos::Response &nah);


class robotController{
  public:

  double theta_1;
  double theta_2;
  double simTime;
  float xpoly[4] = {};
  float ypoly[4] = {};
  float time_to_finish = 0;
  bool new_polynomial = false;

  // standard callback function for subscribers
  void posCallbackL(const webots_ros::Float64Stamped::ConstPtr &value){
    theta_1 = value->data+(M_PI-0.3364441);
    simTime = value->header.stamp.sec;
    //ROS_INFO("PosL sensor sent value %f (time: %d:%d).", value->data, value->header.stamp.sec, value->header.stamp.nsec);
  }
  void posCallbackR(const webots_ros::Float64Stamped::ConstPtr &value){
    theta_2 = -value->data-(M_PI-0.2931572);
  }

  void polyCallback(const new_controller::trajectory_polynomial::ConstPtr &value){
    for(int i = 0; i < 4; i++){
      xpoly[i] = value->poly_x[i];
      ypoly[i] = value->poly_y[i];
      time_to_finish = value->delta_time;
    }
  new_polynomial = true;
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

  // Service for retrieving current simulation time
  ros::service::waitForService("/fivebarTrailer/robot/get_time");
  time_client = n.serviceClient<webots_ros::get_float>("/fivebarTrailer/robot/get_time");
  webots_ros::get_float time_request;
  time_request.request.ask = true;

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
  //ros::Publisher encoderPub = n.advertise<new_controller::motor_pos>("encoderData", 10);
  // message for the encoder
  new_controller::motor_pos encoderMsg;

  // Set available torque for the motors
  ros::ServiceClient torqueLClient = n.serviceClient<webots_ros::set_float>("/fivebarTrailer/MotorL/set_available_torque");
  ros::ServiceClient torqueRClient = n.serviceClient<webots_ros::set_float>("/fivebarTrailer/MotorR/set_available_torque");
  webots_ros::set_float torqueMsg;
  torqueMsg.request.value = 21;
  //torqueLClient.call(torqueMsg);
  //torqueRClient.call(torqueMsg);

  // Set up the trajectory generation callback function
  ros::Subscriber trajectoryService = n.subscribe("trajectory_polynomial", 10, &robotController::polyCallback, &rc);

  // main loop
  while (ros::ok()) {

    float trajectory_refresh_rate = 1000.0;
    new_controller::set_pos motorSrv;
    ros::spinOnce();
    
    // If there is a new polynomial run the while loop
    while(rc.new_polynomial){
      // Set the new_polynomial to false, since i started working on it
      rc.new_polynomial = false;
      float step_size = 1/trajectory_refresh_rate;
      
      //Get initial time
      float previous_time;
      if(time_client.call(time_request)){
        previous_time =  time_request.response.value;
      }
      else{exit(420);} // Error message in case time client is unavailable

      // Run through the trajectory with a given step size
      time_client.call(time_request);
      float start_time = time_request.response.value;
      float t = 0.0;
      int n = 1;
      while(t <= rc.time_to_finish && !rc.new_polynomial){
        time_client.call(time_request);
        t = time_request.response.value-start_time;
        float a = step_size*n;
        //ROS_INFO("current time = %f seconds\n next point = %f", t, a);

        // send trajectory to the manipulator
        motorSrv.request.x = rc.xpoly[3] * pow(t, 3) + rc.xpoly[2] * pow(t, 2) + rc.xpoly[1] * t + rc.xpoly[0];
        motorSrv.request.y = rc.ypoly[3] * pow(t, 3) + rc.ypoly[2] * pow(t, 2) + rc.ypoly[1] * t + rc.ypoly[0];
        setPos(motorSrv.request,motorSrv.response);

        while(t <= step_size*n && !rc.new_polynomial){
          time_client.call(time_request);
          t = time_request.response.value-start_time;
          ros::spinOnce();
        }
        n++;
        ros::spinOnce();
      }

    ros::spinOnce();
    }
  }
}

// Inverse kinematics
void invKin(float xPos, float yPos){
  // Robot lengths
  int L0 = 176;
  int L1 = 573;
  int L2 = 714;
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
bool setPos(new_controller::set_pos::Request &posmsg,
            new_controller::set_pos::Response &nt){
  // input
  float posx = posmsg.x;
  float posy = posmsg.y;
  new_controller::set_pos tempmsg;
  //ROS_INFO("Position set to %f %f", posx, posy);

  invKin(posx, posy); // get the motor positions
  //ROS_INFO("calculated motor pos %f %f", angl1, angl2);
  // package it and send it towards the motors
  tempmsg.request.theta_1 = angl1;
  tempmsg.request.theta_2 = angl2;
  setMotorPos(tempmsg.request,tempmsg.response);
  return true;
}

// Function for setting the motors in a desired position
bool setMotorPos(new_controller::set_pos::Request &motormsg,
                 new_controller::set_pos::Response &nah){

  double posL = motormsg.theta_1-(M_PI-0.3364441);
  double posR = -motormsg.theta_2+(M_PI-0.2931572);

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
