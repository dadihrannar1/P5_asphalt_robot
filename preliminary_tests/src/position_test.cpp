#include <ros/ros.h>
#include <cmath>
#include <vision/Draw_workspace.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Float64Stamped.h>
#include <pretests/set_pos.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <fstream>

#define TIME_STEP 4
#define N_TESTS 600 // times we check a coordinate

// Robot lengths
int L0 = 176;
int L1 = 573;
int L2 = 714;

// Global variables for read x and y values
float x;
float y;

// Global variables for calculatred angles
float angl1;
float angl2;


class Tester{
  public:
  float robot_motorL_pos = 0.0;
  float robot_motorR_pos = 0.0;
    // Callback functions for current motor positions
  void posCallbackL(const webots_ros::Float64Stamped::ConstPtr &value){
    float recieved_data = value->data;
    ROS_INFO("Recieved data L: %f", recieved_data);
    robot_motorL_pos = recieved_data+(M_PI-0.3364441);
  }
  void posCallbackR(const webots_ros::Float64Stamped::ConstPtr &value){
    float recieved_data = value->data;
    ROS_INFO("Recieved data R: %f", recieved_data);
    robot_motorR_pos = -recieved_data-(M_PI-0.2931572);
  }
    struct forKin_out{
    float x;
    float y;
  };

  forKin_out ForKin(){
    ROS_INFO("Motor positions: %f, %f", robot_motorL_pos, robot_motorR_pos);
    float AB[2] = {L1 * cos(robot_motorL_pos), L1 * sin(robot_motorL_pos)};
    ROS_INFO("AB = %f, %f", AB[0], AB[1]);
    float AD[2] = {L0 - L1 * cos(robot_motorR_pos), L1 * sin(robot_motorR_pos)};
    ROS_INFO("AD = %f, %f", AD[0], AD[1]);
    float BD[2] = {abs(AD[0] - AB[0]), abs(AD[1] - AB[1])};
    ROS_INFO("BD = %f, %f", BD[0], BD[1]);
    float L3 = sqrt(pow(BD[0], 2) + pow(BD[1], 2));
    ROS_INFO("L3 = %f", L3);
    float phi2 = acos((L3 / 2) / (L2));
    float phi1 = atan2(AD[1] - AB[1], AD[0] - AB[0]);
    ROS_INFO("phi1 phi2 = %f, %f",phi1, phi2);

    forKin_out ee_pos;
    ee_pos.x = AB[0] + L2 * cos(phi1 + phi2);
    ee_pos.y = AB[1] + L2 * sin(phi1 + phi2);
    return ee_pos;
  }
};




std::vector<int> generateCoord(int amountOfPoints){
  // Setting a vector in the size we want
  std::vector<int> w_pos;
  w_pos.resize(amountOfPoints*2);

  int workspace[4] = {};
  workspace[0] = L0/2-500; // left edge
  workspace[1] = L0/2+500; // right edge
  workspace[2] = abs(L1-L2); // bottom edge
  workspace[3] = abs(L1-L2)+1000; // top edge

  // vectors work like pointers, so when iterating through them i call their first pointer and then itterate through them
  for(auto i=w_pos.begin(); i != w_pos.end(); i=i+2){
      *i= workspace[0]+(rand()%(workspace[1]-workspace[0]));// x_pos
      *(i+1)= workspace[2]+(rand()%(workspace[3]-workspace[2]));// y_pos
  }
  return w_pos;
}


void callback(const geometry_msgs::PointStamped::ConstPtr &value){
  //ROS_INFO("x = %f, y = %f, z = %f", value->point.x,value->point.y,value->point.z);
  x = -(value->point.y*1000-L0/2);
  y = value->point.x*1000;
}

int main(int argc, char **argv)
{
  // start the ros node
  ros::init(argc, argv, "pos_test");
  ros::NodeHandle n;
  srand((unsigned)time(NULL)); // seed for random generation


  ROS_INFO("start");
  Tester test;
  // Setting up the webots timeStep messages
  webots_ros::set_int timeStepSrv;
  timeStepSrv.request.value = TIME_STEP;
  // Enable the GPS in webots
  ros::ServiceClient gpsClient = n.serviceClient<webots_ros::set_int>("/fivebarTrailer/NozzlePos/enable");
  ros::ServiceClient manipulatorClient = n.serviceClient<pretests::set_pos>("/manipulatorSetPos");
  gpsClient.call(timeStepSrv);

  ros::Subscriber posL = n.subscribe("/fivebarTrailer/PosL/value", 1, &Tester::posCallbackL, &test);
  ros::Subscriber posR = n.subscribe("/fivebarTrailer/PosR/value", 1, &Tester::posCallbackR, &test);

  ros::ServiceClient draw_client = n.serviceClient<vision::Draw_workspace>("draw_in_workspace");
  
  // In this test we only care about if positions can be reached, so we increase the motor output to faster rach desired points
  ros::ServiceClient torqueLClient = n.serviceClient<webots_ros::set_float>("/fivebarTrailer/MotorL/set_available_torque");
  ros::ServiceClient torqueRClient = n.serviceClient<webots_ros::set_float>("/fivebarTrailer/MotorR/set_available_torque");
  webots_ros::set_float torqueMsg;
  torqueMsg.request.value = 500;
  torqueLClient.call(torqueMsg);
  torqueRClient.call(torqueMsg);

  pretests::set_pos motorSrv; // This is the message for the motor node

  ros::Subscriber gpsSub = n.subscribe("/fivebarTrailer/NozzlePos/values", 1, callback); // GPS pos

  // set a vector class for the wanted positions
  std::vector<int> w_pos;
  w_pos.resize(N_TESTS*2);
  w_pos = generateCoord(N_TESTS); // generate positions

  float x_acc_res[N_TESTS] = {}; // legit remember the {} or we might get leftover memory
  float y_acc_res[N_TESTS] = {};
  float mag_acc_res[N_TESTS] = {};
  float acc_sum[3] = {};

  // First call does not work with the GPS so we just send a dummy pos first
  motorSrv.request.x = L0/2;
  motorSrv.request.y = 400;
  manipulatorClient.call(motorSrv);
  ros::Duration(1).sleep();
  ros::spinOnce();

  Tester::forKin_out start_coords;

  vision::Draw_workspace drawing_pos_srv;
  // Run through random positions
  int j=0; //since we are looping trough pointers I need a different counter
  
  for(auto n=w_pos.begin(); n!=w_pos.end();n=n+2){
    // The *(<variable>+value) is to get the next value in a pointer, sorta like an array
    motorSrv.request.x = float(*n);
    motorSrv.request.y = float(*(n+1));
    //ROS_INFO("\nwanted pos in x y = %f, %f",float(*n), float(*(n+1)));
    manipulatorClient.call(motorSrv); // tell the motors to move
    // Wait a little so that the arms can keep up
    ros::Duration(2).sleep(); // Sleep time depends on simulation speed
    ros::spinOnce();
    ros::Duration(2).sleep(); // Sleep time depends on simulation speed
    start_coords = test.ForKin();
    drawing_pos_srv.request.x = start_coords.x;
    drawing_pos_srv.request.y = start_coords.y;
    drawing_pos_srv.request.radius = int(ceil(10/0.9));
    ROS_INFO("\nwanted pos in x y = %f, %f", motorSrv.request.x, motorSrv.request.y);
    ROS_INFO("\ndrawing in pos in x y = %f, %f", drawing_pos_srv.request.x, drawing_pos_srv.request.y);
    draw_client.call(drawing_pos_srv);
    ros::Duration(0.1).sleep();



    // save the important data for the results file
    float x_acc = abs(*n-x);
    float y_acc = abs(*(n+1)-y);
    float magnitude = sqrt(pow(*n-x,2)+pow(*(n+1)-y,2));
    ROS_INFO("\naccuracy in x y = %f, %f",x_acc, y_acc);
    x_acc_res[j] = x_acc;
    y_acc_res[j] = y_acc;
    mag_acc_res[j] = magnitude;
    acc_sum[0] += x_acc;
    acc_sum[1] += y_acc;
    acc_sum[2] += magnitude;
    j++;
  }

  // Reset torque values
  torqueMsg.request.value = 21;
  torqueLClient.call(torqueMsg);
  torqueRClient.call(torqueMsg);


  //calculate the average
  float x_acc_avg = acc_sum[0]/N_TESTS;
  float y_acc_avg = acc_sum[1]/N_TESTS;
  float mag_acc_avg = acc_sum[2]/N_TESTS;
  // Pointers for the max and min values
  float* xpointer = std::max_element(x_acc_res, x_acc_res+N_TESTS);
  float* ypointer = std::max_element(y_acc_res, y_acc_res+N_TESTS);
  float* magpointer = std::max_element(mag_acc_res, mag_acc_res+N_TESTS);
  float x_acc_max = *(xpointer); float y_acc_max = *(ypointer); float mag_acc_max = *(magpointer);
  std::ofstream resfile; // .txt file to return results


  // This is just for debugging
  //ROS_INFO("Average accuracy:   x=%f, y=%f",x_acc_avg, y_acc_avg);
  //ROS_INFO("Highest deviations: x=%f, y=%f",x_acc_max, y_acc_max);

  // the results file can be found in your catkin workspace
  resfile.open("accuracy_test.txt"); // This will overwrite old data
  if (resfile.is_open()){
    // first we write in the important stuff
    resfile << "--------SUMMED RESULTS-------" << std::endl;
    resfile << "Average accuracy:   x=" << x_acc_avg << " y=" << y_acc_avg << " magnitude="<<mag_acc_avg << std::endl;
    resfile << "Highest deviations: x=" << x_acc_max << " y=" << y_acc_max << " magnitude="<<mag_acc_max << std::endl;
    resfile << "---------FULL RESULTS--------" << std::endl;
    for(int i =0; i<=N_TESTS;++i){
      resfile << "x,y,mag:" << x_acc_res[i] << ":" << y_acc_res[i] << ":" << mag_acc_res[i] << std::endl;
    }
  }
  resfile.close();
  
  return 0;
}