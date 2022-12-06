#include <ros/ros.h>
#include <cmath>

#include <webots_ros/set_int.h>
#include <pretests/set_pos.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <fstream>

#define TIME_STEP 32
#define N_TESTS 100 // times we check a coordinate

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

std::vector<int> generateCoord(int amountOfPoints){
  // Setting a vector in the size we want
  std::vector<int> w_pos;
  w_pos.resize(amountOfPoints);

  int workspace[4] = {};
  workspace[0] = L0/2-500; // left edge
  workspace[1] = L0/2+500; // right edge
  workspace[2] = abs(L1-L2); // bottom edge
  workspace[3] = L1-L2+1000; // top edge

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
  // Setting up the webots timeStep messages
  webots_ros::set_int timeStepSrv;
  timeStepSrv.request.value = TIME_STEP;
  // Enable the GPS in webots
  ros::ServiceClient gpsClient = n.serviceClient<webots_ros::set_int>("/fivebarTrailer/NozzlePos/enable");
  ros::ServiceClient manipulatorClient = n.serviceClient<pretests::set_pos>("/manipulatorSetPos");
  gpsClient.call(timeStepSrv);
  //GPS needs to start up
  

  pretests::set_pos motorSrv; // This is the message for the motor node

  ros::Subscriber gpsSub = n.subscribe("/fivebarTrailer/NozzlePos/values", 1, callback); // GPS pos

  // set a vector class for the wanted positions
  std::vector<int> w_pos;
  w_pos.resize(N_TESTS);
  w_pos = generateCoord(N_TESTS); // generate positions

  float x_acc_res[N_TESTS] = {}; // legit remember the {} or we might get leftover memory
  float y_acc_res[N_TESTS] = {};
  float acc_sum[2] = {};

  // First call does not work with the GPS so we just send a dummy pos first
  motorSrv.request.x = L0/2;
  motorSrv.request.y = 400;
  manipulatorClient.call(motorSrv);
  ros::Duration(0.5).sleep();
  ros::spinOnce();

  // Run through random positions
  int j=0; //since we are looping trough pointers I need a different counter
  for(auto n=w_pos.begin(); n!=w_pos.end();n=n+2){
    // The *(<variable>+value) is to get the next value in a pointer, sorta like an array
    motorSrv.request.x = float(*n);
    motorSrv.request.y = float(*(n+1));
    ROS_INFO("\nwanted pos in x y = %f, %f",float(*n), float(*(n+1)));
    manipulatorClient.call(motorSrv); // tell the motors to move
    // Wait a little so that the arms can keep up
    ros::Duration(0.1).sleep(); // Sleep time depends on simulation speed
    ros::spinOnce();

    // save the important data for the results file
    float x_acc = abs(*n-x);
    float y_acc = abs(*(n+1)-y);
    ROS_INFO("\naccuracy in x y = %f, %f",x_acc, y_acc);
    x_acc_res[j] = x_acc;
    y_acc_res[j] = y_acc;
    acc_sum[0] += x_acc;
    acc_sum[1] += y_acc;
    j++;
  }
  
  //calculate the average
  float x_acc_avg = acc_sum[0]/N_TESTS;
  float y_acc_avg = acc_sum[1]/N_TESTS;
  // Pointers for the max and min values
  float* xpointer = std::max_element(x_acc_res, x_acc_res+N_TESTS);
  float* ypointer = std::max_element(y_acc_res, y_acc_res+N_TESTS);
  float x_acc_max = *(xpointer); float y_acc_max = *(ypointer);
  std::ofstream resfile; // .txt file to return results


  // This is just for debugging
  ROS_INFO("Average accuracy:   x=%f, y=%f",x_acc_avg, y_acc_avg);
  ROS_INFO("Highest deviations: x=%f, y=%f",x_acc_max, y_acc_max);

  // the results file can be found in your catkin workspace
  resfile.open("accuracy_test.txt"); // This will overwrite old data
  if (resfile.is_open()){
    // first we write in the important stuff
    resfile << "--------SUMMED RESULTS-------" << std::endl;
    resfile << "Average accuracy:   x=" << x_acc_avg << " y=" << y_acc_avg << std::endl;
    resfile << "Highest deviations: x=" << x_acc_max << " y=" << y_acc_max << std::endl;
    resfile << "---------FULL RESULTS--------" << std::endl;
    for(int i =0; i<=N_TESTS;++i){
      resfile << "x:" << x_acc_res[i] << " y:" << y_acc_res[i] << std::endl;
    }
  }
  resfile.close();
  
  return 0;
}