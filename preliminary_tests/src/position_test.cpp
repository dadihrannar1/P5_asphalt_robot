#include <ros/ros.h>
#include <cmath>

#include <webots_ros/set_int.h>
#include <pretests/set_pos.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

#define TIME_STEP 32
#define N_TESTS 100

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


float invKin(float xPos, float yPos)
{
  float F0 = sqrt(pow(xPos, 2) + pow(yPos, 2));
  float F1 = sqrt(pow((L0 - xPos), 2) + pow(yPos, 2));
  float y00 = acos((pow(L1, 2) + pow(F0, 2) - pow(L2, 2)) / (2 * F0 * L1));
  float y01 = acos((pow(xPos, 2) + pow(F0, 2) - pow(yPos, 2)) / (2 * F0 * xPos));
  float y10 = acos((pow(L1, 2) + pow(F1, 2) - pow(L2, 2)) / (2 * F1 * L1));
  float y11 = acos((pow((L0 - xPos), 2) + pow(F1, 2) - pow(yPos, 2)) / (2 * F1 * (L0 - xPos)));
  float Theta1 = y00 + y01;
  float Theta2 = y10 + y11;

  angl1 = Theta1;
  angl2 = Theta2;

  
  return 0;
}

void callback(const geometry_msgs::PointStamped::ConstPtr &value){
  ROS_INFO("x = %f, y = %f, z = %f", value->point.x,value->point.y,value->point.z);
  x = (value->point.z*1000+L0/2);
  y = -value->point.y*1000;
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
  ros::ServiceClient motorClient = n.serviceClient<pretests::set_pos>("motorSetPos");
  gpsClient.call(timeStepSrv);
  //GPS needs to start up
  

  pretests::set_pos motorSrv; // This is the message for the motor node

  ros::Subscriber gpsSub = n.subscribe("/fivebarTrailer/NozzlePos/values", 1, callback); // GPS pos

  // set a vector class for the wanted positions
  std::vector<int> w_pos;
  w_pos.resize(N_TESTS);
  w_pos = generateCoord(N_TESTS); // generate positions

  float acc_res[N_TESTS][2];

  // First call does not work with the GPS so we just send a dummy pos first
  motorSrv.request.theta_1 = M_PI/2;
  motorSrv.request.theta_2 = M_PI/2;
  motorClient.call(motorSrv);
  ros::Duration(2).sleep();
  ros::spinOnce();

  // Run through random positions
  int j=0; //since we are looping trough pointers I need a different counter
  for(auto n=w_pos.begin(); n!=w_pos.end();n=n+2){
    // The *(<variable>+value) is to get the next value in a pointer, sorta like an array
    invKin(*n, *(n+1));
    motorSrv.request.theta_1 = angl1;
    motorSrv.request.theta_2 = angl2;
    motorClient.call(motorSrv);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    float x_acc = *n-x;
    float y_acc = *(n+1)-y;
    ROS_INFO("\naccuracy in x y = %f, %f",x_acc, y_acc);
    acc_res[j][0] = x_acc;
    acc_res[j][1] = y_acc;
    j++;
  }
    
  
  
  return 0;
}