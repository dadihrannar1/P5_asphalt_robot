#include <ros/ros.h>
#include <cmath>

#include <webots_ros/set_int.h>
#include <pretests/set_pos.h>
#include <geometry_msgs/PointStamped.h>

#define TIME_STEP 32

// Robot lengths
int L0 = 290;
int L1 = 604;
int L2 = 595;

// Global variables for read x and y values
float x;
float y;

// Global variables for calculatred angles
float angl1;
float angl2;

int *generateCoord(){
  // We are returning pointers, aka static variables are needed
  static int w_pos[10];

  int workspace[4] = {};
  workspace[0] = L0/2-500; // left edge
  workspace[1] = L0/2+500; // right edge
  workspace[2] = L1-L2; // bottom edge
  workspace[3] = L1-L2+1000; // top edge

  
  for(int i=0; i<10; i=i+2){
      w_pos[i]= workspace[0]+(rand()%(workspace[1]-workspace[0]));// x_pos
      w_pos[i+1]= workspace[2]+(rand()%(workspace[3]-workspace[2]));// y_pos
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
  x = -(value->point.z*1000+L0/2);
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

  pretests::set_pos motorSrv; // This is the message for the motor node

  ros::Subscriber gpsSub = n.subscribe("/fivebarTrailer/NozzlePos/values", 1, callback); // GPS pos

  // Run through random positions
  for(int i=0; i<5; i++){
    int *w_pos;
    w_pos = generateCoord(); // generate positions
    ROS_INFO("%d",(w_pos+1));
    for(int n=0; n<10;n=n+2){
      // The *(<variable>+value) is to get the next value in a pointer, sorta like an array
      float *thetas; 
      invKin(*(w_pos+n), *(w_pos+n+1));
      ROS_INFO("\nDesired pos = %d, %d\n Calculated angle = %f,%f\n x y = %f, %f",*(w_pos+n),*(w_pos+n+1),angl1, angl2, x , y);
      motorSrv.request.theta_1 = angl1;
      motorSrv.request.theta_2 = angl2;
      motorClient.call(motorSrv);
      ros::Duration(2.0).sleep();
      ros::spinOnce();
    }
    
  }
  
  return 0;
}