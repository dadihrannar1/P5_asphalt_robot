#include <armadillo>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_generation/vision_out.h>
#include <cmath>

int L0 = 250;
int L1 = 620;
int L2 = 745;

void coordinate_callback(const trajectory_generation::vision_out::ConstPtr &coordinates)
{
  ROS_INFO("x coordinate: [%d]", coordinates->x);
  ROS_INFO("y coordinate: [%d]", coordinates->y);
  // ROS_INFO("starting a crack: [%s]", msg-> crack);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("crack_coordinates", 1000, coordinate_callback);

  ros::spin();
  return 0;
}

float *Trajpoints(float *startPos, float *endPos, const int points)
{
  float wpts[] = {{startPos[0]}, {startPos[1]}};
  for (int i = 1; i <= points; i++)
  {
    wpts += {{startPos[0] + (startPos[1] - startPos[0]) / points * i}, {endPos[0] + (endPos[1] - endPos[0]) / points * i}};
  }
  return wpts;
}

float *avals(float thetastart, float thetaend, float startacc, float endacc, float tf)
{
  float a[4] = {};
  a[0] = thetastart;
  a[1] = startacc;
  a[2] = 3 / (pow(tf, 2)) * (thetaend - thetastart);
  a[3] = -2 / (pow(tf, 3)) * (thetaend - thetastart);
  return a;
}

float *ForKin(float theta1, float theta2)
{
  float AB[2] = {L1 * cos(theta1), L1 * sin(theta1)};
  float AD[2] = {L0 - L1 * cos(theta2), L1 * sin(theta2)};
  float BD[2] = {abs(AD[0] - AB[0]), abs(AD[1] - AB[1])};
  float L3 = sqrt(pow(BD[0], 2) + pow(BD[1], 2));
  // hojde = sqrt((L2^2)-((L3/2)^2));
  float phi2 = acos((L3 / 2) / (L2));
  float phi1 = atan2(AD[1] - AB[1], AD[0] - AB[0]);

  // y1 = acos((L3^2)/((2*L3*L2)));
  // vinkel = acos((dot([1,0],P34))/(1*L3));
  float E[] = {AB[0] + L2 * cos(phi1 + phi2), AB[1] + L2 * sin(phi1 + phi2)};
  return E;
}

float *InvKin(float xPos, float yPos)
{
  float F0 = sqrt(pow(xPos, 2) + pow(yPos, 2));
  float F1 = sqrt(pow((L0 - xPos), 2) + pow(yPos, 2));
  float y00 = acos((pow(L1, 2) + pow(F0, 2) - pow(L2, 2)) / (2 * F0 * L1));
  float y01 = acos((pow(xPos, 2) + pow(F0, 2) - pow(yPos, 2)) / (2 * F0 * xPos));
  float y10 = acos((pow(L1, 2) + pow(F1, 2) - pow(L2, 2)) / (2 * F1 * L1));
  float y11 = acos((pow((L0 - xPos), 2) + pow(F1, 2) - pow(yPos, 2)) / (2 * F1 * (L0 - xPos)));
  float Theta1 = y00 + y01;
  float Theta2 = y10 + y11;

  float Angl[] = {Theta1, Theta2};
  return Angl;
}