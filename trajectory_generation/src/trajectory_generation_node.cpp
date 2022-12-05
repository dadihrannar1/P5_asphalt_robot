#include <armadillo>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <deque>

/*
//Class to handle kinematic control and trajectory planning for the robot
class RobotKinematics{
private:
  //Link sizes
  const int L0 = 250;
  const int L1 = 620;
  const int L2 = 745;

  //Current motor angles
  double theta1, theta2;

  //Current end effector position
  double posx, posy;

  //Desired angles
  double desired_theta1, desired_theta2;

  //Desired end effector position
  double desired_posx, desired_posy;

public:
  double *forward_kinematics(double theta1, double theta2){
    double AB[2] = {L1 * cos(theta1), L1 * sin(theta1)};
    double AD[2] = {L0 - L1 * cos(theta2), L1 * sin(theta2)};
    double BD[2] = {abs(AD[0] - AB[0]), abs(AD[1] - AB[1])};
    double L3 = sqrt(pow(BD[0], 2) + pow(BD[1], 2));
    double phi2 = acos((L3 / 2) / (L2));
    double phi1 = atan2(AD[1] - AB[1], AD[0] - AB[0]);

    double E[] = {AB[0] + L2 * cos(phi1 + phi2), AB[1] + L2 * sin(phi1 + phi2)};
    return E;
  }

  double *inverse_kinematics(double xPos, double yPos){
    double F0 = sqrt(pow(xPos, 2) + pow(yPos, 2));
    double F1 = sqrt(pow((L0 - xPos), 2) + pow(yPos, 2));
    double y00 = acos((pow(L1, 2) + pow(F0, 2) - pow(L2, 2)) / (2 * F0 * L1));
    double y01 = acos((pow(xPos, 2) + pow(F0, 2) - pow(yPos, 2)) / (2 * F0 * xPos));
    double y10 = acos((pow(L1, 2) + pow(F1, 2) - pow(L2, 2)) / (2 * F1 * L1));
    double y11 = acos((pow((L0 - xPos), 2) + pow(F1, 2) - pow(yPos, 2)) / (2 * F1 * (L0 - xPos)));
    double Theta1 = y00 + y01;
    double Theta2 = y10 + y11;

    double Angl[] = {Theta1, Theta2};
    return Angl;
  }

  //Create interpolation points between a start and end position
  float *interpolated_points(float *startPos, float *endPos, const int points)
  {
    float wpts[] = {{startPos[0]}, {startPos[1]}};
    for (int i = 1; i <= points; i++)
    {
      wpts += {{startPos[0] + (startPos[1] - startPos[0]) / points * i}, {endPos[0] + (endPos[1] - endPos[0]) / points * i}};
    }
    return wpts;
  }

  //Create polynomium between two points
  float *avals(float thetastart, float thetaend, float startacc, float endacc, float tf)
  {
    float a[4] = {};
    a[0] = thetastart;
    a[1] = startacc;
    a[2] = 3 / (pow(tf, 2)) * (thetaend - thetastart);
    a[3] = -2 / (pow(tf, 3)) * (thetaend - thetastart);
    return a;
  }
};
*/
class CrackMapper{
private:
  ros::NodeHandle n;
  ros::Subscriber point_sub;
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;

  //List of current trajectories in world coordinates
  std::deque<geometry_msgs::PointStamped> trajectory_coordinates;

  //Boundary box for camera frame in world coordinate
  //Pose of camera origin? and width + height
  double length_camera;
  double width_camera;

public:
  CrackMapper(double camera_bbox_length, double camera_bbox_width):
  tf2_buffer(ros::Duration(600)), tf2_listener(tf2_buffer)
  {
    length_camera = camera_bbox_length;
    width_camera = camera_bbox_width;

    point_sub = n.subscribe<geometry_msgs::PointStamped>("points", 1000, &CrackMapper::coordinate_callback, this);
  }

  //geometry_msgs::TransformStamped camera_transform_stamped; //Transform from current image to world coordinates

  void coordinate_callback(const geometry_msgs::PointStamped::ConstPtr &coordinate){
    //Add trajectory coordinate to the back of the list
    geometry_msgs::PointStamped recieved_point;
    recieved_point.header = coordinate -> header;
    recieved_point.point = coordinate -> point;
    trajectory_coordinates.push_back(recieved_point);

    ROS_INFO("Received point: (%f, %f)", recieved_point.point.x, recieved_point.point.y);
  }
};

//Fetch a new transform with the proper time if the current one does not match
    /*
    while(recieved_point.header.stamp != camera_transform_stamped.header.stamp){
      try{
        //tfBuffer.waitForTranform();
        //tfListener.waitForTransform();

        camera_transform_stamped = tf2_buffer.lookupTransform("robot_frame", "world_frame", ros::Time(recieved_point.header.stamp.sec, recieved_point.header.stamp.nsec));
        
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.1).sleep();
        continue;
      }
    }

    //Transform coordinate into frame
    geometry_msgs::PointStamped point;
    tf2_buffer.transform(recieved_point, point, "world_frame");
    */

int main(int argc, char **argv){
  ros::init(argc, argv, "crack_points_listener");
  ros::NodeHandle n;

  CrackMapper trajectory_map(1.6, 1.024);

  while (n.ok()){
    ros::spinOnce(); //Spin subscriber once
    
  }
  
  return 0;
}