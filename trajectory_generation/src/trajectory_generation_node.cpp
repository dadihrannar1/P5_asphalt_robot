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
#include <std_msgs/Float64.h>
#include <new_controller/set_pos.h>
#include <cmath>
#include <deque>
#include <thread>

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
struct TrajectoryPolynomial{
  float a0;
  float a1;
  float a2;
  float a3;
};

struct TrajectoryCombinedPoly{
  TrajectoryPolynomial x_polynomial;
  TrajectoryPolynomial y_polynomial;
  float delta_time;
};

class CrackMapper{
private:
  ros::NodeHandle n;
  ros::Subscriber point_sub;
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;

  //List of current points in world coordinates
  std::deque<geometry_msgs::PointStamped> world_trajectory_coordinates;

  //Robot link sizes (in mm)
  int L0 = 176;
  int L1 = 573;
  int L2 = 714;

  //Bounds for robot workspace in robot frame (in m)
  float robot_x_min = (L0/2-500)/1000;
  float robot_x_max = (L0/2+500)/1000;
  float robot_y_min = abs(L1-L2)/1000;
  float robot_y_max = (abs(L1-L2)+1000)/1000;

  TrajectoryPolynomial generate_polynomial(float start_pos, float end_pos, float start_velocity, float end_velocity, float travel_time){
    TrajectoryPolynomial polynomial;
    polynomial.a0 = start_pos;
    polynomial.a1 = start_velocity;
    polynomial.a2 = 3/pow(travel_time, 2) * (end_pos - start_pos) - (2/travel_time * polynomial.a1) - (1/travel_time * end_velocity);
    polynomial.a3 = -2/pow(travel_time, 3) * (end_pos - start_pos) + (1/pow(travel_time, 2) * (end_velocity + polynomial.a1));
    return polynomial;
  }

  void coordinate_callback(const geometry_msgs::PointStamped::ConstPtr &coordinate){
    //Add trajectory coordinate to the back of the list
    geometry_msgs::PointStamped recieved_point;
    recieved_point.header = coordinate -> header;
    recieved_point.point = coordinate -> point;
    world_trajectory_coordinates.push_back(recieved_point);

    ROS_INFO("Received point: (%f, %f)", recieved_point.point.x, recieved_point.point.y);
  }

public:
  CrackMapper():
  tf2_buffer(ros::Duration(600)), tf2_listener(tf2_buffer){
    point_sub = n.subscribe<geometry_msgs::PointStamped>("points", 1000, &CrackMapper::coordinate_callback, this);
  }

  std::deque<TrajectoryCombinedPoly> generate_trajectory(float vehicle_speed){
    //List of current coordinates in robot coordinates
    std::deque<geometry_msgs::PointStamped> robot_trajectory_coordinates;

    //Transform coordinates from world coordinates to robot coordinates
    for(int i = 0; i < world_trajectory_coordinates.size(); i++){
      geometry_msgs::PointStamped point_in_robot_frame = tf2_buffer.transform(world_trajectory_coordinates.at(i), "robot_frame");

      //Transform distances from m to mm
      point_in_robot_frame.point.x = point_in_robot_frame.point.x*1000;
      point_in_robot_frame.point.y = point_in_robot_frame.point.y*1000;

      //Determine if coordinates are within the robot frame
      if(point_in_robot_frame.point.x < robot_x_min){}
      else if(point_in_robot_frame.point.x > robot_x_max){}
      else if(point_in_robot_frame.point.y < robot_y_min){}
      else if(point_in_robot_frame.point.y > robot_y_max){}
      else {
        //coordinate is within robot frame
        robot_trajectory_coordinates.push_back(point_in_robot_frame);
        world_trajectory_coordinates.erase(world_trajectory_coordinates.begin() + i);

        //TODO: What if the coordinate is already fixed? How do we remove them from the world trajectory?
      }
    }

    //List of current trajectories in robot coordinates
    std::deque<TrajectoryCombinedPoly> full_combined_trajectory;
    float start_x_velocity = 0;
    float start_y_velocity = 0;

    //Keeping track of the movements in the y direction
    float total_travel_time = 0;

    //Generate polynomial between all the points in robot frame
    for(int i = 0; i < robot_trajectory_coordinates.size()-1; i++){
      geometry_msgs::PointStamped coordinate_1 = robot_trajectory_coordinates.at(i);
      geometry_msgs::PointStamped coordinate_2 = robot_trajectory_coordinates.at(i+1);

      //Maximum velocity in either x or y direction (mm/s)
      float max_operating_velocity = 305; //TODO this does not need to be static as it varies throughout the workspace

      //TODO: Calculate travel time as function of travel distance
      float travel_time = 0.1; //in seconds

      //Define x trajectory end velocity dependent on which part is calculated
      float end_x_velocity;
      if (robot_trajectory_coordinates.size()-1 == i+1){
        //Last coordinate ends with robot standing still
        end_x_velocity = 0;
      }
      else{
        //Determine direction of travel
        if(coordinate_2.point.x < robot_trajectory_coordinates.at(i+2).point.x){
          end_x_velocity = max_operating_velocity;
        }
        else if(coordinate_2.point.x > robot_trajectory_coordinates.at(i+2).point.x){
          end_x_velocity = -max_operating_velocity;
        }
        else {
          end_x_velocity = 0;
        }
      }
      //Polynomial for x movements
      TrajectoryPolynomial x_polynomial = generate_polynomial(coordinate_1.point.x, coordinate_2.point.x, start_x_velocity, end_x_velocity, travel_time);
      start_x_velocity = end_x_velocity;


      //Define y trajectory end velocity dependent on which part is calculated
      float end_y_velocity;
      if (robot_trajectory_coordinates.size()-1 == i+1){
        //Last coordinate ends with robot standing still
        end_y_velocity = 0;
      }
      else{
        //Determine direction of travel
        if(coordinate_2.point.x < robot_trajectory_coordinates.at(i+2).point.x){
          end_y_velocity = max_operating_velocity;
        }
        else if(coordinate_2.point.x > robot_trajectory_coordinates.at(i+2).point.x){
          end_y_velocity = -max_operating_velocity;
        }
        else {
          end_y_velocity = 0;
        }
      }
      
      //Calculate how much the y coordinates moved based on the speed of the vehicle and the amount of time spent on the trajectory so far
      float y_coord_offset_start = vehicle_speed * total_travel_time;
      float y_coord_offset_end = vehicle_speed * travel_time;
      total_travel_time += travel_time;
      TrajectoryPolynomial y_polynomial = generate_polynomial(coordinate_1.point.y + y_coord_offset_start, coordinate_2.point.y + y_coord_offset_end, start_y_velocity, end_y_velocity, travel_time);
      start_y_velocity = end_y_velocity;

      //Make full trajectory
      TrajectoryCombinedPoly combined_polynomial;
      combined_polynomial.x_polynomial = x_polynomial;
      combined_polynomial.y_polynomial = y_polynomial;
      combined_polynomial.delta_time = travel_time;
      full_combined_trajectory.push_back(combined_polynomial);
    }
    return full_combined_trajectory;
  }
};

//calculate neccesary vehicle speed (m/s)
float adjust_speed(){
  float vehicle_speed; //TODO: Calculate vehicle speed
  return vehicle_speed;
}

//Thread function for following trajectory
void trajectory_thread(std::deque<TrajectoryCombinedPoly> polynomial, ros::NodeHandle n, int step_size){

  ros::ServiceClient manipulatorClient = n.serviceClient<new_controller::set_pos>("/manipulatorSetPos");
  new_controller::set_pos motorSrv; // This is the message for the motor node

  for(int i = 0; i <polynomial.size(); i++){
    int start_time;
    int end_time = start_time + polynomial.at(i).delta_time;

    for(int t = start_time; t <= end_time; t+=step_size){
      //Send trajectory to manipulator
      motorSrv.request.x = polynomial.at(i).x_polynomial.a3 * pow(t, 3) + polynomial.at(i).x_polynomial.a2 * pow(t, 2) + polynomial.at(i).x_polynomial.a1 * t + polynomial.at(i).x_polynomial.a0;
      motorSrv.request.y = polynomial.at(i).y_polynomial.a3 * pow(t, 3) + polynomial.at(i).y_polynomial.a2 * pow(t, 2) + polynomial.at(i).y_polynomial.a1 * t + polynomial.at(i).y_polynomial.a0;
      
      ros::Duration(step_size).sleep();
    }
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "crack_points_listener");
  ros::NodeHandle n;
  ros::Publisher vehicle_vel_pub = n.advertise<std_msgs::Float64>("/vehicle_speed", 10);
  ros::topic::waitForMessage<geometry_msgs::PointStamped>("/points");

  CrackMapper trajectory_mapper;

  //trajectory service frequency
  float srv_hz = 100;

  while (n.ok()){
    ros::spinOnce(); //Spin subscriber once
    float vehicle_speed = adjust_speed();
    std::deque<TrajectoryCombinedPoly> trajectory_combined = trajectory_mapper.generate_trajectory(vehicle_speed);

    //Start thread to send trajectory
    std::thread t(std::bind(trajectory_thread, trajectory_combined, n, 1/srv_hz));
    t.join();
  }
  
  return 0;
}