#include <armadillo>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <new_controller/set_pos.h>
#include <webots_ros/get_float.h>
#include <cmath>
#include <deque>
#include <thread>

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
  //ros::NodeHandle n_;
  //ros::Subscriber point_sub;
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener;

  //Robot link sizes (in mm)
  int L0 = 176;
  int L1 = 573;
  int L2 = 714;

  //Bounds for robot workspace in robot frame (in m)
  float robot_x_min = (L0/2-500);
  float robot_x_max = (L0/2+500);
  float robot_y_min = abs(L1-L2);
  float robot_y_max = (abs(L1-L2)+1000);

  TrajectoryPolynomial generate_polynomial(float start_pos, float end_pos, float start_velocity, float end_velocity, float travel_time){
    TrajectoryPolynomial polynomial;
    //std::cout << "Polynomial generated: " << start_pos << " " << end_pos << " " << start_velocity << " " << end_velocity << std::endl;
    polynomial.a0 = start_pos;
    polynomial.a1 = start_velocity;
    polynomial.a2 = 3/pow(travel_time, 2) * (end_pos - start_pos) - (2/travel_time * polynomial.a1) - (1/travel_time * end_velocity);
    polynomial.a3 = -2/pow(travel_time, 3) * (end_pos - start_pos) + (1/pow(travel_time, 2) * (end_velocity + polynomial.a1));
    return polynomial;
  }

  bool is_new_coordinate(geometry_msgs::PointStamped coordinate, std::deque<geometry_msgs::PointStamped> coordinate_list, float min_dist){
    //Iterate through all existing coordinates
    for(int i = 0; i < coordinate_list.size(); i++){
      //Calculate the distance between coordinates in list and the new coordinate
      float distance = sqrt(pow(coordinate_list.at(i).point.x - coordinate.point.x, 2) + pow(coordinate_list.at(i).point.y - coordinate.point.y, 2));
      
      //If coordinate is too close to any other coordinate then do now accept it as new
      if(distance < min_dist){return false;}
    }
    return true;
  }

public:
  CrackMapper() : tf2_buffer(ros::Duration(600)), tf2_listener(tf2_buffer){
    //point_sub = n_.subscribe<geometry_msgs::PointStamped>("/points", 1000, &CrackMapper::coordinate_callback, this);
  }

  //List of current points in world coordinates
  std::deque<geometry_msgs::PointStamped> world_trajectory_coordinates;

  void coordinate_callback(const geometry_msgs::PointStamped::ConstPtr &coordinate){
    //Add trajectory coordinate to the back of the list
    geometry_msgs::PointStamped recieved_point;
    recieved_point.header = coordinate -> header;
    recieved_point.point = coordinate -> point;

    //Check to see if the point is already in the list
    if (is_new_coordinate(recieved_point, world_trajectory_coordinates, 0.01)){
      world_trajectory_coordinates.push_back(recieved_point);
      //ROS_INFO("Received new point: (%f, %f)", recieved_point.point.x, recieved_point.point.y);
    }else{ 
      //ROS_INFO("Received old point: (%f, %f)", recieved_point.point.x, recieved_point.point.y);
    }
  }

  std::deque<TrajectoryCombinedPoly> generate_trajectory(float vehicle_speed){
    //List of current coordinates in robot coordinates
    std::deque<geometry_msgs::PointStamped> robot_trajectory_coordinates;

    //ROS_INFO_STREAM(world_trajectory_coordinates.size());

    //Transform coordinates from world coordinates to robot coordinates
    for(int i = 0; i < world_trajectory_coordinates.size(); i++){
      geometry_msgs::PointStamped point_in_robot_frame;

      try{
        geometry_msgs::TransformStamped current_robot_transform = tf2_buffer.lookupTransform("robot_frame", "world_frame", ros::Time(0));

        //Apply transform to points
        tf2::doTransform(world_trajectory_coordinates.at(i), point_in_robot_frame, current_robot_transform);
        
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

      }catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
      }
    }

    //List of current trajectories in robot coordinates
    std::deque<TrajectoryCombinedPoly> full_combined_trajectory = {};

    // In case there are not enough points return
    if(robot_trajectory_coordinates.size() < 2){
      return full_combined_trajectory;
    }

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
  float vehicle_speed = 1.388; //TODO: Calculate vehicle speed
  return vehicle_speed;
}

//Thread function for following trajectory
void trajectory_thread(std::deque<TrajectoryCombinedPoly> polynomial, ros::NodeHandle& n, float step_size){
  ros::ServiceClient manipulatorClient = n.serviceClient<new_controller::set_pos>("/manipulatorSetPos");
  new_controller::set_pos motorSrv; // This is the message for the motor node

  //Service for timing with webots
  ros::service::waitForService("/fivebarTrailer/robot/get_time");
  ros::ServiceClient time_client = n.serviceClient<webots_ros::get_float>("/fivebarTrailer/robot/get_time");
  webots_ros::get_float time_request;
  time_request.request.ask = true;

  //Get initial time
  float previous_time;
  if(time_client.call(time_request)){
      previous_time =  time_request.response.value;
  }
  else{exit(420);}

  for(int i = 0; i <polynomial.size(); i++){
    int start_time = 0;
    int end_time = start_time + polynomial.at(i).delta_time;
    

    for(float t = start_time; t <= end_time; t+=step_size){
      //Send trajectory to manipulator
      motorSrv.request.x = polynomial.at(i).x_polynomial.a3 * pow(t, 3) + polynomial.at(i).x_polynomial.a2 * pow(t, 2) + polynomial.at(i).x_polynomial.a1 * t + polynomial.at(i).x_polynomial.a0;
      motorSrv.request.y = polynomial.at(i).y_polynomial.a3 * pow(t, 3) + polynomial.at(i).y_polynomial.a2 * pow(t, 2) + polynomial.at(i).y_polynomial.a1 * t + polynomial.at(i).y_polynomial.a0;
      //std::cout << "time: " << t << "x_polynomial.a3: " << polynomial.at(i).x_polynomial.a3 << "\n";
      //std::cout << "Request x " << motorSrv.request.x << std::endl;
      //std::cout << "Request y " << motorSrv.request.y << std::endl;
      manipulatorClient.call(motorSrv);
      while(true){
        if(time_client.call(time_request)){
            float current_time = time_request.response.value;
            if(current_time > previous_time + step_size/1000){
                previous_time = current_time;
                break;
            }
            sleep(0.01);
        }else{exit(69);} //Did not get an answer
      }
    }
  }
}

void coordinate_callback_dummy(const geometry_msgs::PointStamped::ConstPtr &coordinate){
    //Add trajectory coordinate to the back of the list
    geometry_msgs::PointStamped recieved_point;
    recieved_point.header = coordinate -> header;
    recieved_point.point = coordinate -> point;

    //ROS_INFO("Received new point: (%f, %f)", recieved_point.point.x, recieved_point.point.y);
  }

int main(int argc, char **argv){
  ros::init(argc, argv, "crack_points_listener");
  ros::NodeHandle n;
  ros::Publisher vehicle_vel_pub = n.advertise<std_msgs::Float64>("/vehicle_speed", 10);
  ros::service::waitForService("/fivebarTrailer/robot/get_time");
  //ros::topic::waitForMessage<geometry_msgs::PointStamped>("/points");

  CrackMapper trajectory_mapper;
  //ros::Subscriber point_sub = n.subscribe<geometry_msgs::PointStamped>("/points", 1000, coordinate_callback_dummy);
  ros::Subscriber point_sub = n.subscribe<geometry_msgs::PointStamped>("/points", 10000, &CrackMapper::coordinate_callback, &trajectory_mapper);
  
  std::cout << "Trajectory subscribed to " << point_sub.getTopic() << " with " << point_sub.getNumPublishers() << " publishers\n";

  //trajectory service frequency
  float srv_hz = 10;

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