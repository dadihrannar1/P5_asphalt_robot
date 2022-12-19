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
#include <std_msgs/Float32.h>
#include <new_controller/set_pos.h>
#include <new_controller/trajectory_polynomial.h>
#include <vision/Draw_workspace.h>
#include <webots_ros/get_float.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Float64Stamped.h>
#include <cmath>
#include <deque>

struct PolynomialCoefficients{
  float a0;
  float a1;
  float a2;
  float a3;
};

struct TrajectoryCombinedPoly{
  PolynomialCoefficients x_polynomial;
  PolynomialCoefficients y_polynomial;
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
  float workspace_length = robot_y_max - robot_y_min;

  //Maximum velocity in either x or y direction (mm/s)
  float max_operating_velocity = 2847; //TODO this does not need to be static as it varies throughout the workspace
  
  PolynomialCoefficients generate_polynomial(float start_pos, float end_pos, float start_velocity, float end_velocity, float travel_time){
    PolynomialCoefficients polynomial;
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

  float robot_motorL_pos;
  float robot_motorR_pos;

public:

  struct forKin_out{
    float x;
    float y;
  };

  forKin_out ForKin(){
    float AB[2] = {L1 * cos(robot_motorL_pos), L1 * sin(robot_motorL_pos)};
    float AD[2] = {L0 - L1 * cos(robot_motorR_pos), L1 * sin(robot_motorR_pos)};
    float BD[2] = {abs(AD[0] - AB[0]), abs(AD[1] - AB[1])};
    float L3 = sqrt(pow(BD[0], 2) + pow(BD[1], 2));
    float phi2 = acos((L3 / 2) / (L2));
    float phi1 = atan2(AD[1] - AB[1], AD[0] - AB[0]);

    forKin_out ee_pos;
    ee_pos.x = AB[0] + L2 * cos(phi1 + phi2);
    ee_pos.y = AB[1] + L2 * sin(phi1 + phi2);
    return ee_pos;
  }

  bool new_points = false;
  CrackMapper() : tf2_buffer(ros::Duration(600)), tf2_listener(tf2_buffer){
    //point_sub = n_.subscribe<geometry_msgs::PointStamped>("/points", 1000, &CrackMapper::coordinate_callback, this);
  }

  //List of current points in world coordinates
  std::deque<geometry_msgs::PointStamped> world_frame_coordinates;
  std::deque<geometry_msgs::PointStamped> robot_frame_coordinates;

  void coordinate_callback(const geometry_msgs::PointStamped::ConstPtr &coordinate){
    //Add trajectory coordinate to the back of the list
    geometry_msgs::PointStamped recieved_point;
    recieved_point.header = coordinate -> header;
    recieved_point.point = coordinate -> point;

    //Check to see if the point is already in the list
    if (is_new_coordinate(recieved_point, world_frame_coordinates, 0.0008853*2)){
      world_frame_coordinates.push_back(recieved_point);
      new_points = true;
      //ROS_INFO("Set new points to true");
      //ROS_INFO("Received new point: (%f, %f)", recieved_point.point.x, recieved_point.point.y);
    }else{ 
      //ROS_INFO("Received old point: (%f, %f)", recieved_point.point.x, recieved_point.point.y);
    }
  }

  // Callback functions for current motor positions
  void posCallbackL(const webots_ros::Float64Stamped::ConstPtr &value){
    //ROS_INFO("received motor pos");
    robot_motorL_pos = value->data+(M_PI-0.3364441);
  }
  void posCallbackR(const webots_ros::Float64Stamped::ConstPtr &value){
    robot_motorR_pos = -value->data-(M_PI-0.2931572);
  }
  //Function returns a deque of all coordinates in the robot workspace (coordinates in mm not m)
  std::deque<geometry_msgs::PointStamped> get_points_in_workspace(){
    //List of current coordinates in robot coordinates
    std::deque<geometry_msgs::PointStamped> points_in_workspace = {};

    //Transform coordinates from world coordinates to robot coordinates
    for(int i = 0; i < world_frame_coordinates.size(); i++){
      geometry_msgs::PointStamped point_in_robot_frame;

      try{
        geometry_msgs::TransformStamped current_robot_transform = tf2_buffer.lookupTransform("robot_frame", "world_frame", ros::Time(0));

        //Apply transform to points
        tf2::doTransform(world_frame_coordinates.at(i), point_in_robot_frame, current_robot_transform);
        
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
          points_in_workspace.push_back(point_in_robot_frame);
          world_frame_coordinates.erase(world_frame_coordinates.begin() + i);

          //TODO: What if the coordinate is already fixed? How do we remove them from the world trajectory?
        }

      }catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
      }
    }
    return points_in_workspace;
  }
  
  //Method to generate trajectory polynomial (Currently UNUSED in this implementation, as webots is slow)
  std::deque<TrajectoryCombinedPoly> generate_trajectory(float vehicle_speed){
    //List of current coordinates in robot coordinates
    std::deque<geometry_msgs::PointStamped> robot_trajectory_coordinates;
    
    //Get current position of end effector and add it as first coord for trajectory planning
    geometry_msgs::PointStamped start_pos;
    forKin_out start_coords = ForKin();
    start_pos.point.x = start_coords.x;
    start_pos.point.y = start_coords.y;
    start_pos.point.z = 0;

    robot_trajectory_coordinates.push_back(start_pos);

    //Transform coordinates from world coordinates to robot coordinates
    for(int i = 0; i < world_frame_coordinates.size(); i++){
      geometry_msgs::PointStamped point_in_robot_frame;

      try{
        geometry_msgs::TransformStamped current_robot_transform = tf2_buffer.lookupTransform("robot_frame", "world_frame", ros::Time(0));

        //Apply transform to points
        tf2::doTransform(world_frame_coordinates.at(i), point_in_robot_frame, current_robot_transform);
        
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

          //TODO: What if the coordinate is already fixed? How do we remove them from the world trajectory?

          //Does not work if previous polynomium is not finished by the time the robot calculates poly again
          //world_frame_coordinates.erase(world_frame_coordinates.begin() + i); 
        }

      }catch (tf2::TransformException &ex) {
        ROS_ERROR("Trajectory: %s", ex.what());
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

      //TODO: Calculate travel time as function of travel distance
      // Calculate time to finish
      float travel_time = sqrt(pow(coordinate_2.point.x-coordinate_1.point.x,2)+pow(coordinate_2.point.y-coordinate_1.point.y,2))/max_operating_velocity; //in seconds
      float vehicle_travel = (coordinate_1.point.y - coordinate_2.point.y)/vehicle_speed;
      if (vehicle_travel < travel_time){
        float travel_time = travel_time - vehicle_travel;
      }
      //ROS_INFO("Travel time: %f", travel_time);
      //ROS_INFO("Distance: %f", sqrt(pow(coordinate_2.point.x-coordinate_1.point.x,2)+pow(coordinate_2.point.y-coordinate_1.point.y,2)));
      
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
      PolynomialCoefficients x_polynomial = generate_polynomial(coordinate_1.point.x, coordinate_2.point.x, start_x_velocity, end_x_velocity, travel_time);
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
      PolynomialCoefficients y_polynomial = generate_polynomial(coordinate_1.point.y + y_coord_offset_start, coordinate_2.point.y + y_coord_offset_end, start_y_velocity, end_y_velocity, travel_time);
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

  //Calculate neccesary vehicle speed (m/s) from list of coordinates it needs to reach
  float adjust_speed(std::deque<geometry_msgs::PointStamped> crack_points){
    float travel_length = 0.0;
    for (int i = 1; i < crack_points.size(); i++){
      //Calculate distance bewteen crack_points
      float travel_length = sqrt(pow(crack_points.at(i).point.x - crack_points.at(i-1).point.x, 2) + pow(crack_points.at(i).point.y - crack_points.at(i-1).point.y, 2));
    }
    //Calculate max vehicle speed (mm/s) 
    float vehicle_speed = 1000/(max_operating_velocity/travel_length);
  return vehicle_speed;
  }
};

//Callback to receive the current trailer speed from the display node
float vehicle_speed = 1.388; //Standard vehicle speed in m/s
void vehicle_speed_callback(const std_msgs::Float32::ConstPtr &speed_msg){
  if(vehicle_speed != speed_msg->data){
    vehicle_speed = speed_msg->data;
  }
  ROS_INFO("Trajectory: Vehicle speed is %f", vehicle_speed);
}

//Callback to receive the current time from webots
float current_time = 0.0;
void webots_time_callback(const std_msgs::Float32::ConstPtr &time){
  current_time = time->data;
  //ROS_INFO("Trajectory: Simulation time is %f", current_time);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "crack_points_listener");
  ros::NodeHandle n;
  ros::Publisher poly_pub = n.advertise<new_controller::trajectory_polynomial>("trajectory_polynomial", 10);
  //ros::topic::waitForMessage<geometry_msgs::PointStamped>("/points");

  CrackMapper trajectory_mapper;
  //ros::Subscriber point_sub = n.subscribe<geometry_msgs::PointStamped>("/points", 1000, coordinate_callback_dummy);
  ros::Subscriber point_sub = n.subscribe<geometry_msgs::PointStamped>("/points", 10000, &CrackMapper::coordinate_callback, &trajectory_mapper);

  //Get current vehicle speed from the display node (Technically this should be from TF instead)
  ros::Subscriber vehicle_speed_sub = n.subscribe<std_msgs::Float32>("/velocity", 10, vehicle_speed_callback);

  //Callback for motorpositions from Webot (for forward kinematics)
  ros::Subscriber posL = n.subscribe<webots_ros::Float64Stamped>("/fivebarTrailer/PosL/value", 1, &CrackMapper::posCallbackL, &trajectory_mapper);
  ros::Subscriber posR = n.subscribe<webots_ros::Float64Stamped>("/fivebarTrailer/PosR/value", 1, &CrackMapper::posCallbackR, &trajectory_mapper);

  //Timing with webots
  ros::service::waitForService("/fivebarTrailer/robot/get_time");
  ros::Subscriber time_sub = n.subscribe<std_msgs::Float32>("/webots_time", 1 , webots_time_callback);

  //Service for drawing in the workspace
  ros::ServiceClient draw_client = n.serviceClient<vision::Draw_workspace>("draw_in_workspace");

  ros::ServiceClient manipulatorClient = n.serviceClient<new_controller::set_pos>("/manipulatorSetPos");
  new_controller::set_pos ee_pos_msg;
  ros::ServiceClient vehicleSpeedClient = n.serviceClient<webots_ros::set_float>("set_display_velocity");
  webots_ros::set_float vehicle_speed_msg;

  while (n.ok()){
    ros::spinOnce(); //Spin subscriber once to get new points, vehicle_speed, and end effector positions
    vision::Draw_workspace drawing_pos_srv;

    //Get all point inside the robot workspace
    std::deque<geometry_msgs::PointStamped> points = trajectory_mapper.get_points_in_workspace();

    //Set vehicle speed
    vehicle_speed_msg.request.value = trajectory_mapper.adjust_speed(points);
    //vehicleSpeedClient.call(vehicle_speed_msg);

    float y_offset = 0.0;
    float start_time = current_time;

    for (int i = 0; i < points.size(); i++){
      //Calculate y offset by vehicle speed
      y_offset = (current_time - start_time) * vehicle_speed_msg.request.value * 1000;

      trajectory_mapper.new_points = false;
      ee_pos_msg.request.x = points.at(i).point.x;
      ee_pos_msg.request.y = points.at(i).point.y + y_offset;
      manipulatorClient.call(ee_pos_msg);

      drawing_pos_srv.request.x = ee_pos_msg.request.x;
      drawing_pos_srv.request.y = ee_pos_msg.request.y + y_offset;
      drawing_pos_srv.request.radius = int(ceil(10/0.9));
      draw_client.call(drawing_pos_srv);
      //ROS_INFO("manipulator set to pos x = %f and y = %f", drawing_pos_srv.request.x, drawing_pos_srv.request.y);
      ros::spinOnce();
      if(trajectory_mapper.new_points){
        break;
      }
    }
    /* UNUSED Webots is too slow
    //Generate polynomium between all received points within workspace
    std::deque<TrajectoryCombinedPoly> trajectory_combined = trajectory_mapper.generate_trajectory(vehicle_speed);
    trajectory_mapper.new_points = false;
    
    //Get time before sending trajectory message
    time_client.call(time_request);
    float start_time = time_request.response.value;

    //Send messages to controller containing all the snippets of the trajectory one by one
    new_controller::trajectory_polynomial poly_msg;
    for (int i = 0; i < trajectory_combined.size() && !trajectory_mapper.new_points; i++){
      //Populate and send the message
      poly_msg.poly_x = {trajectory_combined.at(i).x_polynomial.a0, trajectory_combined.at(i).x_polynomial.a1, trajectory_combined.at(i).x_polynomial.a2, trajectory_combined.at(i).x_polynomial.a3};
      poly_msg.poly_y = {trajectory_combined.at(i).y_polynomial.a0, trajectory_combined.at(i).y_polynomial.a1, trajectory_combined.at(i).y_polynomial.a2, trajectory_combined.at(i).y_polynomial.a3};
      poly_msg.delta_time = trajectory_combined.at(i).delta_time;
      poly_pub.publish(poly_msg);

      //Wait for delta_time before sending next message
      time_client.call(time_request);
      float current_time = time_request.response.value;
      while (current_time < start_time + poly_msg.delta_time){
        ros::spinOnce();

        time_client.call(time_request);
        float current_time = time_request.response.value;
        if (trajectory_mapper.new_points){
          break;
        }
      }
      //Update start time
      start_time = start_time + poly_msg.delta_time;
    }
    */
  }
  
  return 0;
}