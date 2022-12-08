#include <iostream>
#include <fstream>
#include <new_controller/display_movement.h>



#include <ros/ros.h>


#include <webots_ros/display_draw_line.h>
#include <webots_ros/display_get_info.h>
#include <webots_ros/display_image_copy.h>
#include <webots_ros/display_image_delete.h>
#include <webots_ros/display_image_load.h>
#include <webots_ros/display_image_new.h>
#include <webots_ros/display_image_paste.h>
#include <webots_ros/display_image_save.h>


#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>


#include <signal.h>
#include <std_msgs/String.h>
#include <cmath>


#define TIME_STEP 32 


#define NUM_DISPLAYS 4

ros::ServiceClient pos_motor_client[NUM_DISPLAYS]; 
ros::ServiceClient vel_motor_client[NUM_DISPLAYS];




static int model_count;
static std::vector<std::string> model_list;
static bool callbackCalled = false;

void modelNameCallback(const std_msgs::String::ConstPtr &name) {
  model_count++;
  model_list.push_back(name->data);
  ROS_INFO("Model #%d: %s.", model_count, model_list.back().c_str());
  callbackCalled = true;
}





bool displayMover(new_controller::display_movement::Request &dispmsg, new_controller::display_movement::Response &nt){

float end_goal = dispmsg.y;
float velocity = dispmsg.velocity_dis;

webots_ros::set_float motor_srv;

motor_srv.request.value = velocity; // motor speed is in m/s

vel_motor_client[NUM_DISPLAYS].call(motor_srv);

motor_srv.request.value = end_goal;

pos_motor_client[NUM_DISPLAYS].call(motor_srv);
  

}


int main(int argc, char **argv) {
  std::string model_name = "fivebarTrailer";

  ros::init(argc, argv, "display_test", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  
  ros::Subscriber name_sub = n.subscribe("fivebarTraile", 100, modelNameCallback);




  webots_ros::display_get_info display_get_info_srv;
  
/*
  ros::ServiceClient display_get_info_client = n.serviceClient<webots_ros::display_get_info>(model_name + "/CrackDisplay/get_info");



  display_get_info_client.call(display_get_info_srv);
  
  ROS_INFO("width: %d, height: %d", (int)display_get_info_srv.response.width,(int)display_get_info_srv.response.height);
  

  display_get_info_client.shutdown();


*/
  

  webots_ros::display_image_load display_image_load_srv;
  ros::ServiceClient display_image_load_client;
  webots_ros::display_image_paste display_image_paste_srv;
  ros::ServiceClient display_image_paste_client; 

  uint64_t loaded_image;
  
 ros::Rate r(100000); //1000hz

 int i = 1;
 int j = 1296;

while (ros::ok())
{


  std::string image_path = "/long_crack_jpg/saved_image_"+std::to_string(j)+".jpg";
  ROS_INFO_STREAM(image_path);
  display_image_load_client = n.serviceClient<webots_ros::display_image_load>(model_name + "/CrackDisplay" + std::to_string(i) +"/image_load");

  display_image_load_srv.request.filename = (std::string(getenv("HOME")) + image_path);
  display_image_load_client.call(display_image_load_srv);
  ROS_INFO("Image successfully loaded to clipboard.");

  
  display_image_paste_client  = n.serviceClient<webots_ros::display_image_paste>(model_name + "/CrackDisplay"+ std::to_string(i) +"/image_paste");
  
  loaded_image = display_image_load_srv.response.ir;

  display_image_paste_srv.request.ir = loaded_image;

    if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("Image successfully load and paste.");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image.");

  


 i++;
 j++;
  
  if (i == 5){
    i == 1;
  }

 //std::cout<<std::to_string(j);


ros::Duration(0.5).sleep();

}
/*
  display_image_load_client = n.serviceClient<webots_ros::display_image_load>(model_name + "/CrackDisplay1/image_load");

  display_image_load_srv.request.filename = std::string(getenv("HOME")) + std::string("/long_crack_jpg/saved_image_1296.jpg");
  display_image_load_client.call(display_image_load_srv);
  ROS_INFO("Image successfully loaded to clipboard.");

  
  display_image_paste_client  = n.serviceClient<webots_ros::display_image_paste>(model_name + "/CrackDisplay1/image_paste");

  
  loaded_image = display_image_load_srv.response.ir;

  display_image_paste_srv.request.ir = loaded_image;

    if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("Image successfully load and paste.");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image.");


  
  display_image_load_client = n.serviceClient<webots_ros::display_image_load>(model_name + "/CrackDisplay2/image_load");

 
  display_image_load_srv.request.filename = std::string(getenv("HOME")) + std::string("/long_crack_jpg/saved_image_1297.jpg");
  display_image_load_client.call(display_image_load_srv);
  ROS_INFO("Image successfully loaded to clipboard.");

  
  display_image_paste_client = n.serviceClient<webots_ros::display_image_paste>(model_name + "/CrackDisplay2/image_paste");

  
  loaded_image = display_image_load_srv.response.ir;

  display_image_paste_srv.request.ir = loaded_image;

  if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("Image successfully load and paste.");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image.");


  display_image_load_client = n.serviceClient<webots_ros::display_image_load>(model_name + "/CrackDisplay3/image_load");
 

  display_image_load_srv.request.filename = std::string(getenv("HOME")) + std::string("/long_crack_jpg/saved_image_1298.jpg");
  display_image_load_client.call(display_image_load_srv);
  ROS_INFO("Image successfully loaded to clipboard.");

  
  display_image_paste_client = n.serviceClient<webots_ros::display_image_paste>(model_name + "/CrackDisplay3/image_paste");
  
  loaded_image = display_image_load_srv.response.ir;

  display_image_paste_srv.request.ir = loaded_image;

  if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("Image successfully load and paste.");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image.");

  
  display_image_load_client = n.serviceClient<webots_ros::display_image_load>(model_name + "/CrackDisplay4/image_load");

 
  display_image_load_srv.request.filename = std::string(getenv("HOME")) + std::string("/long_crack_jpg/saved_image_1299.jpg");
  display_image_load_client.call(display_image_load_srv);
  ROS_INFO("Image successfully loaded to clipboard.");

  
  display_image_paste_client = n.serviceClient<webots_ros::display_image_paste>(model_name + "/CrackDisplay4/image_paste");

  
  loaded_image = display_image_load_srv.response.ir;

  display_image_paste_srv.request.ir = loaded_image;



  

  
  if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("Image successfully load and paste.");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image.");

 

*/

  

   ros::ServiceServer displayMotor = n.advertiseService("display_vel", displayMover);

  

  
  ros::shutdown();


    return 0;



}

  

  /*
  ros::ServiceClient display_image_load_client;
  webots_ros::display_image_load display_image_load_srv;
  display_image_load_client = n.serviceClient<webots_ros::display_image_load>(model_name + "/display/image_load");

  display_image_load_srv.request.filename = std::string(getenv("HOME")) + std::string("/test_image_camera.png");
  display_image_load_client.call(display_image_load_srv);
  ROS_INFO("Image successfully loaded to clipboard.");
  uint64_t loaded_image = display_image_load_srv.response.ir;

  display_image_paste_srv.request.ir = loaded_image;
  if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("Image successfully load and paste.");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image.");

    */