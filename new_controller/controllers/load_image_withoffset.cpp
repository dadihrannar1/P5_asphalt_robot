#include <iostream>
#include <fstream>
#include <new_controller/display_movement.h>

#include <ros/ros.h>
#include <boost/bind.hpp>

#include <webots_ros/display_draw_line.h>
#include <webots_ros/display_image_delete.h>
#include <webots_ros/display_image_load.h>
#include <webots_ros/display_image_new.h>
#include <webots_ros/display_image_paste.h>

#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <webots_ros/Float64Stamped.h>


#include <signal.h>
#include <std_msgs/String.h>
#include <cmath>
#include <vector>
#include <cstring> 


#define TIME_STEP 32 
#define NUM_DISPLAYS 25

class DisplayMover;

int main(int argc, char **argv) {
    ros::init(argc, argv, "displays");
}

class DisplayMover {
    private:
    float real_width = 1.86; // size of the cameras FoV
    float real_height = 1.04;

    ros::NodeHandle n;
    std::string model_name = "fivebarTrailer";
    webots_ros::set_int timeStepSrv;

    ros::ServiceClient pos_motor_client[NUM_DISPLAYS] = {}; 
    ros::ServiceClient vel_motor_client[NUM_DISPLAYS] = {};
    float curr_vel = 0;
    std::vector<std::string> filenames_crack;
    std::vector<int> time_IRL;
    std::vector<int> encoder1;
    std::vector<int> encoder2;
    

    void jsonFileReader(std::string path){
        // world sizes
        float pixelSize = 0.9712;
        float tickSize = 277.8/100;

        float tickPixel = pixelSize/tickSize;

        std::fstream jsonfile;
        jsonfile.open(path,std::ios::in); //open a file to perform read operation using file object
        if (jsonfile.is_open()){ //checking whether the file is open
            // define variables
            std::string tp;

            int j = 0;
            while(getline(jsonfile, tp, '"')){ //read data from file object and put it into string.
                if(tp == ", "){continue;} // if it's a ", " go next
                if(tp == "], ["){ // New file type
                    j++;
                    continue;
                    }
                if(tp == "]]"){continue;} // End of document

                std::istringstream fileStream(tp);
                std::string f_name;
                switch(j){
                    case 0:
                    while(getline(fileStream, f_name, '/')){} // I only care for the file name
                    filenames_crack.push_back(f_name);
                    break;
                    case 1:
                    time_IRL.push_back(std::stoi(tp));
                    break;
                    case 2:
                    encoder1.push_back(std::stoi(tp)/tickPixel); // save the encoder values as pixel offset
                    break;
                    case 3:
                    encoder2.push_back(std::stoi(tp)/tickPixel);
                    break;
                }
            }
            jsonfile.close(); //close the file object.
            ROS_INFO("files");
            ROS_INFO_STREAM(j);
        }
        }
    
    void importImages(){

    }

    void readDisplaypos(const webots_ros::Float64Stamped::ConstPtr &value){
        float x_length = value->data;
        int disp_num = 1;
        if (x_length >= real_width){

            // Set up the speed so we can move back to the start
            webots_ros::set_float motor_srv;
            motor_srv.request.value = 10000; // motor speed is in m/s
            vel_motor_client[disp_num].call(motor_srv);

            // make the motor go back to start
            motor_srv.request.value = 0; // 0 is the starting position 
            pos_motor_client[disp_num].call(motor_srv);

            // Set the velocity back
            motor_srv.request.value = curr_vel; // motor speed is in m/s
            vel_motor_client[disp_num].call(motor_srv);

        }
    }

    void SetSpeed(webots_ros::set_float::Request &msg, webots_ros::set_float::Response &ans){
        curr_vel = msg.value;
        webots_ros::set_float motor_srv;
        webots_ros::set_float pos_srv;
        motor_srv.request.value = curr_vel; // motor speed is in m/s

        for(int i = 0; i<NUM_DISPLAYS;i++){
            vel_motor_client[i].call(motor_srv);

            // Set up the speed so we can move back to the start

            motor_srv.request.value = 100; // motor speed is in m/s
            pos_motor_client[i].call(pos_srv);
        }
        
    }

    public:
    
    DisplayMover(){
        // set time step for encoder activation
        timeStepSrv.request.value = TIME_STEP;

        for(int i = 0; i<NUM_DISPLAYS;i++){
            webots_ros::set_int tempi;
            tempi.request.value = i;
            // Start up the encoders
            ros::ServiceClient encoderClient = n.serviceClient<webots_ros::set_int>(model_name + "/position_sensor" + std::to_string(i+1) +"/enable");
            encoderClient.call(timeStepSrv);
            ros::Subscriber subLeft = n.subscribe<webots_ros::Float64Stamped> (model_name + "/position_sensor" + std::to_string(i+1) + "/value", 1, &DisplayMover::readDisplaypos);
            //ros::Subscriber subLeft = n.subscribe<webots_ros::Float64Stamped>(model_name + "/position_sensor" + std::to_string(i+1) + "/value", 1, boost::bind(&DisplayMover::readDisplaypos,_1,&i));

            // set_position client
            vel_motor_client[i] = n.serviceClient<webots_ros::set_float>(model_name + "/position_sensor" + std::to_string(i+1) + "/set_velocity");
            pos_motor_client[i] = n.serviceClient<webots_ros::set_float>(model_name + "/position_sensor" + std::to_string(i+1) + "/set_position");
        }
    }
};


