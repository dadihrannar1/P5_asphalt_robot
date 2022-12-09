#include <iostream>
#include <fstream>
#include <new_controller/display_movement.h>

#include <ros/ros.h>

#include <webots_ros/display_draw_line.h>
#include <webots_ros/display_image_delete.h>
#include <webots_ros/display_image_load.h>
#include <webots_ros/display_image_new.h>
#include <webots_ros/display_image_paste.h>

#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>


#include <signal.h>
#include <std_msgs/String.h>
#include <cmath>
#include <vector>
#include <cstring> 


#define TIME_STEP 32 
#define NUM_DISPLAYS 25

Class DisplayMover();

int main(int argc, char **argv) {

}

Class DisplayMover(){

}


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