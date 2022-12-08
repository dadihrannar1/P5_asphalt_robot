#include <iostream>
#include <fstream>
#include <string>
#include <cstring> 
#include <vector>

#include "ros/ros.h"


float pixelSize = 0.9712;
float tickSize = 277.8/100;

float tickPixel = pixelSize/tickSize;

int main(int argc, char **argv){
    ros::init(argc, argv, "a");
    ros::NodeHandle n;

   std::fstream jsonfile;
   jsonfile.open("/media/sf_shared_files/Images_lang2/image_details.json",std::ios::in); //open a file to perform read operation using file object
   if (jsonfile.is_open()){ //checking whether the file is open
      // define variables
      std::string tp;
      std::vector<std::string> filenames;
      std::vector<int> time;
      std::vector<int> encoder1;
      std::vector<int> encoder2;
      int j = 0;
      while(getline(jsonfile, tp, '"')){ //read data from file object and put it into string.
        if(tp == ", "){continue;} // if it's a ", " go next
        if(tp == "], ["){ // New file type
            j++;
            continue;
            }
        if(tp == "]]"){continue;} // End of document

        switch(j){
            case 0:
            filenames.push_back(tp);
            break;
            case 1:
            time.push_back(std::stoi(tp));
            break;
            case 2:
            encoder1.push_back(std::stoi(tp));
            break;
            case 3:
            encoder2.push_back(std::stoi(tp));
            break;
        }
      }
      jsonfile.close(); //close the file object.
      ROS_INFO("files");
      ROS_INFO_STREAM(j);
   }
}
