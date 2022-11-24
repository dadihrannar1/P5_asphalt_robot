// #include <webots/Robot.hpp>
// #include <webots/Motor.hpp>
// #include <webots/PositionSensor.hpp>
#include </usr/local/webots/include/controller/cpp/webots/Robot.hpp>
#include </usr/local/webots/include/controller/cpp/webots/Motor.hpp>
#include </usr/local/webots/include/controller/cpp/webots/PositionSensor.hpp>
#include </usr/local/webots/include/controller/cpp/webots/Display.hpp>
#include </usr/local/webots/include/controller/cpp/webots/GPS.hpp>
// #include </usr/local/webots/include/controller/cpp/webots/Shape.hpp>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include </home/dadi/p5_ws/src/P5_asphalt_robot/UDP/include/UDP_Com.h>
#include <chrono>
#include "include/Controller.h"
#include "include/Encoder.h"
#include "include/MotionPlanning.h"

//Til multithreading
#include <sys/wait.h>
#include <unistd.h>
#include <thread>
#include <mutex>

std::mutex MutexP;

Encoder Encoder;

// Marie Alternation
void Simulation(){ //Udnytte positioner og tiden beregnet i encoderen

  Controller RobotController;
  MotionPlanning Motion;
  Encoder.setMeasurements(RobotController.L0, RobotController.L1, RobotController.L2);
  webots::Display CrackDisplay("CrackDisplay");
  webots::GPS GPS("gps");
  GPS.enable(1);

  Motion.debug = false;
  Encoder.debug = false;

  std::ofstream fout;
  fout.open("TimeLog.txt");
  fout.clear();

  int iteration = 0;
  double ptime = 0;
  std::vector<Point> goals;
  RobotController.FastMove(0.1, 1, false);

  std::vector<float> avgY;

  while(RobotController.robot->step(1) != -1){

    double time = RobotController.robot->getTime();

    if(goals.size() > 1){
      if( (goals.at(1).goalT < time)){
        MutexP.lock();  
          auto TimeBegin = std::chrono::high_resolution_clock::now();
          goals = Encoder.getGoalsForTrajectoryPlanning(time);
          auto TimeEncoder = std::chrono::high_resolution_clock::now();
        MutexP.unlock();
        Motion.Plan(goals, time);
        Motion.GetPosition(time-ptime);
        auto TimeTrajectory = std::chrono::high_resolution_clock::now();
        ptime = (double)goals.at(0).goalT;

        auto TimeTakenEncoder = std::chrono::duration_cast<std::chrono::nanoseconds>(TimeEncoder - TimeBegin);
        auto TimeTakenTrajectory = std::chrono::duration_cast<std::chrono::nanoseconds>(TimeTrajectory - TimeEncoder);

        fout << "EncoderTime: " << TimeTakenEncoder.count() << " TrajectoryTime: " << TimeTakenTrajectory.count() << std::endl;

        iteration+=1;
      }
    } else if(goals.size() == 1){
        ptime = (double)goals.at(0).goalT;
        MutexP.lock();  
          goals = Encoder.getGoalsForTrajectoryPlanning(time);
        MutexP.unlock();
        Motion.Plan(goals, time);
        iteration+=1;
    } else {
      MutexP.lock();  
        goals = Encoder.getGoalsForTrajectoryPlanning(time);
      MutexP.unlock();
    }
    
    float* PositionToMove = Motion.GetPosition(time-ptime);

    float movex = PositionToMove[0] - 0.375;
    float movey = -PositionToMove[1] - 0.6857;
    
     std::cout << "X:" << PositionToMove[0] << " Y:" << PositionToMove[1] << " Iteration:" << iteration << " Time:" << time-ptime << " goals size:" << goals.size() << std::endl;
 
    RobotController.FastMove(movex, movey, false);

    RobotController.robot->getMotor("MotorL")->enableTorqueFeedback(1);
    RobotController.robot->getMotor("MotorR")->enableTorqueFeedback(1);
    fout << "LeftTorque:" << RobotController.robot->getMotor("MotorL")->getTorqueFeedback() << " ,RightTorque:" << RobotController.robot->getMotor("MotorR")->getTorqueFeedback() << "\n";
     std::cout << "LeftTorque:" << RobotController.robot->getMotor("MotorL")->getTorqueFeedback() << " ,RightTorque:" << RobotController.robot->getMotor("MotorR")->getTorqueFeedback() << "\n";


    float* coord = RobotController.ReturnCoord();
    // coord[0] = coord[0] + 0.375;
    // coord[1] = - coord[1] - 0.6857; 

    //avgY.push_back(coord[1]);

    const double* gpsCoord = GPS.getValues();

    coord[0] = (float)gpsCoord[0]-0.137;
    coord[1] = (float)gpsCoord[2]-2.014;

    float moveyy[2] = {movex+0.38, -movey-0.68};

    Encoder.visualizePoints(&CrackDisplay, time, coord);
    Encoder.visualizeEndEffector(&CrackDisplay,coord, "00FF00", 2);
    // Encoder.visualizeEndEffector(&CrackDisplay,moveyy, "FF00FF", 5);

    if(Encoder.Goals.size() == 0){
      int counter[4] = {0,0,0,0};
      float accuracy[4] = {0.05, 0.025, 0.015, 0.005};
      for (auto &&element : Encoder.GoalsHistory)
      {
        for (size_t i = 0; i < 4; i++)
        {
         if(element.goalT > accuracy[i]){
          counter[i]++; 
          }
        }
      }

     // for (size_t i = 0; i < 4; i++)
      //{
        //double HitPecentage = (double)counter[i] / (double)Encoder.GoalsHistory.size();
        //std::cout << "Percentage hit within " << accuracy[i] << " meters: " << (1-HitPecentage)*100 << "%" << std::endl;
     // }

      double averageY = 0;
      double accumulatedY = 0.0;
      int AverageYSize = 0;

      // for (size_t i = 0; i < avgY.size(); i++)
      // {
      //   if(isnan(avgY.at(i)) == 0){
      //     accumulatedY = accumulatedY + (double)avgY.at(i);
      //     AverageYSize++;
      //   }         
      // }
      // averageY = accumulatedY/(double)AverageYSize;
      // double loadindex = -92.215 * averageY -83.431;
      // std::cout << "LoadIndex: " << loadindex << "%" << std::endl;

      delete RobotController.robot;

    }

    
  }

fout.close();

delete RobotController.robot;

}

void Communication(){ // Udlede positioner og tider fra vision
//CHILD PROCESS
  UDP_Com UDP;

  // File pointer
  std::fstream fin;
  std::string filename = "/home/dadi/p5-ws/src/packages/files from mortensen/2021-12-16 Test/json_Realistic.txt";
  // Open an existing file
  std::cout << "Loading file: " << filename << std::endl;
  fin.open(filename, std::ios::in);
  if (!fin.is_open())
  {
      std::cout << "File: " << filename << " not opened. Check filename" << std::endl;
      system("pause");
      exit(1);
  }
  std::vector<std::string> tempdata;
  std::string line;
  std::stringstream s;
  while (std::getline(fin, line)) {
    
    if(line.size() > 2){

    UDP.DecodeMessage(line);

    int *pos = UDP.ExtractPosition();
    float *time = UDP.ExtractTime();
    float *crackDet = UDP.ExtractCrackDet();
    Point goal;
    goal.x = pos[0];
    goal.y = pos[1];   
    goal.frameT = time[0];
    goal.shift = crackDet[0];
      MutexP.lock();
        Encoder.addGoal(goal);
      MutexP.unlock();
    }
  }
  fin.close();

  MutexP.lock();
    Encoder.GoalsHistory = Encoder.Goals;
    for (auto &&i : Encoder.GoalsHistory)
    {
      i.goalT = 1;
    }
    
  MutexP.unlock();



  UDP.InitiateServer();
  //UDP.ToggleDebug(true);

  int rounds = 0;
  while (1){
    UDP.ReceiveMessage();
    // UDP.PrintMessage();
    int *pos = UDP.ExtractPosition();
    float *time = UDP.ExtractTime();
    float *crackDet = UDP.ExtractCrackDet();
    Point goal;
    goal.x = pos[0];
    goal.y = pos[1];   
    goal.frameT = time[0];
    goal.shift = crackDet[0];
      MutexP.lock();
        Encoder.addGoal(goal);
      MutexP.unlock();
    rounds++;
  }

}


int main(int argc, char **argv)
{
  std::thread WeBotsController (Simulation);
  std::thread CommunicationHandler (Communication);

  WeBotsController.join();
  CommunicationHandler.join();
  std::cout << "Syncronization of Threads Complete" << std::endl;
  
} 
        