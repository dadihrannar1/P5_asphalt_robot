// File:          Controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <math.h>
#include <vector>
#include <iostream>
#include "include/Controller.h"
#include </home/dadi/p5-ws/src/packages/UDP/include/UDP_Com.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define TIME_STEP 2


void Controller::LinearMove(float x, float y)
{
  //std::cout << "Moving linearly to " << x << ", " << y << std::endl;
  line(x, y);
}


void Controller::FastMove(float x, float y, bool PosCheck)
{
  //std::cout << "Moving fast to " << x << ", " << y << std::endl;

  InverseKinematics(x, y, PosCheck);
}

void Controller::waitForrobotToreachPos(float x, float y)
{
  robot->step(TIME_STEP);
  ForwardKinematics(getLpos(), getRpos());
  //std::cout << "Going to " << x << ", " << y << " while in " << xCoord << ", " << yCoord << std::endl;
 
  //while ( (-0.001 > (x-xCoord) > 0.001) || (-0.001 > (y-yCoord) > 0.001) )
   while(((xCoord-x) < -0.01 || (xCoord-x) > 0.01) || ((yCoord-y) < -0.01 || (yCoord-y) > 0.01))
  {
    robot->step(TIME_STEP);
    ForwardKinematics(getLpos(), getRpos());
    // //std::cout << "Waiting :) " << x << ", " << xCoord << ", " << y << ", " << yCoord << std::endl;
  }
  
}
float Controller::deg2rad(float angle)
{
  return (angle * M_PI / 180);
}

float Controller::getLpos()
{
  return PosL->getValue() + deg2rad(90);
}
float Controller::getRpos()
{
  return (PosR->getValue() * (-1)) + deg2rad(90);
}
float Controller::square(float input)
{
  return (input * input);
}

/*
void Controller::ForwardKinematics(float theta, float thetad)
{
  // float L0 = 0.4;
  // float L1 = 1.0;
  // float L2 = 1.2;
  double L3 = L2;
  double L4 = L1;
  //theta += deg2rad(90);
  //thetad = (thetad * (-1)) + deg2rad(90);
  double T1 = sqrt(square(L0) + square(L1) - 2 * L0 * L1 * cos(theta));
  double V1 = acos((square(T1) + square(L1) - square(L0)) / (2 * T1 * L1));
  double V2 = deg2rad(180) - theta - V1;
  double thetadH = thetad - V2;
  double T2 = sqrt(square(L4) + square(T1) - 2 * L4 * T1 * cos(thetadH));
  double V12 = acos((square(T1) + square(T2) - square(L4)) / (2 * T1 * T2));
  double thetaV = V1 + V12;
  double thetaTA = acos((square(T2) + square(L2) - square(L3)) / (2 * T2 * L2));

  double thetaL = thetaTA + thetaV;
  double T3 = sqrt(square(L1) + square(L2) - 2 * L1 * L2 * cos(thetaL));
  double thetaTB = acos((square(T3) + square(L2) - square(L1)) / (2 * T3 * L2));
  double thetaTC = deg2rad(180) - thetaL - thetaTB;
  double thetaXY = theta - thetaTC;
  float x = cos(thetaXY) * T3;
  float y = sqrt(square(T3) - square(x));
  xCoord = x;
  yCoord = y;
  //std::cout << "Forward kinematics X: " << x << ", Y: " << y << std::endl;
}

*/

void Controller::ForwardKinematics(float theta, float thetad)
{
    float AB1 = L1*cos(theta);
    float AB2 = L1*sin(theta);
    float AD1 = L0-L1*cos(thetad);
    float AD2 = L1*sin(thetad);
    float L3 = sqrt(pow((AD1-AB1),2)+pow((AD2-AB2),2));
    float phi2 = acos((L3/2)/L2);
    float phi1 = atan2(AD2-AB2,AD1-AB1);
        
    float xCoord = AB1 + L2*cos(phi1+phi2);
    float yCoord = AB1 + L2*sin(phi1+phi2);

 
}

void Controller::InverseKinematics(float x, float y, bool PosCheck)
{
  float L3 = L2;
  float L4 = L1;
  float LeftAngle{};
  float RightAngle{};
  float AngleRightActuator;
  float AngleLeftActuator;
  float AngleRightActuatorCompensated;
  float AngleLeftActuatorCompensated;
  //x += L0/2;
  //y += 0.60;
  if (x >= 0)
  {
    LeftAngle = abs(atan(y / x));
  }
  if (x < 0)
  {
    LeftAngle = M_PI - abs(atan(y / x));
  }

  float d = sqrt((x * x) + (y * y));

  float alpha = acos(((d * d) + (L1 * L1) - (L2 * L2)) / (2 * d * L1));

  AngleLeftActuator = alpha + LeftAngle;
  AngleLeftActuatorCompensated = AngleLeftActuator - (90 * M_PI / 180);

  if (x > L0)
  {
    RightAngle = M_PI - abs(atan(y / (x - L0)));
  }
  else if (x <= L0)
  {
    RightAngle = atan(y / (L0 - x));
  }
  float dodd = sqrt(((L0 - x) * (L0 - x)) + (y * y));

  float alphaodd = acos(((dodd * dodd) + (L4 * L4) - (L3 * L3)) / (2 * dodd * L4));

  AngleRightActuator = alphaodd + RightAngle;

  AngleRightActuatorCompensated = (AngleRightActuator * (-1)) + (90 * M_PI / 180);
  //std::cout << "Left: " << AngleLeftActuatorCompensated << ", Right: " << AngleRightActuatorCompensated << std::endl;
  MotorR->setPosition(AngleRightActuatorCompensated);
  MotorL->setPosition(AngleLeftActuatorCompensated);
  
  //robot->step(TIME_STEP);

  if(PosCheck)
  {
    waitForrobotToreachPos(x, y);
  }
  
  
}
void Controller::line(float x0, float y0)
{
  
  Controller::ForwardKinematics(getLpos(), getRpos());
  //robot->step(TIME_STEP);
  float x, y;
  x = xCoord;
  y = yCoord;
  float L0 = 0.4;
  float deltaX = x0 - x;
  float deltaY = y0 - y;
  float linelength{sqrt(square(deltaX) + square(deltaY))};

  float a = (deltaY) / (deltaX);

  float b = y - a * x;

  ////std::cout << "x: " << x << ", y: " << y << ", x0: " << x0 << ", y0: " << y0 << ", a: " << a << ", b: "
  //<< b << ", x1: " << x1 << ", y1: " << y1 << ", Line length: " << linelength << std::endl;

  int divisions = linelength * 50;
  float x1 = (deltaX / divisions * 1) + x;
  float y1 = a * x1 + b;
  Controller::InverseKinematics(x1, y1, false);
  for (size_t i = 2; i < divisions; i++)
  {
    
    float x2 = (deltaX / divisions * i) + x;
    float y2 = a * x2 + b;
    //std::cout << x2 << ", " << y2 << ", " << divisions << std::endl;
    Controller::InverseKinematics(x2, y2, false);
    //float y1 = (a * (deltaX/divisions*i) + x + b);
    
    //InverseKinematics(x1, y1);
    
    //std::cout << x1 << "    " << divisions << "    " << i << std::endl;
  }
  float x3 = (deltaX / divisions * divisions) + x;
  float y3 = a * x3 + b;
  Controller::InverseKinematics(x3, y3, false);
}

//Returns coordinates of robot.
float* Controller::ReturnCoord(){
  float* coord = new float[2];

  Controller::ForwardKinematics(Controller::getLpos(), Controller::getRpos());
  
  coord[0] = xCoord;
  coord[1] = yCoord;

  return coord;
}

Controller::Controller()
{
  Controller::robot = new Robot();

  Controller::MotorL = robot->getMotor("MotorL");
  Controller::MotorR = robot->getMotor("MotorR");

  Controller::PosL = robot->getPositionSensor("PosL");
  Controller::PosR = robot->getPositionSensor("PosR");
  // create the Robot instance.
  Controller::PosR->enable(TIME_STEP);
  Controller::PosL->enable(TIME_STEP);
}



