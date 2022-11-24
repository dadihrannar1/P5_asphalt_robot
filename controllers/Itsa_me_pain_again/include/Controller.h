//#pragma once

#include </usr/local/webots/include/controller/cpp/webots/Robot.hpp>
#include </usr/local/webots/include/controller/cpp/webots/Motor.hpp>
#include </usr/local/webots/include/controller/cpp/webots/PositionSensor.hpp>
#include <math.h>
class Controller
{
private:

    void ForwardKinematics(float theta, float thetad);
    float square(float input);
    float deg2rad(float angle);
    float getLpos();
    float getRpos();
    void waitForrobotToreachPos(float LeftAngle, float RightAngle);
    void InverseKinematics(float x, float y, bool PosCheck);
    void line(float x0, float y0);
    webots::Motor *MotorR;
    webots::Motor *MotorL;
    webots::PositionSensor *PosL;
    webots::PositionSensor *PosR;
    float xCoord{};
    float yCoord{}; 
public:

    Controller(/* args */);
    void LinearMove(float x, float y);
    void FastMove(float x, float y, bool PosCheck);
    float* ReturnCoord();

    //Dimensioner på robot
    float L0 = 0.25;    //Distancen imellem motorerne
    float L1 = 0.620;     //Længden på det første led.
    float L2 = 0.745;   //Længden på det andet led.
   // ~Controller();
    
    webots::Robot *robot; //whats the need?
};
