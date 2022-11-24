#pragma once
#include <vector>
#include "Point.h"
#include <math.h>
#include </home/dadi/p5_ws/src/P5_asphalt_robot/UDP/include/UDP_Com.h>
#include <iostream>
#include <vector>

class MotionPlanning
{
private:

    long double a[2][5];
    float DP[3][4];
    long double LastVel[2];
    int numGoals = 0;
    bool PolynomialPlanning = true;

    long double CalculateDesiredVelocity(double x1, double y1, double x2, double y2, double x3, double y3, double t1, double t2, char coor);

public:

    //Constructor
    MotionPlanning(){
        LastVel[0] = 0;
        LastVel[1] = 0;
    }
    bool EndPoint = false;
    bool debug = false;

    void EraseOldPoints(std::vector<std::vector<float>>* GoalsVector, double time);
    void InitiateTestData();
    void ComputeA();
    void ComputeALinear();
    float* GetPosition(double t);
    float* GetVelocity(float t);
    float* GetAcceleration(float t);
    void Plan(std::vector<Point> GoalsVector, double PresentTime);
};
