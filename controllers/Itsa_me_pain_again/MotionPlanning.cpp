#include "include/MotionPlanning.h"


void MotionPlanning::Plan(std::vector<Point> GoalsVector, double PresentTime){

    numGoals = GoalsVector.size();

    if (numGoals > 0){
        // if ((DP[0][0] != GoalsVector.at(0).x || DP[0][1] != GoalsVector.at(0).y || DP[0][2] != GoalsVector.at(0).y) && PresentTime > DP[0][2])
        {
            if (numGoals == 0) { //If Goal vector is empty, then do nothing.
                DP[0][0] = 0; //X0
                DP[0][1] = 0; //Y0
                DP[0][2] = 0; //T0
                DP[0][3] = 0;
                
                DP[1][0] = 0; //X1
                DP[1][1] = 0; //Y1
                DP[1][2] = 0; //T1
                DP[1][3] = 0;

                DP[2][0] = 0;
                DP[2][1] = 0;
                DP[2][2] = 0;
                DP[2][3] = 0;
            } else if (numGoals == 1) { //If one goal is present, move linearly to goal.
                DP[0][0] = GoalsVector.at(0).x; //X1
                DP[0][1] = GoalsVector.at(0).y; //Y1
                DP[0][2] = GoalsVector.at(0).goalT; //T1
                DP[0][3] = GoalsVector.at(0).shift;
                
                DP[1][0] = 0; //X1
                DP[1][1] = 0; //Y1
                DP[1][2] = 0; //T1
                DP[1][3] = 0;

                DP[2][0] = 0;
                DP[2][1] = 0;
                DP[2][2] = 0;
                DP[2][3] = 0;

                a[0][0] = DP[0][0];
                a[0][1] = 0;
                a[0][2] = 0;
                a[0][3] = 0;

                a[1][0] = DP[0][1];
                a[1][1] = 0;
                a[1][2] = 0;
                a[1][3] = 0;

                if(debug){
                    std::cout << "Goal0: " << DP[0][0] << ", " << DP[0][1] << ", t: " << DP[0][2] << ", EndPoint: " << DP[0][3] << std::endl;
                }
                
            } else if (numGoals == 2) { //If two goals are present, move through polynomial to goal, but assume some velocity.
                DP[0][0] = GoalsVector.at(0).x; //X0
                DP[0][1] = GoalsVector.at(0).y; //Y0
                DP[0][2] = GoalsVector.at(0).goalT; //T0
                DP[0][3] = GoalsVector.at(0).shift;
                
                DP[1][0] = GoalsVector.at(1).x; //X1
                DP[1][1] = GoalsVector.at(1).y; //Y1
                DP[1][2] = GoalsVector.at(1).goalT; //T1
                DP[1][3] = GoalsVector.at(1).shift;

                DP[2][0] = 0;
                DP[2][1] = 0;
                DP[2][2] = 0;
                DP[2][3] = 0;

                if(debug){
                    std::cout << "Goal0: " << DP[0][0] << ", " << DP[0][1] << ", t: " << DP[0][2] << ", EndPoint: " << DP[0][3] << std::endl;
                    std::cout << "Goal1: " << DP[1][0] << ", " << DP[1][1] << ", t: " << DP[1][2] << ", EndPoint: " << DP[1][3] << std::endl;
                }

                ComputeA();
            } else if (numGoals > 2) { //If more than two goals are present, move through polynomial to goal.
                DP[0][0] = GoalsVector.at(0).x; //X0
                DP[0][1] = GoalsVector.at(0).y; //Y0
                DP[0][2] = GoalsVector.at(0).goalT; //T0
                DP[0][3] = GoalsVector.at(0).shift;
                
                DP[1][0] = GoalsVector.at(1).x; //X1
                DP[1][1] = GoalsVector.at(1).y; //Y1
                DP[1][2] = GoalsVector.at(1).goalT; //T1
                DP[1][3] = GoalsVector.at(1).shift;

                DP[2][0] = GoalsVector.at(2).x;
                DP[2][1] = GoalsVector.at(2).y;
                DP[2][2] = GoalsVector.at(2).goalT;
                DP[2][3] = GoalsVector.at(2).shift;

                if(debug){
                    std::cout << "Goal0: " << DP[0][0] << ", " << DP[0][1] << ", t: " << DP[0][2] << ", EndPoint: " << DP[0][3] << std::endl;
                    std::cout << "Goal1: " << DP[1][0] << ", " << DP[1][1] << ", t: " << DP[1][2] << ", EndPoint: " << DP[1][3] << std::endl;
                    std::cout << "Goal2: " << DP[2][0] << ", " << DP[2][1] << ", t: " << DP[2][2] << ", EndPoint: " << DP[2][3] << std::endl;
                }

                ComputeA();
            }

        }
        
    }
    
   
}

float* MotionPlanning::GetPosition(double t){
    static float Position[2];
    Position[0] = a[0][0] + (a[0][1]*t) + (a[0][2]*(t*t)) + (a[0][3]*(t*t*t));
    Position[1] = a[1][0] + (a[1][1]*t) + (a[1][2]*(t*t)) + (a[1][3]*(t*t*t));
    return Position;
}

float* MotionPlanning::GetVelocity(float t){
    static float Velocity[2];
    Velocity[0] = a[0][1] + (2*a[0][2]*t) + (3*a[0][3]*(t*t));
    Velocity[1] = a[1][1] + (2*a[1][2]*t) + (3*a[1][3]*(t*t));
    return Velocity;
}

float* MotionPlanning::GetAcceleration(float t){
    static float Acceleration[2];
    Acceleration[0] = (2*a[0][2]) + (6*a[0][3]*t);
    Acceleration[1] = (2*a[1][2]) + (6*a[1][3]*t);
    return Acceleration;
}

void MotionPlanning::InitiateTestData(){
    DP[0][0] = 0.8; //X0
    DP[0][1] = 1.0; //Y0
    DP[0][2] = 0;   //T0
    DP[0][3] = 0;   //Shift
    
    DP[1][0] = -0.8;//X1
    DP[1][1] = 1.0; //Y1
    DP[1][2] = 1;   //T1
    DP[1][3] = 0;   //Shift

    DP[2][0] = 5;   //X2
    DP[2][1] = -3;  //Y2
    DP[2][2] = 2;   //T2
    DP[2][3] = 0;   //Shift

    numGoals = 3;
}

long double MotionPlanning::CalculateDesiredVelocity(double x1, double y1, double x2, double y2, double x3, double y3, double t1, double t2, char coor){
    
    long double Xdif = x2 - x1;
    long double Ydif = y2 - y1;
    long double distance = sqrt((Xdif*Xdif) + (Ydif*Ydif));
    long double timeDif = t2 - t1;
    long double DesiredVelocity = distance/timeDif;
    
    long double DesiredDirX = x3 - x1;
    long double DesiredDirY = y3 - y1;
    long double DesiredDirLength = sqrt((DesiredDirX*DesiredDirX) + (DesiredDirY*DesiredDirY));

    DesiredDirX = DesiredDirX/DesiredDirLength;
    DesiredDirY = DesiredDirY/DesiredDirLength;
    if (coor == 'x')
    {
        DesiredVelocity = DesiredVelocity * DesiredDirX;
    } else if (coor == 'y')
    {
        DesiredVelocity = DesiredVelocity * DesiredDirY;
    }
    
    return DesiredVelocity;
}

void MotionPlanning::ComputeA(){

    if (PolynomialPlanning){
        long double velX = 0; 
        long double velY = 0;

        long double velXi = LastVel[0];
        long double velYi = LastVel[1];

        //Calculate desired velocities in X and Y directions.
        if (numGoals > 2) //If more than two goals are present, use point 1 and 3 to determine direction in point 2.
        {   
            
            if (DP[0][3] == 1 && DP[1][3] == 1)
            {
                velX = 0;
                velY = 0;
                velXi = 0;
                velYi = 0;
            } else if (DP[1][3] == 1 && DP[2][3] == 1){
                velX = 0;
                velY = 0;
            } else if (DP[2][3])
            {   
                long double Ax3 = DP[0][0] + 2*(DP[1][0] - DP[0][0]);
                long double Ay3 = DP[0][1] + 2*(DP[1][1] - DP[0][1]);
                velX = CalculateDesiredVelocity(DP[0][0], DP[0][1], DP[1][0],DP[1][1], Ax3, Ay3 , DP[0][2], DP[1][2], 'x');
                velY = CalculateDesiredVelocity(DP[0][0], DP[0][1], DP[1][0],DP[1][1], Ax3, Ay3 , DP[0][2], DP[1][2], 'y');
            } else {
                velX = CalculateDesiredVelocity(DP[0][0], DP[0][1], DP[1][0],DP[1][1],DP[2][0], DP[2][1], DP[0][2], DP[1][2], 'x');
                velY = CalculateDesiredVelocity(DP[0][0], DP[0][1], DP[1][0],DP[1][1],DP[2][0], DP[2][1], DP[0][2], DP[1][2], 'y');
            }
            
        } else if (numGoals == 2 && DP[0][3] == 1 && DP[1][3] == 1){
            velX = 0;
            velY = 0;
            velXi = 0;
            velYi = 0;
        } else if (numGoals == 2){   
            velX = 0;
            velY = 0;
        }
        
        long double Goal1Time = 0;
        long double Goal2Time = DP[1][2] - DP[0][2];
        
        // define x
        a[0][0] = (DP[0][0]*(Goal2Time*Goal2Time*Goal2Time) - DP[1][0]*(Goal1Time*Goal1Time*Goal1Time) - velX*(Goal2Time*Goal2Time)*(Goal1Time*Goal1Time) + velXi*(Goal2Time*Goal2Time)*(Goal1Time*Goal1Time) + 3*DP[1][0]*Goal2Time*(Goal1Time*Goal1Time) - 3*DP[0][0]*(Goal2Time*Goal2Time)*Goal1Time + velX*Goal2Time*(Goal1Time*Goal1Time*Goal1Time) - velXi*(Goal2Time*Goal2Time*Goal2Time)*Goal1Time)/((Goal2Time - Goal1Time)*((Goal2Time*Goal2Time) - 2*Goal2Time*Goal1Time + (Goal1Time*Goal1Time)));
        a[0][1] = (velXi*(Goal2Time*Goal2Time*Goal2Time) - velX*(Goal1Time*Goal1Time*Goal1Time) - 6*DP[1][0]*Goal2Time*Goal1Time + 6*DP[0][0]*Goal2Time*Goal1Time - velX*Goal2Time*(Goal1Time*Goal1Time) + 2*velX*(Goal2Time*Goal2Time)*Goal1Time - 2*velXi*Goal2Time*(Goal1Time*Goal1Time) + velXi*(Goal2Time*Goal2Time)*Goal1Time)/((Goal2Time - Goal1Time)*((Goal2Time*Goal2Time) - 2*Goal2Time*Goal1Time + (Goal1Time*Goal1Time)));
        a[0][2] = (3*DP[1][0]*Goal2Time - 3*DP[0][0]*Goal2Time + 3*DP[1][0]*Goal1Time - 3*DP[0][0]*Goal1Time - velX*(Goal2Time*Goal2Time) - 2*velXi*(Goal2Time*Goal2Time) + 2*velX*(Goal1Time*Goal1Time) + velXi*(Goal1Time*Goal1Time) - velX*Goal2Time*Goal1Time + velXi*Goal2Time*Goal1Time)/((Goal2Time - Goal1Time)*((Goal2Time*Goal2Time) - 2*Goal2Time*Goal1Time + (Goal1Time*Goal1Time)));
        a[0][3] = -(2*DP[1][0] - 2*DP[0][0] - velX*Goal2Time - velXi*Goal2Time + velX*Goal1Time + velXi*Goal1Time)/((Goal2Time - Goal1Time)*((Goal2Time*Goal2Time) - 2*Goal2Time*Goal1Time + (Goal1Time*Goal1Time)));

        

        // define y
        a[1][0] = (DP[0][1]*(Goal2Time*Goal2Time*Goal2Time) - DP[1][1]*(Goal1Time*Goal1Time*Goal1Time) - velY*(Goal2Time*Goal2Time)*(Goal1Time*Goal1Time) + velYi*(Goal2Time*Goal2Time)*(Goal1Time*Goal1Time) + 3*DP[1][1]*Goal2Time*(Goal1Time*Goal1Time) - 3*DP[0][1]*(Goal2Time*Goal2Time)*Goal1Time + velY*Goal2Time*(Goal1Time*Goal1Time*Goal1Time) - velYi*(Goal2Time*Goal2Time*Goal2Time)*Goal1Time)/((Goal2Time - Goal1Time)*((Goal2Time*Goal2Time) - 2*Goal2Time*Goal1Time + (Goal1Time*Goal1Time))); 
        a[1][1] = (velYi*(Goal2Time*Goal2Time*Goal2Time) - velY*(Goal1Time*Goal1Time*Goal1Time) - 6*DP[1][1]*Goal2Time*Goal1Time + 6*DP[0][1]*Goal2Time*Goal1Time - velY*Goal2Time*(Goal1Time*Goal1Time) + 2*velY*(Goal2Time*Goal2Time)*Goal1Time - 2*velYi*Goal2Time*(Goal1Time*Goal1Time) + velYi*(Goal2Time*Goal2Time)*Goal1Time)/((Goal2Time - Goal1Time)*((Goal2Time*Goal2Time) - 2*Goal2Time*Goal1Time + (Goal1Time*Goal1Time)));
        a[1][2] = (3*DP[1][1]*Goal2Time - 3*DP[0][1]*Goal2Time + 3*DP[1][1]*Goal1Time - 3*DP[0][1]*Goal1Time - velY*(Goal2Time*Goal2Time) - 2*velYi*(Goal2Time*Goal2Time) + 2*velY*(Goal1Time*Goal1Time) + velYi*(Goal1Time*Goal1Time) - velY*Goal2Time*Goal1Time + velYi*Goal2Time*Goal1Time)/((Goal2Time - Goal1Time)*((Goal2Time*Goal2Time) - 2*Goal2Time*Goal1Time + (Goal1Time*Goal1Time)));
        a[1][3] = -(2*DP[1][1] - 2*DP[0][1] - velY*Goal2Time - velYi*Goal2Time + velY*Goal1Time + velYi*Goal1Time)/((Goal2Time - Goal1Time)*((Goal2Time*Goal2Time) - 2*Goal2Time*Goal1Time + (Goal1Time*Goal1Time)));   
        
        LastVel[0] = velX;
        LastVel[1] = velY;

        if (debug){
            std::cout << "X formula: " << a[0][0] << " + " << a[0][1] << "*t + " << a[0][2] << "*t² + " << a[0][3] << "*t³" << std::endl;
            std::cout << "Y formula: " << a[1][0] << " + " << a[1][1] << "*t + " << a[1][2] << "*t² + " << a[1][3] << "*t³" << std::endl;
        }

    } else {

        a[0][0] = (long double)DP[1][0];
        a[0][1] = 0;
        a[0][2] = 0;
        a[0][3] = 0;

        a[1][0] = (long double)DP[1][1];
        a[1][1] = 0;
        a[1][2] = 0;
        a[1][3] = 0;
        
        if (debug){
            std::cout << "X formula: " << a[0][0] << " + " << a[0][1] << "*t + " << a[0][2] << "*t² + " << a[0][3] << "*t³" << std::endl;
            std::cout << "Y formula: " << a[1][0] << " + " << a[1][1] << "*t + " << a[1][2] << "*t² + " << a[1][3] << "*t³" << std::endl;
        }
    }
}


