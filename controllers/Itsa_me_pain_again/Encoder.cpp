#include "include/Encoder.h"
//std::vector<std::vector<float>> MotionVec;

// Returns the appropriate goals for trajectory planning.
std::vector<Point> Encoder::getGoalsForTrajectoryPlanning(double time){
    
  std::vector<Point> TrajectoryGoals; //coordinates, final timestamp, and crack detected  
    
  int size = Goals.size();
  if (size > 2){
    //If point 0 and 1 are within workspace:
    if(checkWorkspace(PresentPosition(Goals.at(0),time),0.01,time) == true && checkWorkspace(PresentPosition(Goals.at(1),time),0.01,time) == true){
      if(debug){
        std::cout << "goal.at(0) and goal.at(1) are within workspace" << std::endl;
      }
      
      double timeItr = time;
      for (size_t i = 0; i < 3; i++)
      {
        Point goal = Goals.at(i);
        timeItr+=timeDelta(i);
        goal.goalT = timeItr;
        goal.y = YAtTime(Goals.at(i), goal.goalT);
        TrajectoryGoals.push_back(goal);
      }
      Goals.erase(Goals.begin());
    }

    //If point 0 are within workspace, and point 1 is outside.
    else if(checkWorkspace(PresentPosition(Goals.at(0),time),0.01,time) == true && checkWorkspace(PresentPosition(Goals.at(1),time),0.01,time) == false){

      if(debug){
        std::cout << "goal.at(0) is within workspace and goal.at(1) is outside workspace" << std::endl;
      }

      Point goal0 = Goals.at(0);
      Point goal1 = Goals.at(1);
      Point goal2 = Goals.at(2);

      goal0.goalT = time;
      goal1.goalT = timeAtY(goal1, -(-(cos(ActuatorLimit)*L1) - L2)) + CorrectionFactor;
      goal2.goalT = timeAtY(goal2, -(-(cos(ActuatorLimit)*L1) - L2)) + CorrectionFactor;

      goal0.y = YAtTime(Goals.at(0), goal0.goalT);
      goal1.y = YAtTime(Goals.at(1), goal1.goalT);
      goal2.y = YAtTime(Goals.at(2), goal2.goalT);

      TrajectoryGoals.push_back(goal0);
      TrajectoryGoals.push_back(goal1);
      TrajectoryGoals.push_back(goal2);

      Goals.erase(Goals.begin());
      
    }

    //If point 0 and 1 are outside workspace.
    else if(checkWorkspace(PresentPosition(Goals.at(0),time),0.01,time) == false && checkWorkspace(PresentPosition(Goals.at(1),time),0.01,time) == false){
      if(debug){
        std::cout << "goal.at(0) and goal.at(1) are outside workspace" << std::endl;
      }
      
      Point goal0 = Goals.at(0);
      goal0.goalT = timeAtY(goal0, -(-(cos(ActuatorLimit)*L1) - L2)) + 0.01;
      goal0.y = YAtTime(Goals.at(0), goal0.goalT);
      goal0.shift = 1;
      TrajectoryGoals.push_back(goal0);     
    }
  }
    
  if (size == 2){

    double timeItr = time;
      for (size_t i = 0; i < 2; i++)
      {
        Point goal = Goals.at(i);
        timeItr+=timeDelta(i);
        goal.goalT = timeItr;
        goal.y = YAtTime(Goals.at(i), goal.goalT);
        TrajectoryGoals.push_back(goal);
      }
    
      Goals.erase(Goals.begin());

  } else if (size == 1){
    double timeItr = time;
    
      for (size_t i = 0; i < 1; i++)
      {
        Point goal = Goals.at(i);
        timeItr+=timeDelta(i);
        goal.goalT = timeItr;
        goal.y = YAtTime(Goals.at(i), goal.goalT);
        TrajectoryGoals.push_back(goal);
      }
    
      Goals.erase(Goals.begin());
  }

  return TrajectoryGoals;
  
}

double Encoder::timeAtY(Point point, float y){
  double distance = point.y + y + DistVehicle;
  double tAty = point.frameT+(distance/getVelocity()); // The time at which the point was found + The time it takes to get to y.
  return tAty;
}

double Encoder::YAtTime(Point point, double t){
  double y = point.y-((t-point.frameT)*getVelocity());
  return y;
}

//Returns true if a point is within the workspace, by a margin.
bool Encoder::checkWorkspace(Point point, float margin, double time){

  float ButtomBorder = -(cos(ActuatorLimit)*L1) - L2 - margin;
  float part1 = L1+L2;
  float part2 = 0.5-(L0/2);
  float topBorder = -sqrt((part1*part1) - (part2*part2)) + margin ;

  // std::cout << "ButtomBorder:" << ButtomBorder-DistVehicle << " TopBorder:" << topBorder-DistVehicle << std::endl;

  if(point.x < (1 - margin) && point.x > (0 + margin) && point.y > topBorder-DistVehicle && point.y < ButtomBorder-DistVehicle){
  // if(point.x < (1 - margin) && point.x > (0 + margin) && point.y > topBorder-DistVehicle && point.y < ButtomBorder){
    return true;
  } else {
    if(debug){
      std::cout << "Goal is outside workspace" << std::endl;
    }
    return false;
  }
}

//Returns true if a point is beyond the treshold
bool Encoder::beyondThreshold(Point point, float threshold, double time){
  // If point is beyond specified threshold, return true;
  if (point.y < (-DistVehicle - startThreshold)){
    if (debug){
      std::cout << "Goal is beyond threshold" << std::endl;
    }
    return true;
  } else {
    return false;
  }
}

//Returnerer aktuel position i camera workspace.
Point Encoder::PresentPosition(Point point, double time){ 
  //return delta tid * hastighed - her beregner vi hvor meget framen har rykket sig ift. det tidspunkt billedet er taget.
  Point NewY = point;
  double timePassed = time - point.frameT;
  NewY.y = point.y - (timePassed*getVelocity());

  return NewY;
}

double Encoder::getVelocity(){
    //Do sketchy shit in webots
    //Webots_encoder = Velocity: 
    return VehicleVelocity*0.277777; // 1km/h = 0.277777 m/s
}

//Returns the timedifference between goal i and i-1. If goal = 0, then timeDelta returns 0;
double Encoder::timeDelta(int goal){
  double time = 0;

  if (goal > 0 && Goals.size() > 1){
    float xDif = Goals.at(goal-1).x - Goals.at(goal).x;
    float yDif = Goals.at(goal-1).y - Goals.at(goal).y; 
    double distance = sqrt((xDif*xDif) + (yDif * yDif));

    time = distance/Velocity;
  }
  
  return time;
}

//add goal to goalsvector. Takes in goal with xy in camera pixel values!
void Encoder::addGoal(Point goal){
  Point PushGoal = goal;
  float* pos = ConvertPixToMeter(goal.x, goal.y);
  PushGoal.x = pos[0];
  PushGoal.y = pos[1];
  
  Goals.push_back(PushGoal);
}

float* Encoder::ConvertPixToMeter(int X, int Y){
  float *PosXY = new float[2];
  
  float fovX = 2 * atan(SensorXSize / (2*focallength));// * 57.2957; for degrees
  float fovY = 2 * atan(SensorYSize / (2*focallength));// * 57.2957; for degrees

  float Xmeter = sin(fovY/2)*2*CameraMountHeight;
  float Ymeter = sin(fovX/2)*2*CameraMountHeight;

  PosXY[0] = (X*(Xmeter/ResX));
  PosXY[1] = (Y*(Ymeter/ResY));

  // std::cout << "ConvertPixToMeter: X:" << PosXY[0] << " Y:" << PosXY[1] << std::endl;
  return PosXY;
}

void Encoder::setMeasurements(float dist, float length1, float length2){
  L0 = dist;
  L1 = length1;
  L2 = length2;
}

//Visualiserer alle datapunkter i goals vektoren.
void Encoder::visualizePoints(webots::Display *display, double time, float* robotPos){

  //Fill display with black
  display->setColor(std::stoi("000000",0,16));
  display->setAlpha(0.5);
  display->fillRectangle(0,0,320,1600);

  //Create text
  display->setAlpha(1);
  display->setFont("Arial", 20, 0);
  display->setColor(std::stoi("0000FF",0,16));
  display->drawText("Crack Display", 10, 10);

  //Draw cracks with blue
  display->setAlpha(1);
  display->setColor(std::stoi("0000FF",0,16));
  
  for (auto &&GoalPoint : GoalsHistory)
  {
    float ydiff = abs(YAtTime(GoalPoint, time) - robotPos[1]);
    float xdiff = abs(GoalPoint.x - robotPos[0]);
    float dist = sqrt((ydiff*ydiff)+(xdiff*xdiff));
    
    if(GoalPoint.goalT > dist){
      GoalPoint.goalT = dist;
    }
    
    if(GoalPoint.goalT < 0.025){
      display->setColor(std::stoi("FF0000",0,16));
    } else {
      display->setColor(std::stoi("0000FF",0,16));
    }
    
    int* coordinates = getDisplayCoordinates(GoalPoint, time);
    display->fillOval(coordinates[0], coordinates[1], 2, 2);
    
  }
  

}

//Returnerer en int vector med display-koordinaterne til et punkt.
int* Encoder::getDisplayCoordinates(Point point, double time){
  int *Coord = new int[2];
  
  //X-coordinate
  Coord[0] = (int)(point.x * 320);
  
  //Y-coordinate
  //y=800 equals y=0 in camera frame
  Coord[1] = 800 + (int)(320*YAtTime(point, time));

  return Coord;
  
}

void Encoder::visualizeEndEffector(webots::Display *display, float* robotPos, std::string color, int size){
  
  int *Coord = new int[2];
  
  //X-coordinate
  Coord[0] = (int)(robotPos[0] * 320);
  
  //Y-coordinate
  //y=800 equals y=0 in camera frame
  Coord[1] = 800 + (int)(320*robotPos[1]);
  display->setAlpha(1);
  display->setColor(std::stoi(color,0,16));

  display->fillOval(Coord[0], Coord[1], size, size);
}