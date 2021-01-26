// File:          navigation.h
// Date: 26/01/2021
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#ifndef NAVIGATION
#define NAVIGATION

enum NavState{def, doubleChecking, initialScan, movingTo, findingLost};

class Navigation {
public:
  Navigation(Robot *_robot, DataBase *_dataBase, int _timeStep);
  ~Navigation() {}
  
  // get functions
  bool AmIRed(){ return iAmRed; }
  vec GetPosition(){ return position; }
  double GetBearing(){ return bearing; }
  vec PositionInFront(){
    vec delta = {(float)(0.15*cos(bearing)), (float)(0.15*sin(bearing))};
    return position + delta;
  }
  
  // main functions
  void BeginStep();
  void EndStep(float leftSpeed, float rightSpeed);
  
private:
  Robot *robot;
  DataBase *dataBase;
  int timeStep;
  Motor *wheels[WHEELS_N];
  GPS *gps;
  DistanceSensor *distanceSensors[SENSORS_N];
  const float dSensorsOff[SENSORS_N] = {0.1, 0.1}; // distances between the centre of the robot and distance sensors
  bool iAmRed;
  
  NavState state;
  
  // double checking
  int dCingN;
  float dCTime = 0.5; // seconds
  vec dCPos;
  
  // finding lost
  bool fLTurningRight;
  double fLTurnTo;
  
  double distances[SENSORS_N];
  
  vec position;
  double bearing;
  
  vec destination;
  Colour destColour;
  unsigned short destIndex;
  bool GetDestination(){
    return dataBase->GetBlock(destColour, destIndex, &destination);
  }
  
  const char dsNames[SENSORS_N][10] = {"ds_front", "ds_rear"};
  const char wlNames[WHEELS_N][10] = {"wheel1", "wheel2"};
  
  // gets the gps position and stores it as a vector in the 2D plane
  void ReadGPS(){
    double values[3];
    memcpy(values, gps->getValues(), 3*sizeof(double));
    position = {(float)values[2], (float)values[0]};
  }
  void ReadDistanceSensors(){
    for(int i=0; i<SENSORS_N; i++){
      distances[i] = dSensorsOff[i] + distanceSensors[i]->getValue();
    }
  }
  
};

#endif