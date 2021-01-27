// File:          navigation.h
// Date: 26/01/2021
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#ifndef NAVIGATION
#define NAVIGATION

#include "header.h"
#include "database.h"
#include "state.h"

class Navigation {
public:
  // initialises all inputs and outputs, and works out which robot
  // 	this instance of the controller is controlling.
  //	then sets the navigation state to initial scan
  Navigation(Robot *_robot, DataBase *_dataBase, int _timeStep);
  ~Navigation() {}
  
  // get and set functions (mainly used by state objects)
  char GetC(){ return names[iAmRed]; }
  int GetTS(){ return timeStep; }
  bool IAmRed(){ return iAmRed; }
  DataBase *GDB(){ return dataBase; }
  vec GetPosition(){ return position; }
  vec GetDestination(){ RetrieveDBDestination(); return destination; }
  float GetDistance(int which){ return distances[which]; }
  void SetBearing(double newBearing){ bearing = newBearing; }
  double GetBearing(){ return bearing; }
  vec PositionInFront(){ // position where the block should be in front
    vec delta = {(float)(0.15*cos(bearing)), (float)(0.15*sin(bearing))};
    return position + delta;
  }
  
  bool DBGetDestination();
  bool DBLogReading(bool canConfirm);
  
  // main functions
  // runs the code to be called every step
  void Run();
  
  // sets the motor speeds and integrates them to modify the bearing estimate
  // generally called by state objects to wrap up at the end of each step
  void EndStep(float leftSpeed, float rightSpeed);
  
private:
  Robot *robot;
  DataBase *dataBase;
  int timeStep;
  
  // identification stuff
  bool iAmRed;
  const char names[2] = {'b', 'r'};
  
  // gps stuff
  GPS *gps;
  
  // wheel stuff
  const char wlNames[WHEELS_N][10] = {"wheel1", "wheel2"};
  Motor *wheels[WHEELS_N];
  
  // distance sensor stuff
  const char dsNames[SENSORS_N][10] = {"ds_front", "ds_rear"};
  DistanceSensor *distanceSensors[SENSORS_N];
  const float dSensorsOff[SENSORS_N] = {0.1, 0.1}; // distances between the centre of the robot and distance sensors
  double distances[SENSORS_N]; // stores distance sensor readings
  
  // state stuff
  StateManager *stateManager;
  
  // variables
  vec position;
  double bearing;
  
  // destination stuff
  vec destination;
  Colour destColour; // location in database of destination block
  unsigned short destIndex; // location in database of destination block
  bool RetrieveDBDestination(){ // gets the position of the destination block from the database
    return dataBase->GetBlock(destColour, destIndex, &destination);
  }
  
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