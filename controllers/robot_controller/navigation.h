// File:          navigation.h
// Date: 26/01/2021
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#ifndef NAVIGATION
#define NAVIGATION

#include "header.h"
#include "database.h"

// Current state system states
// 'def':				Asking the database for a destination. Will switch to 'movingTo' upon receiving one.
//						Otherwise will turn slowly, scanning information for the database
// 'doubleChecking':	Moving straight forwards from a remembered initial position. Re-calibrates bearing
//						after 'dCTime' seconds, then goes back to 'def'
// 'initialScan':		Scans a bit of 90 degrees downwards, logging questions into the database
//						('canConfirm' = 'false'). Switches to 'def' when finished
// 'movingTo':			Turns towards, then moves towards destination. Will switch to 'findingLost' if
//						distances sensor reading becomes unexpected
// 'findingLost':		Wiggles left and right to increasing angles until distance sensor reads distance
//						to destination !!! does not have a way of getting out of this is if it never reads
//						the right thing
enum NavState{def, doubleChecking, initialScan, movingTo, findingLost};

class Navigation {
public:
  // initialises all inputs and outputs, and works out which robot
  // 	this instance of the controller is controlling.
  //	then sets the navigation state to initial scan
  Navigation(Robot *_robot, DataBase *_dataBase, int _timeStep);
  ~Navigation() {}
  
  // get functions
  bool AmIRed(){ return iAmRed; }
  vec GetPosition(){ return position; }
  double GetBearing(){ return bearing; }
  vec PositionInFront(){ // position where the block should be in front
    vec delta = {(float)(0.15*cos(bearing)), (float)(0.15*sin(bearing))};
    return position + delta;
  }
  
  // main functions
  // runs the code according to the state, then calls 'EndStep' with appropriate wheel speeds
  void BeginStep();
  
  // sets the motor speeds and integrates them to modify the bearing estimate
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
  
  NavState state; // state, description is by enumeration above
  
  // variables for double checking
  int dCingN; // time steps left
  float dCTime = 0.5; // seconds, how long double checking takes
  vec dCPos; // position at start of double check
  
  // finding lost
  bool fLTurningRight; // which way are we wiggling
  double fLTurnTo; // how far out are we wiggling to
  
  // stores distance sensor readings
  double distances[SENSORS_N];
  
  vec position;
  double bearing;
  
  vec destination;
  Colour destColour; // location in database of destination block
  unsigned short destIndex; // location in database of destination block
  bool GetDestination(){ // gets the position of the destination block from the database
    return dataBase->GetBlock(destColour, destIndex, &destination);
  }
  
  const char names[2] = {'b', 'r'};
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