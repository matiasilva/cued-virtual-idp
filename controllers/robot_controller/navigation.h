// File:          navigation.h
// Date: 26/01/2021
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#ifndef NAVIGATION
#define NAVIGATION

#include "header.h"
#include "database.h"
#include "state.h"
#include "scan.h"

enum EMotor {leftWheel, rightWheel, arm, leftClaw, rightClaw};

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
  vec GetDestination(){ return destination; }
  vec GetOtherPos() { return dataBase->getOtherPos(); }
  float GetDistance(int which){ return distances[which]; }
  Colour ReadCamera();
  void SetBearing(double newBearing){ bearing = newBearing; }
  double GetBearing(){ return bearing; }
  vec PositionInFront(){ // position where the block should be in front
    vec delta = {(float)(0.1*cos(bearing)), (float)(0.1*sin(bearing))};
    return position + delta;
  }
  vec GetInitialPosition(){ return initialPosition; }
  StateManager *GetStateManager(){ return stateManager; }
  
  void Got();
  bool DBGetDestination();
  bool DBLogReading(bool canConfirm, bool feedBackDest=false);
  void DestinationInvalid();
  
  // main functions
  // runs the code to be called every step
  void Run();
  
  // sets the motor speeds and integrates them to modify the bearing estimate
  // generally called by state objects to wrap up at the end of each step
  void EndStep(float leftSpeed, float rightSpeed);
  void SetArmAngle(double angle);
  void SetClawWidth(float width);
  
private:
	// =========== set this to false for windows
  const bool visualiserActive = false;

  Robot *robot;
  DataBase *dataBase;
  int timeStep;
  
  Key key;
  
  // other classes we use
  StateManager *stateManager;
  Scan *scan;
  
  // identification stuff
  bool iAmRed;
  const char names[2] = {'b', 'r'};
  
  // motor stuff
  const char motorNames[MOTORS_N][10] = {"wheel1", "wheel2", "arm", "slider1", "slider2"};
  Motor *motors[MOTORS_N];
  
  // variables we read in
  vec initialPosition;
  vec position;
  double bearing;
  double distances[SENSORS_N]; // stores distance sensor readings
  
  // destination stuff
  vec destination;
  
  // gets the gps position and stores it as a vector in the 2D plane
  void TakeReadings(){
    position = scan->ReadPosition();
    bearing = scan->ReadBearing();
    distances[0] = scan->ReadLeftDistance();
    distances[1] = scan->ReadRightDistance();
  }
  
};

#endif