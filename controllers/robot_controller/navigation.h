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

// where each motor in the proto is stored in the array of motors
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
  
  // tells the database that we have captured the block at our destination
  void Got();
  
  // asks the database for a destination
  bool DBGetDestination();
  
  // sends sensor information to the database. 'canConfirm' defines whether the reading can confirm a question, 'feedBackDest' defines whether the reading is supposed to be of our destination, and so if the database should update the position of our destination according to the reading
  bool DBLogReading(bool canConfirm, bool feedBackDest=false);
  
  // tells the database to remove block information from the location of our destination, as we can't see a block at our destination
  void DestinationInvalid();
  
  // runs the code to be called every step: reads sensors, executes state behaviour, sends the database our position then receives information from the other controller
  void Run();
  
  // sets the motor speeds and integrates them to modify the bearing estimate
  // generally called by state objects to wrap up at the end of each step
  void EndStep(float leftSpeed, float rightSpeed);
  
  // controls the arm motor
  void SetArmAngle(double angle);
  
  // controls the grabber motors
  void SetClawWidth(float width);
  
private:
	// =========== set this to false for windows
  const bool visualiserActive = true;

  Robot *robot;
  DataBase *dataBase;
  int timeStep;
  
  // colour of the robot associated with this instance of the controller
  Key key;
  
  // other classes we use
  StateManager *stateManager;
  Scan *scan;
  
  // identification stuff
  // colour of the robot associated with this instance of the controller in boolean form
  bool iAmRed;
  // for debugging
  const char names[2] = {'b', 'r'};
  
  // motors
  const char motorNames[MOTORS_N][10] = {"wheel1", "wheel2", "arm", "slider1", "slider2"};
  Motor *motors[MOTORS_N];
  
  // variables we read in
  // starting position (where the blocks we collect are deposited)
  vec initialPosition;
  vec position;
  double bearing;
  double distances[SENSORS_N]; // stores distance sensor readings
  
  // current destination (if we have one)
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