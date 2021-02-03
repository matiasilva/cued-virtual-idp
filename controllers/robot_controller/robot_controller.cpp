// File:          first_basic.cpp
// Date:          25/01/2021
// Description:   
// Author:        Edmund Prager
// Modifications: 

#include "header.h"
#include "database.h"
#include "navigation.h"

// The arguments of the main function can be specified by the "controllerArgs" field of the Robot node
int main(int argc, char **argv){

  Robot *robot = new Robot();
  
  int timeStep = (int)robot->getBasicTimeStep();
    
  
  // ----- variables -----
  // stores and deals with the controllers knowledge of the arena
  DataBase *dataBase = new DataBase();
  
  // resposible for robot input and output (at the moment - input to be done by sensor class in future)
  Navigation *navigation = new Navigation(robot, dataBase, timeStep);
  
  // Main loop; perform simulation steps until Webots is stopping the controller
  while(robot->step(timeStep) != -1){
    
    // runs the navigation code
    navigation->Run();

 }

  delete robot;
  return 0;
}