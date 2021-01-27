// File:          navigation.cpp
// Date: 26/01/2021
// Description: Sets translation and rotation prodedures by robot
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#include "navigation.h"

Navigation::Navigation(Robot *_robot, DataBase *_dataBase, int _timeStep) {
    robot = _robot;
    dataBase = _dataBase;
    timeStep = _timeStep;
    
    // wheels
    for(int i=0; i<WHEELS_N; i++){
      wheels[i] = robot->getMotor(wlNames[i]);
      wheels[i]->setPosition(INFINITY);
    }
    
    // sensors
    for(int i=0; i<SENSORS_N; i++){
      distanceSensors[i] = robot->getDistanceSensor(dsNames[i]);
      distanceSensors[i]->enable(timeStep);
    }
    
    // gps
    gps = robot->getGPS("gps");
    gps->enable(timeStep);
    
    // starting so we can read already
    robot->step(timeStep);
    
    // reading gps to figure out which robot this instance of the controller is controlling
    ReadGPS();
    if(position.z < 0){
      iAmRed = false;
      bearing = 0;
    } else {
      iAmRed = true;
      bearing = PI;
    }
    printf("%c: Initial = (%f, %f), %d\n", names[iAmRed], position.z, position.x, RD(bearing));
    
    stateManager = new StateManager(this);
    stateManager->SetNextState(new InitialScanState(this));
}

bool Navigation::DBGetDestination(){
	return dataBase->GetDestination(iAmRed, position, &destColour, &destIndex);
}
bool Navigation::DBLogReading(bool canConfirm){
	return dataBase->LogReading(position, &bearing, distances[0], canConfirm);
}

void Navigation::Run(){
    ReadGPS();
    ReadDistanceSensors();
    RetrieveDBDestination();
    
    stateManager->Run();
}

void Navigation::EndStep(float leftSpeed, float rightSpeed){
    //printf("%c: state is %d; vs = %f, %f\n", names[iAmRed], state, leftSpeed, rightSpeed);
    
    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
    
    // integrates the wheel velocities to precict the change in orientation
    bearing -= TURN_FACTOR*leftSpeed*timeStep/2000;
    bearing += TURN_FACTOR*rightSpeed*timeStep/2000;
    bearing = MakePPMP(bearing);
}