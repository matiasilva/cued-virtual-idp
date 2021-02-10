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
    
    stateManager = new StateManager(this);
    stateManager->SetNextState(new InitialScanState(this));
    
    scan = new Scan(robot, timeStep);
    
    // wheels
    for(int i=0; i<WHEELS_N; i++){
      wheels[i] = robot->getMotor(wlNames[i]);
      wheels[i]->setPosition(INFINITY);
    }
    
    // starting so we can read already
    robot->step(timeStep);
    
    // reading position to figure out which robot this instance of the controller is controlling
    position = scan->ReadPosition();
    if(position.z < 0){
      iAmRed = false;
      bearing = 0;
      key = kblue;
    } else {
      iAmRed = true;
      bearing = PI;
      key = kred;
    }
    printf("%c: Initial = (%f, %f), %d\n", names[iAmRed], position.z, position.x, RD(bearing));
}

bool Navigation::DBGetDestination(){
	return dataBase->GetDestination(iAmRed, position, &destColour, &destIndex);
}
bool Navigation::DBLogReading(bool canConfirm){
	return dataBase->LogReading(position, bearing, distances[0], canConfirm, key);
}

void Navigation::Run(){
	TakeReadings();
	
    RetrieveDBDestination();
    
    stateManager->Run();

    // send robot location
    Block robotBlock{ position, key, 0, 0, robotPos };
    dataBase->sendData(&robotBlock);

    // Check receiver - only if not an InputState
    if (!dynamic_cast<InputState*>(stateManager->GetState())) {
        dataBase->receiveData();
    }
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