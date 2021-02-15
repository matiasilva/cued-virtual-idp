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
    for(int i=0; i<MOTORS_N; i++){
      motors[i] = robot->getMotor(motorNames[i]);
    }
    motors[leftWheel]->setPosition(INFINITY);
    motors[rightWheel]->setPosition(INFINITY);
    motors[arm]->setPosition(0);
    motors[leftClaw]->setPosition(0);
    motors[rightClaw]->setPosition(0);
    
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
    initialPosition = position;
    printf("%c: Initial = (%f, %f), %d\n", names[iAmRed], initialPosition.z, initialPosition.x, RD(bearing));
    
    if(!iAmRed && visualiserActive) dataBase->StartVisualiser();
}

void Navigation::Got(){
	dataBase->Got(iAmRed, position);
}

bool Navigation::DBGetDestination(){
	return dataBase->GetDestination(iAmRed, position, &destination);
}
bool Navigation::DBLogReading(bool canConfirm){
	return dataBase->LogReading(position, bearing, distances[0], distances[1], canConfirm, key);
}

Colour Navigation::ReadCamera(){
	Colour col = scan->ReadColour();
	dataBase->ColourAtPos(col, destination);
	return col;
}

void Navigation::Run(){
	TakeReadings();
    
    stateManager->Run();

    // send robot location
    Block robotBlock{ position, key, 0, 0, question };
    dataBase->sendData(&robotBlock, robotPos);

    // Check receiver - only if not an InputState
    if (!dynamic_cast<InputState*>(stateManager->GetState())) {
        dataBase->receiveData();
    }
	
	dataBase->Step();
	
}

void Navigation::SetArmAngle(double angle){
	motors[arm]->setPosition(angle);
}
void Navigation::SetClawWidth(float width){
	float dist = (0.14f - width)/2.0f;
	motors[leftClaw]->setPosition(dist);
	motors[rightClaw]->setPosition(-dist);
}

void Navigation::EndStep(float leftSpeed, float rightSpeed){
    //printf("%c: state is %d; vs = %f, %f\n", names[iAmRed], state, leftSpeed, rightSpeed);
    
    motors[leftWheel]->setVelocity(leftSpeed);
    motors[rightWheel]->setVelocity(rightSpeed);
    
    // integrates the wheel velocities to precict the change in orientation
    bearing -= TURN_FACTOR*leftSpeed*timeStep/2000;
    bearing += TURN_FACTOR*rightSpeed*timeStep/2000;
    bearing = MakePPMP(bearing);
}