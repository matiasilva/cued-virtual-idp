// File:          navigation.cpp
// Date: 26/01/2021
// Description: Sets translation and rotation prodedures by robot
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

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
    
    // reading gps
    ReadGPS();
    if(position.z < 0){
      iAmRed = false;
      bearing = 0;
    } else {
      iAmRed = true;
      bearing = PI;
    }
    printf("%c: Initial = (%f, %f), %d\n", names[iAmRed], position.z, position.x, RD(bearing)); // for debugging
    
    state = initialScan; printf("%c: Starting initial scan.\n", names[iAmRed]); // for debugging
}

void Navigation::BeginStep(){
    ReadGPS();
    switch(state){
      case def: {
        if(!dataBase->GetDestination(iAmRed, position, &destColour, &destIndex)){ // no destination to be given
          ReadDistanceSensors();
          if(!dataBase->LogReading(position, &bearing, distances[0], true)){
            state = doubleChecking; printf("%c: Starting double check.\n", names[iAmRed]); // for debugging
            dCingN = 1000 * dCTime / timeStep;
            dCPos = position;
            EndStep(0, 0);
            break;
          }
          EndStep(-1, 1); // rotate slowly to scan for blocks
          break;
        }
        GetDestination();
        // can have a new destination
        printf("%c: New destination: %f, %f\n", names[iAmRed], destination.z, destination.x);
        state = movingTo;
        EndStep(0, 0);
        break;
      }
      case doubleChecking: {
        dCingN--;
        if(dCingN <= 0){
          bearing = (position - dCPos).Bearing();
          state = def; printf("%c: Back to default.\n", names[iAmRed]); // for debugging
          EndStep(0, 0);
          break;
        }
        EndStep(4, 4);
        break;
      }
      case initialScan: {
        ReadDistanceSensors();
        dataBase->LogReading(position, &bearing, distances[0], false);
        if(iAmRed){
          if(bearing > -1.22 && bearing < 0){
            state = def; printf("%c: Back to default.\n", names[iAmRed]); // for debugging
            EndStep(0, 0);
            break;
          }
          EndStep(-1, 1);
        } else {
          if(bearing < -1.92){
            state = def; printf("%c: Back to default.\n", names[iAmRed]); // for debugging
            EndStep(0, 0);
            break;
          }
          EndStep(1, -1);
        }
        break;
      }
      case movingTo: {
        GetDestination();
        if((PositionInFront() - destination).SqMag() < 0.01){
          EndStep(0, 0);
          printf("%c: Found.\n", names[iAmRed]); // for debugging
          break;
        }
        float leftSpeed, rightSpeed;
        float expected = sqrt((destination - position).SqMag());
        float off = fabs(expected - distances[0]);
        if(off > 0.05){
          state = findingLost;
          fLTurningRight = false;
          fLTurnTo = 0.08;
          EndStep(0, 0);
        } else {
          double expectedBearing = (destination - position).Bearing();
          dataBase->LogReading(position, &bearing, distances[0], false);
          bearing = expectedBearing;
          //printf("%c: dest bearing = %f\n", names[iAmRed], delta.Bearing());
          leftSpeed = 4;
          rightSpeed = 4;
          EndStep(leftSpeed, rightSpeed);
        }
        break;
      }
      case findingLost: {
        float expected = sqrt((destination - position).SqMag());
        float off = fabs(expected - distances[0]);
        if(off > 0.05){
          double expectedBearing = (destination - position).Bearing();
          if(fLTurningRight){
            if(MakePPMP(bearing - (expectedBearing - fLTurnTo)) < 0){
              fLTurningRight = false;
              fLTurnTo += 0.08;
              EndStep(0, 0);
            } else {
              EndStep(2, -2);
            }
          } else {
            if(MakePPMP(bearing - (expectedBearing + fLTurnTo)) > 0){
              fLTurningRight = true;
              EndStep(0, 0);
            } else {
              EndStep(-2, 2);
            }
          }
        } else {
          state = movingTo;
          EndStep(0, 0);
        }
        break;
      }
      default: {
        printf("Invalid state.\n");
        break;
      }
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