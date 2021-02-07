// File:          communication.cpp
// Date: 26/01/2021
// Description: Handling of communication between two robots
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>

#include "header.h"
#include "database.h"
#include "sensor.h"

// All the webots classes are defined in the "webots" namespace
using namespace webots;


void packData(Block* block, double* data) {

	// Based on packet length
	*data = block->position.x;
	*(data + 1) = block->position.z;
	*(data + 2) = (double)block->blockColour;
	*(data + 3) = block->primaryKey;
	*(data + 4) = block->foreignKey;
}

void unpackData(Block* block, double* data) {

	// Based on packet length
	block->position.x = *data;
	block->position.z = *(data + 1);
	block->blockColour = (colour) *(data + 2);
	block->primaryKey = (unsigned short int) *(data+3);
	block->foreignKey = (unsigned short int) * (data + 4);
}

void sendData(Block* block, SensorEmitter* em) {

	// Create array of doubles for transmission from given data
	double data[5];

	packData(block, data);
	em->send(data);
}

void receiveData();
