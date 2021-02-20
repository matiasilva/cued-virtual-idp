// File:          communication.cpp
// Date: 26/01/2021
// Description: Handling of communication between two robots
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include "communication.h"

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
	block->blockColour = (Colour) *(data + 2);
	block->primaryKey = (unsigned short int) *(data+4); // note primary and foreign key swapped here - as reference robot is swapped
	block->foreignKey = (unsigned short int) * (data + 3);
}

void sendData(Block* block, SensorEmitter* em) {

	// Create array of doubles for transmission from given data
	double data[5];

	packData(block, data);
	em->send(data);
}

void receiveData(DataBase* database, SensorReceiver* rec) {
	while (!rec->getQueueEmpty()) {
		printf("Data received.\n");
		// receive data from queue
		Block block;
		unpackData(&block, rec->getData());

		bool keyExists = false;
		// verify primary key - if it is non-zero
		if (block.primaryKey != 0) keyExists = database->VerifyPrimaryKey(block.primaryKey);

		if (keyExists) database->ModifyBlockByPrimaryKey(&block);
		else {
			Loc loc;
			if(database->FindByPosition(block.position, &loc)) database->ModifyBlockByIndex(&block, loc);
			else database->AddNewBlock(&block);
		}
	}
}
