// File:          communication.h
// Date: 26/01/2021
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#ifndef COMMUNICATION
#define COMMUNICATION

#include "database.h"

/**
   packData function rearranges data from Block object into array of doubles to be transmitted as packets.
   #param: Pointer to Block struct "block" - representing block in database.
           double pointer "data" - array of doubles where packet data will be stored.
*/
void packData(Block *block, double* data);

/**
   unpackData function rearranges data from packet form to Block struct.
   #param: Pointer to Block struct "block" - representing block in database.
           double pointer "data" - array of doubles where packet data is stored.
*/
void unpackData(Block *block, double* data);

/**
   sendData sends data for a specific block through emitter as a packet.
   #param: Pointer to Block struct "block" - representing block in database.
           Pointer to SensorEmitter object "em" - representing emitter sensor.
*/
void sendData(Block* block, SensorEmitter* em);

/**
    receiveData operates on receiving all data from receiver and makes use of proper database procedures
    as to synchronise data between the databases of both robots.
    
*/
void receiveData(DataBase* database, SensorReceiver* rec);

#endif