// File:          	database.h
// Date: 			26/01/2021
// Author: 			Edmund Prager, Richard Monteiro
// Modifications: 	

#ifndef DATABASE
#define DATABASE

#include "header.h"

enum Key {kblue, kred, kboth};

struct Block {
	vec position;
	Key seenBy;
    short unsigned int primaryKey; // Block primary key (defined on current robot)
    short unsigned int foreignKey; // Block foreign key (defined on other robot)
    Colour blockColour; // Known block colour
};

class DataBase {
public:
  DataBase(){
  	// initialises all numbers of blocks to zero
    colourNs[dunno] = colourNs[blue] = colourNs[red] = colourNs[question] = 0;
  }
  ~DataBase(){}
  
  // Compares a distance sensor reading of 'distance' taken from 'position' at bearing '*bearing'
  // 	with the database, and stores the new information.
  // Returns agreement:
  //	false - data doesn't agree with any known database data. Have stored a question are measurement location
  //	true - data does agree with data base data and either:
  //		- you are looking at/near a wall so we have calibrated your bearing via 'bearing'
  //		- you are looking at/near a known block so we have modified our data according to your reading
  //		- you are looking at/near a question AND 'canConfirm' is 'true', so we have confirmed that question into a known block (colour dunno)
  //		- you are looking at/near a question AND 'canConfirm' is 'false', so we done nothing
  bool LogReading(vec position, double bearing, float distance, bool canConfirm, Key key);
  
  // Looks over database and stores a new destination for the robot, defined by where the position is stored in 'blocks'
  // Priority of blocks in database as new destinations is as follows:
  // 	1: if there is a block of the same colour (using 'iAmRed'), the closest to 'position' of these
  //	2: if there is a dunno, the closest of these
  //	3: if there is a question, the closest of these
  // If no new destinations exist, returns false, otherwise returns true.
  // Position of destination can be retrieved using 'GetBlock' with the parameters '*retCol' and '*retInd'
  bool GetDestination(bool iamred, vec position, Colour *retCol, unsigned short *retInd);
  
  // Stores the position of the database block of colour 'colour' stored at index 'index' in '*ret'
  // Returns success
  bool GetBlock(Colour colour, unsigned short index, vec *ret){
    if(index >= colourNs[colour]) return false;
    *ret = blocks[colour][index].position;
    return true;
  }
  
  // returns whether the given vector position 'vec' lies within the arena
  bool WithinArena(vec pos, bool tol){
    return (pos.x >= -SIDE_HLENGTH - tol*TOLERANCE &&
            pos.x <=  SIDE_HLENGTH + tol*TOLERANCE &&
            pos.z >= -SIDE_HLENGTH - tol*TOLERANCE &&
            pos.z <=  SIDE_HLENGTH + tol*TOLERANCE);
  }
  
  // for debugging
  void Debug(){
    printf("Dunnos: %d\n", colourNs[dunno]);
    printf("Blues: %d\n", colourNs[blue]);
    printf("Reds: %d\n", colourNs[red]);
    printf("Questions: %d\n", colourNs[question]);
  }

  // handles creation of primary keys for blocks - a value of 0 for keys mean a key has not yet been assigned.
  short unsigned int GenerateKey();
  
private:
  // stores all of the information known by the database (at the moment - obviously it will need to remember robot locations as well in future)
  //			 | this is the index
  Block blocks[4][32];
  //         | this is the colour
  unsigned short colourNs[4]; // numbers of blocks stored of each colour
  
  // Adds a block of unknown colour to the database
  void AddDunnoBlock(Block block){
    Colour colour = dunno;
    if(colourNs[red] >= 4){ // already know of maximum number of red blocks
      if(colourNs[blue] >= 4){ // already know of maximum number of blue blocks
        AddQuestion(block); // already know of maximum number of total blocks. put a question here anyway incase we change our minds about one of the others (cannot yet happen)
        return;
      }
      colour = blue; // it must be blue
    } else if(colourNs[blue] >= 4){
      colour = red; // it must be red
    }
    blocks[colour][colourNs[colour]++] = block; // store the new block
  }
  
  // Adds a new question into the database
  void AddQuestion(Block block){
    printf("Question added at %f, %f, %u\n", block.position.z, block.position.x, block.primaryKey);
    blocks[question][colourNs[question]++] = block;
  }
  // Removes a question from the database. !!! We currently are not doing anything about robots who have a question as their destination when that question is removed.
  void RemoveQuestion(unsigned short index){
    for(int i=index; i<colourNs[question]-1; i++){
      blocks[question][i] = blocks[question][i+1];
    }
    colourNs[question]--;
  }
};

#endif