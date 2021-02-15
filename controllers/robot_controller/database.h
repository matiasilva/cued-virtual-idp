// File:          	database.h
// Date: 			26/01/2021
// Author: 			Edmund Prager, Richard Monteiro
// Modifications: 	

#ifndef DATABASE
#define DATABASE

#include "header.h"
#include "sensor.h"

// ======== comment this out for windows
//#include "visualiser.h"

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
  DataBase(Robot* _robot, int time_step);
  ~DataBase(){}
  
  void Step();
  void StartVisualiser();
  
  // Compares a distance sensor reading of 'distance' taken from 'position' at bearing '*bearing'
  // 	with the database, and stores the new information.
  // Returns agreement:
  //	false - data doesn't agree with any known database data. Have stored a question are measurement location
  //	true - data does agree with data base data and either:
  //		- you are looking at/near a wall so we have calibrated your bearing via 'bearing'
  //		- you are looking at/near a known block so we have modified our data according to your reading
  //		- you are looking at/near a question AND 'canConfirm' is 'true', so we have confirmed that question into a known block (colour dunno)
  //		- you are looking at/near a question AND 'canConfirm' is 'false', so we done nothing
  bool LogReading(vec position, double bearing, float distanceL, float distanceR, bool canConfirm, Key key);
  
  void ColourAtPos(Colour colour, vec position);
  
  void Got(bool iamred, vec position);  
  void RemoveBlock(Colour colour, unsigned short index);
  
  // Looks over database and stores a new destination for the robot, defined by where the position is stored in 'blocks'
  // Priority of blocks in database as new destinations is as follows:
  // 	1: if there is a block of the same colour (using 'iAmRed'), the closest to 'position' of these
  //	2: if there is a dunno, the closest of these
  //	3: if there is a question, the closest of these
  // If no new destinations exist, returns false, otherwise returns true.
  // Position of destination can be retrieved using 'GetBlock' with the parameters '*retCol' and '*retInd'
  bool GetDestination(bool iamred, vec position, vec *destination);
  
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
  // #returns: Unique identifier as short unsigned integer
  short unsigned int GenerateKey();

  // verifies whether given primary key for a block exists in the database
  // #param: (unsigned short int) pkey, representing primary key in check.
  // #returns: Boolean indicating whether block with given primary key is in list.
  bool VerifyPrimaryKey(unsigned short int pkey);

  // finds and returns index of block with given primary key in database
  // #param: (unsigned short int) pkey, representing primary key in check.
  // #returns: Index of block with given primary key in database (if it is not found, returns -1).
  int FindByPrimaryKey(unsigned short int pkey);

  // verifies database for block with matching coordinates - according to a degree of uncertainty.
  // #param: vec struct pos - position to search against
  // #returns: index of matching block - if block with matching coordinates is not found, returns -1. 
  int FindByPosition(vec pos);

  // modifies block in database with equivalent given block (blocks must have the same primary key)
  // modification does not occur when blocks have the same colour (blockColour) - database does not find such a difference major enough for a change
  // #param: Pointer to Block type block (struct), Boolean forceChange which forces the block modification, even if block colours are identical.
  void ModifyBlockByPrimaryKey(Block* block, bool forceChange = false);

  // modifies block in database with equivalent given block (according to position index of block within database)
  // method always modifies foreign key/ primary keys to match two robots. Complete modification does not occur when blocks have the same colour (blockColour) - database does not find such a difference major enough for a change
  // #param: Pointer to Block type block (struct), Boolean forceChange which forces complete block modification, even if block colours are identical.
  //         integer index relating to the position of the block to be modified within the database. This is a direct positional value obtained by transforming the 2D blocks array into 1D.
  void ModifyBlockByIndex(Block* block, int index, bool forceChange = false);

  // method directly assigns a new block to the database, assigning it to the correct category and generating its primary key
  // #param: Pointer to block struct to be added to the database.
  void AddNewBlock(Block* block);

  // Communication methods
  /**
   packData function rearranges data from Block object into array of doubles to be transmitted as packets.
   #param: Pointer to Block struct "block" - representing block in database.
           double pointer "data" - array of doubles where packet data will be stored.
*/
  void packData(Block* block, double* data, Control command);

  /**
     unpackData function rearranges data from packet form to Block struct.
     #param: Pointer to Block struct "block" - representing block in database.
             double pointer "data" - array of doubles where packet data is stored.
  */
  void unpackData(Block* block, double* data);

  /**
     sendData sends data for a specific block through emitter as a packet.
     #param: Pointer to Block struct "block" - representing block in database.
             Pointer to SensorEmitter object "em" - representing emitter sensor.
  */
  void sendData(Block* block, Control command);

  /**
      receiveData operates on receiving all data from receiver and makes use of proper database procedures
      as to synchronise data between the databases of both robots.

  */
  void receiveData();

  // sets own position value
  void setOtherPos(vec pos) {
      otherPos = pos;
  }

  void printAll(Robot* _robot) {
      for (int i = 0; i < colourNs[question]; i++) {
          Block block = blocks[question][i];
          printf("%s: Question at %f, %f, Primary: %u, Foreign: %u\n", _robot->getName().c_str(), block.position.x, block.position.z, block.primaryKey, block.foreignKey);
      }
  }
  
  // ========== comment this out for windows
  //void Render(SDL_Renderer *renderer);
  
private:
  // stores all of the information known by the database (at the moment - obviously it will need to remember robot locations as well in future)
  //			 | this is the index
  Block blocks[4][32];
  //         | this is the colour
  unsigned short colourNs[4]; // numbers of blocks stored of each colour
  
  bool robotDest[2];
  vec robotDestPos[2];
  
  // Communication sensors
  SensorEmitter* em;
  SensorReceiver* rec;

  // Location of other robot
  vec otherPos;

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
    sendData(&block, addBlock);
  }
  
  // Adds a new question into the database
  void AddQuestion(Block block){
    printf("Question added at %f, %f\n", block.position.z, block.position.x);
    blocks[question][colourNs[question]++] = block;

    // send block change through emitter
    sendData(&block, addBlock);
  }
  // Removes a question from the database. !!! We currently are not doing anything about robots who have a question as their destination when that question is removed.
  void RemoveQuestion(unsigned short index){
    for(int i=index; i<colourNs[question]-1; i++){
      blocks[question][i] = blocks[question][i+1];
    }
    colourNs[question]--;
  }
  
  // Removes block from database - index from 0 to 127 looking at database as a 1D array
  void RemoveBlock(unsigned short index) {
      unsigned short colNum = index / 32;
      for (int i = index % 32; i < colourNs[colNum] - 1; i++) {
          blocks[colNum][i] = blocks[colNum][i + 1];
      }
      colourNs[colNum]--;
  }
  
  // Visualiser stuff
  bool visualising;
  Visualiser *visualiser;
  
};

#endif