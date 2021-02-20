// File:          	database.h
// Date: 			26/01/2021
// Author: 			Edmund Prager, Richard Monteiro
// Modifications: 	

#ifndef DATABASE
#define DATABASE

#include "header.h"
#include "sensor.h"

// ======== comment this out for windows
#include "visualiser.h"

// colour identifies for which robot has identified a block
enum Key {kblue, kred, kboth};

// struct containing the information about a location in the database, which is a 2-d array - c moves in 1st dimension, b in the 2nd
struct Loc {
	Colour c;
	unsigned int b;
};

// struct containing all information about a block that is stored in the database
struct Block {
	vec position;
	Key seenBy; // which robot has seen the robot
    short unsigned int primaryKey; // Block primary key (defined on current robot)
    short unsigned int foreignKey; // Block foreign key (defined on other robot)
    Colour blockColour; // Known block colour
};

class DataBase {
public:
  DataBase(Robot* _robot, int time_step);
  ~DataBase(){}
  
  // tells the visualiser a time step has taken place, so it creates and renders the next the image
  void Step();
  
  // creates and initialises visualiser object, making visualiser window appear on screen
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
  bool LogReading(vec position, double bearing, float distanceL, float distanceR, bool canConfirm, Key key, vec *newDest=nullptr);
  
  // compares a colour and position block reading with the database, adding to or modifying it depending on the comparision
  void ColourAtPos(Colour colour, vec position, Key key);
  
  // a robot has retrieved a block, so it can be removed from the database - removes the block from the database if there is one at 'position'
  void Got(bool iamred, vec position);
  
  // Removes a block from the database using the position in the database, 'loc' 
  void RemoveBlock(Loc loc);
  
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
  bool GetBlock(Loc loc, vec *ret){
    if(loc.b >= colourNs[(int)loc.c]) return false;
    *ret = blocks[(int)loc.c][loc.b].position;
    return true;
  }
  
  // returns whether 'position' is outside of both the robot staring squares
  bool NotInSquare(vec position){
  	if(position.x < 0.8) return true;
  	if(position.z < -0.8 || position.z > 0.8) return false;
  	return true;
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
  bool FindByPrimaryKey(unsigned short int pkey, Loc *ret);

  // verifies database for block with matching coordinates - according to a degree of uncertainty.
  // #param: vec struct pos - position to search against
  // #returns: index of matching block - if block with matching coordinates is not found, returns -1. 
  bool FindByPosition(vec pos, Loc *ret);

  // modifies block in database with equivalent given block (blocks must have the same primary key)
  // modification does not occur when blocks have the same colour (blockColour) - database does not find such a difference major enough for a change
  // #param: Pointer to Block type block (struct), Boolean forceChange which forces the block modification, even if block colours are identical.
  void ModifyBlockByPrimaryKey(Block* block, bool forceChange = false);

  // modifies block in database with equivalent given block (according to position index of block within database)
  // method always modifies foreign key/ primary keys to match two robots. Complete modification does not occur when blocks have the same colour (blockColour) - database does not find such a difference major enough for a change
  // #param: Pointer to Block type block (struct), Boolean forceChange which forces complete block modification, even if block colours are identical.
  //         integer index relating to the position of the block to be modified within the database. This is a direct positional value obtained by transforming the 2D blocks array into 1D.
  void ModifyBlockByIndex(Block* block, Loc loc, bool forceChange = false);
  
  // Removes a block from the database according its position within the arena. If 'isDest' is true, the database will erase destination information about the robot defined by 'isred' if a block is successfully removed (because the robot called this function on it's destination)
  void RemoveByPosition(vec position, bool isDest=false, bool isred=false);
	
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
  void setMyPos(vec pos) {
      myPos = pos;
  }

  // sets other robot position value
  void setOtherPos(vec pos) {
      otherPos = pos;
  }

  // sets own position value
  vec getMyPos() {
      return myPos;
  }

  // sets other robot position value
  vec getOtherPos() {
      return otherPos;
  }

  // for debugging
  void printAll(Robot* _robot) {
      for (int i = 0; i < colourNs[question]; i++) {
          Block block = blocks[question][i];
          printf("%s: Question at %f, %f, Primary: %u, Foreign: %u\n", _robot->getName().c_str(), block.position.x, block.position.z, block.primaryKey, block.foreignKey);
      }
  }
  
  // ========== comment this out for windows
  // renders geometry to the visualiser via 'renderer'
  void Render(SDL_Renderer *renderer);
  
private:
  // stores all of the information known by the database (at the moment - obviously it will need to remember robot locations as well in future)
  //			             | this is the index
  Block blocks[COLOURS][BLOCKSEACH];
  //              | this is the colour
  unsigned short colourNs[4]; // numbers of blocks stored of each colour
  
  // stores whether each robot has a destination
  bool robotDest[2];
   // is this colour probably red?
  vec robotDestPos[2];
  
  // checks if a position is the destination of a robot
  bool NotDestOf(bool isred, vec position);
  
  // Communication sensors
  SensorEmitter* em;
  SensorReceiver* rec;

  // Location of the robot not associated with this instance of the controller
  vec otherPos;
  
  // location of the robot associated with this instance of the controller
  vec myPos;

  // colour of the robot associated with this instance of the controller
  Colour ownCol;

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

  }
  // Removes a question from the database. !!! We currently are not doing anything about robots who have a question as their destination when that question is removed.
  void RemoveQuestion(unsigned int index){
    if(index >= colourNs[question]){ printf("Tried to remove non-existent question.\n"); return; }
    for(int i=index; i<colourNs[question]-1; i++){
      blocks[question][i] = blocks[question][i+1];
    }
    colourNs[question]--;
  }
  
  // Visualiser stuff
  bool visualising;
  Visualiser *visualiser;
  
};

#endif