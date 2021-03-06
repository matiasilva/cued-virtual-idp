// File:          database.cpp
// Date: 26/01/2021
// Description: Contains map database of rectangle coordinates and known locations of blocks
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#include "database.h"

DataBase::DataBase(Robot* _robot, int time_step){
  	// initialises all numbers of blocks to zero
    colourNs[dunno] = colourNs[blue] = colourNs[red] = colourNs[question] = 0;

    // initialises all sensors
    em = new SensorEmitter(_robot, "emitter");
    rec = new SensorReceiver(_robot, "receiver", time_step);

	if (_robot->getName() == "skids") ownCol = blue;
	else ownCol = red;
    
   	visualising = false;
   	
   	robotDest[0] = robotDest[1] = false;
}

void DataBase::StartVisualiser(){
	
	// ========= comment this out for windows
	
	visualiser = new Visualiser(660, 660, this);
	if(!visualiser->Init()){
		printf("Visualiser error.\n");
		return;
	}
	visualising = true;
	
}
void DataBase::Step(){
	
	// =========== comment this our for windows
	if(visualising) visualiser->Loop();
}

void DataBase::Got(bool iamred, vec position){
	printf("Got at %f, %f\n", position.z, position.x);
	Colour col = (Colour)(1 + iamred);
	
	double near[12];
	unsigned short bs[12];
	unsigned short nearN = 0;
	float sqDist;
	for(int b=0; b<colourNs[col]; b++){
		sqDist = (position - blocks[col][b].position).SqMag();
		if(sqDist < 0.04){ // reading is near a known block
			bs[nearN] = b;
			near[nearN++] = sqDist;
		}
	}
	if(nearN){ // is near at least one known block
		unsigned short index = FindMin(nearN, near);
		Debug();
		RemoveBlock({col, index});
		printf("Block of colour %d, index %d removed.\n", (int)col, index);
	}
}

// Removes block from database
void DataBase::RemoveBlock(Loc loc){
	if(loc.b >= colourNs[(int)loc.c]){ printf("Tried to remove non-existent block.\n"); return; }
	for(int i=loc.b; i<colourNs[(int)loc.c]-1; i++) {
		blocks[(int)loc.c][i] = blocks[(int)loc.c][i+1];
	}
	colourNs[(int)loc.c]--;
}

void DataBase::ColourAtPos(Colour colour, vec position, Key key){
	double near[BLOCKSEACH];
	Colour cs[BLOCKSEACH]; unsigned short bs[BLOCKSEACH];
	unsigned short nearN = 0;
	float sqDist;
	for(int c=0; c<4; c++){ // only checks dunnos, reds, blues and questions
		for(int b=0; b<colourNs[c]; b++){
			sqDist = (position - blocks[c][b].position).SqMag();
			if(sqDist < BLOCK_WMAX*BLOCK_WMAX){ // reading is near a known block
				cs[nearN] = (Colour)c; bs[nearN] = b;
				near[nearN++] = sqDist;
			}
		}
	}
	if(nearN){ // is near at least one known block
		unsigned short index = FindMin(nearN, near); // find known block closest to
		blocks[colour][colourNs[colour]] = blocks[cs[index]][bs[index]];
		blocks[colour][colourNs[colour]].blockColour = colour;
		if (colour == ownCol)sendData(&blocks[colour][colourNs[colour]], addBlock); // send block info through emitter if colour not own robot's - avoids integrity errors
		colourNs[colour]++;
		RemoveBlock({cs[index], bs[index]});
	} else { // is not near at least one known block
		unsigned short int primaryKey = GenerateKey();
		blocks[colour][colourNs[colour]] = {position, key, primaryKey, 0, colour};
		if (colour == ownCol)sendData(&blocks[colour][colourNs[colour]], addBlock); // send block info through emitter if colour not own robot's - avoids integrity errors
		colourNs[colour]++;
	}
}

bool DataBase::LogReading(vec position, double bearing, float distanceL, float distanceR, bool canConfirm, Key key, vec *newDest){
	
	float difference = fabs(distanceL - distanceR);
	if(difference > 0.05) return true;
	
	float distance = (distanceL + distanceR)/2.0f;
	vec deltaL = {(float)(distanceL * cos(bearing)), (float)(distanceL * sin(bearing))};
	vec deltaR = {(float)(distanceR * cos(bearing)), (float)(distanceR * sin(bearing))};
	vec location = position + 0.5f*(deltaL + deltaR);
	// we're looking at a wall
	float tol = TOLERANCE * distance;
	if(location.z > SIDE_HLENGTH - tol) return true;
	if(location.z < -SIDE_HLENGTH + tol) return true;
	if(location.x > SIDE_HLENGTH - tol) return true;
	if(location.x < -SIDE_HLENGTH + tol) return true;
    
	// a block is being seen?
	double near[BLOCKSEACH];
	Colour cs[BLOCKSEACH]; unsigned short bs[BLOCKSEACH];
	unsigned short nearN = 0;
	float sqDist;
	for(int c=0; c<3; c++){ // only checks dunnos, reds and blues (not questions)
		for(int b=0; b<colourNs[c]; b++){
			sqDist = (location - blocks[c][b].position).SqMag();
			if(sqDist < 4*BLOCK_WMAX*BLOCK_WMAX){ // reading is near a known block
				cs[nearN] = (Colour)c; bs[nearN] = b;
				near[nearN++] = sqDist;
			}
		}
	}
	if(nearN){ // is near at least one known block
		unsigned short index = FindMin(nearN, near); // find known block closest to
		vec newPos = 0.5 * (blocks[cs[index]][bs[index]].position + location);
		blocks[cs[index]][bs[index]].position = newPos; // move position of block to average of previous thought position and new reading
		blocks[cs[index]][bs[index]].seenBy = key;
		//printf("Block position updated at %f, %f\n", blocks[cs[index]][bs[index]].position.z, blocks[cs[index]][bs[index]].position.x);
		if(newDest){
			*newDest = newPos;
			robotDestPos[key] = newPos;
      		robotDest[key] = true;
		}
		return true; // tell robot that we agree with its reading
	} else { // is not near a known block
		// a pre-existing question is being seen?
		double near[colourNs[question]]; unsigned short qs[colourNs[question]];
		unsigned short nearN = 0;
		float sqDist;
		for(int q=0; q<colourNs[question]; q++){
			sqDist = (location - blocks[question][q].position).SqMag();
			if(sqDist < 4*BLOCK_WMAX*BLOCK_WMAX){ // reading is near a pre-existing question
				if(!canConfirm) return true; // can't confirm its a block, but have logged this question already
					qs[nearN] = q;
					near[nearN++] = sqDist;
				}
			}
		if(nearN){ // not near a wall or a known block, but near a pre-existing question
			if(!canConfirm) return true; // can't confirm its a block, but have logged this question already
			unsigned short index = FindMin(nearN, near); // find closest pre-existing question
			AddDunnoBlock(blocks[question][qs[index]]); // turn question into dunno
			vec newPos = blocks[question][qs[index]].position;
			printf("Question confirmed at %f, %f : from %f, %f on bear %d\n", newPos.z, newPos.x, position.z, position.x, RD(bearing));
			RemoveQuestion(qs[index]);             //    a known block
			if(newDest){
				*newDest = newPos;
				robotDestPos[key] = newPos;
				robotDest[key] = true;
			}
			return true; // tell robot that we agree with its reading
		} else { // not near a wall, known block or pre-existing question
			unsigned short int primaryKey = GenerateKey(); // generate primary key for question
			AddQuestion({location, key, primaryKey, 0, question}); // add in a question
			return false; // tell robot to double check
		}
	}
}

bool DataBase::NotDestOf(bool isred, vec position){
	if(!robotDest[isred]) return true;
	return ((robotDestPos[isred] - position).SqMag() > BLOCK_WMAX*BLOCK_WMAX);
}

bool DataBase::GetDestination(bool iamred, vec position, vec *destination){
    Debug();
    Colour colour = (iamred ? red : blue);
	double sqdist;
	int n;
    if(colourNs[colour]){
      n = 0;
      double sqdists[colourNs[colour]];
      unsigned int bs[colourNs[colour]];
      for(int i=0; i<colourNs[colour]; i++){
      	vec blockPos = blocks[colour][i].position;
        if(NotInSquare(blockPos) && NotDestOf(!iamred, blockPos)){
        	sqdists[n] = (blockPos - position).SqMag();
        	bs[n] = i;
        	n++;
        }
      }
      unsigned short index = FindMin(n, sqdists);
      *destination = blocks[colour][bs[index]].position;
      robotDestPos[iamred] = *destination;
      robotDest[iamred] = true;
      return true;
    } else if(colourNs[dunno]){
      n = 0;
      double sqdists[colourNs[dunno]];
      unsigned int bs[colourNs[dunno]];
      for(int i=0; i<colourNs[dunno]; i++){
        vec blockPos = blocks[dunno][i].position;
        if(NotInSquare(blockPos) && NotDestOf(!iamred, blockPos)){
        	bs[n] = i;
        	n++;
        }
      }
      unsigned short index = FindMin(n, sqdists);
      *destination = blocks[dunno][bs[index]].position;
      robotDestPos[iamred] = *destination;
      robotDest[iamred] = true;
      return true;
    } else if(colourNs[question]){
      n = 0;
      double sqdists[colourNs[question]];
      unsigned int bs[colourNs[question]];
      unsigned int toRemove[colourNs[question]];
      unsigned int toRemoveN = 0;
      for(int i=0; i<colourNs[question]; i++){
		  vec blockPos = blocks[question][i].position;
          if(NotInSquare(blockPos) && NotDestOf(!iamred, blockPos)){
			  sqdist = (blockPos - position).SqMag();
			  if(sqdist < 0.1*BLOCK_POS_UNCERTAINTY) {
				  toRemove[toRemoveN++] = i;
			  } else if(blockPos.x < otherPos.x + BLOCK_POS_UNCERTAINTY && blockPos.x > otherPos.x - BLOCK_POS_UNCERTAINTY
						&& blockPos.z < otherPos.z + BLOCK_POS_UNCERTAINTY && blockPos.z > otherPos.z - BLOCK_POS_UNCERTAINTY) {
				  // ensures robot does not pick other robot as the destination
				  toRemove[toRemoveN++] = i;
			  } else {
				  sqdists[n] = sqdist;
				  bs[n] = i;
				  n++;
			  }
		  }
      }
      
      unsigned short index = FindMin(n, sqdists);
      *destination = blocks[question][bs[index]].position;
      robotDestPos[iamred] = *destination;
      robotDest[iamred] = true;
      
      // removing invalid quesions:
      for(int i=toRemoveN-1; i>=0; i--){
      	RemoveQuestion(i);
      }
      
      return true;
    } else {
      robotDest[iamred] = false;
      return false;
    }
}

short unsigned int DataBase::GenerateKey(){
	static unsigned int previous_key = 0; // create static var to store previous key value - such that no primary key is ever repeated
	previous_key++; // No overflow errors occur until previous_key = 65535 - well beyond the possible keys used in this program.
	return previous_key;
}

bool DataBase::VerifyPrimaryKey(unsigned short int pkey){
	for(unsigned int c=0; c<4; c++) {
		for(unsigned int b=0; b<colourNs[c]; b++){
			if (blocks[c][b].primaryKey == pkey) return true;
		}
	}
	return false;
}

bool DataBase::FindByPrimaryKey(unsigned short int pkey, Loc *ret){
	for(unsigned int c=0; c<4; c++){
		for(unsigned int b=0; b<colourNs[c]; b++){
			if(blocks[c][b].primaryKey == pkey){
				*ret = {(Colour)c, b};
				return true;
			}
		}
	}
	return false;
}

bool DataBase::FindByPosition(vec pos, Loc *ret){
	vec pos_db;
	for(unsigned int c=0; c<4; c++){
		for(unsigned int b=0; b<colourNs[c]; b++){
			pos_db = blocks[c][b].position;
			// If both x' and y' (new block) correspond to existing x, y with uncertainty BLOCK_POS_UNCERTAINTY u ( (x - u) < x' < (x + u), (y - u) < y' < (y + u)), blocks are deemed equal.
			if(	pos.x < pos_db.x + BLOCK_POS_UNCERTAINTY && pos.x > pos_db.x - BLOCK_POS_UNCERTAINTY
				&& pos.z < pos_db.z + BLOCK_POS_UNCERTAINTY && pos.z > pos_db.z - BLOCK_POS_UNCERTAINTY){
				*ret = {(Colour)c, b};
				return true;
			}
		}
	}
	return false;
}

void DataBase::ModifyBlockByPrimaryKey(Block* block, bool forceChange){
	// find block in database
    Loc found_location;;
	if(!FindByPrimaryKey(block->primaryKey, &found_location)) return; // block with primary key not in database

	// replace block in database, only if forceChange is set to true, or if colour parameters do not match - representing a relevant change in value for the block.
	if(forceChange || (found_location.c != block->blockColour && block->blockColour != dunno)) {
		blocks[found_location.c][found_location.b] = *block;
		blocks[block->blockColour][colourNs[block->blockColour]] = *block; // Add block to new subarray
		colourNs[block->blockColour]++;
		RemoveBlock(found_location); // Remove block from previous colour subarray
	}
	
}

void DataBase::ModifyBlockByIndex(Block* block, Loc loc, bool forceChange){
	Block block_db = blocks[(int)loc.c][loc.b];
	
	// set foreign key and primary keys to match
	blocks[(int)loc.c][loc.b].foreignKey = block->foreignKey;
	block->primaryKey = block_db.primaryKey;
	
	// replace block in database, only if forceChange is set to true, or if colour parameters do not match - representing a relevant change in value for the block to a colour that is either red or blue.
	if(forceChange || (loc.c != block->blockColour && block->blockColour != dunno)) {
		blocks[(int)loc.c][loc.b] = *block;
		blocks[block->blockColour][colourNs[block->blockColour]] = *block; // Add block to new subarray
		colourNs[block->blockColour]++;
		RemoveBlock(loc); // Remove block from previous colour subarray
	}
}

void DataBase::RemoveByPosition(vec position, bool isDest, bool isred){
	Loc loc;
	if(!FindByPosition(position, &loc)){
		printf("Tried to remove non-existent block.\n");
		return;
	}
	printf("Removing block by position: colour %d, index %d.\n", (int)loc.c, loc.b);
	printf("Blocks of colour %d: %d\n", (int)loc.c, colourNs[(int)loc.c]);
	RemoveBlock(loc); // Remove block from array
	if(isDest) robotDest[isred] = false;
}

void DataBase::AddNewBlock(Block* block){
	Colour col = block->blockColour;
	unsigned short int primaryKey = GenerateKey(); // generate primary key for block
	block->primaryKey = primaryKey;

	if((col == red || col == blue) && colourNs[col] >= 4){
		block->blockColour = question;
		blocks[question][colourNs[question]] = *block;
		colourNs[question]++;
	} else {
		blocks[col][colourNs[col]] = *block;
		colourNs[col]++;
	}
}

// Communication methods

void DataBase::packData(Block* block, double* data, Control command) {

	// Based on packet length
	*data = block->position.x;
	*(data + 1) = block->position.z;
	*(data + 2) = (double)block->blockColour;
	*(data + 3) = block->primaryKey;
	*(data + 4) = block->foreignKey;
	*(data + 5) = (double)command;
}

void DataBase::unpackData(Block* block, double* data) {

	// Based on packet length
	block->position.x = *data;
	block->position.z = *(data + 1);
	block->blockColour = (Colour) * (data + 2);
	block->primaryKey = (unsigned short int) * (data + 4); // note primary and foreign key swapped here - as reference robot is swapped
	block->foreignKey = (unsigned short int) * (data + 3);

}

void DataBase::sendData(Block* block, Control command) {

	// Create array of doubles for transmission from given data
	double data[6];

	packData(block, data, command);
	em->send(data);

}

void DataBase::receiveData() {
	double* p = rec->getData();
	while (!rec->getQueueEmpty() || p != NULL) {
		bool keyExists = false;
		Loc loc;
		// receive data from queue
		Block block;
		unpackData(&block, p);
		Control command = (Control) * (p + 5);

		switch (command) {

			// Add or Modify block
		case addBlock:
			// block received is a block identified by the other robot

			// verify primary key - if it is non-zero
			if (block.primaryKey != 0) keyExists = VerifyPrimaryKey(block.primaryKey);

			if (keyExists) ModifyBlockByPrimaryKey(&block);
			else {
				if(FindByPosition(block.position, &loc)) ModifyBlockByIndex(&block, loc);
				else AddNewBlock(&block);
			}
			break;
			// Remove block from database
		case removeBlock:

			// verify index
			if(FindByPosition(block.position, &loc)) RemoveBlock(loc);
			break;
			// Record other robot position
		case robotPos:
			otherPos = block.position;
			break;
		}
	    p = rec->getData();
	}
}

// ========= comment this whole function out for windows

void DataBase::Render(SDL_Renderer *renderer){
	SDL_Rect rect;
	
	// blocks	
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	vec pos;
	for(int q=0; q<colourNs[question]; q++){
		pos = blocks[question][q].position;
		rect = {(int)(pos.z*300.0f) - 8 + 330, -(int)(pos.x*300.0f) - 8 + 330, 16, 16};
		SDL_RenderDrawRect(renderer, &rect);
	}
	
	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
	for(int q=0; q<colourNs[red]; q++){
		pos = blocks[red][q].position;
		rect = {(int)(pos.z*300.0f) - 8 + 330, -(int)(pos.x*300.0f) - 8 + 330, 16, 16};
		SDL_RenderDrawRect(renderer, &rect);
	}
	
	SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
	for(int q=0; q<colourNs[blue]; q++){
		pos = blocks[blue][q].position;
		rect = {(int)(pos.z*300.0f) - 8 + 330, -(int)(pos.x*300.0f) - 8 + 330, 16, 16};
		SDL_RenderDrawRect(renderer, &rect);
	}
	
	SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
	for(int q=0; q<colourNs[dunno]; q++){
		pos = blocks[dunno][q].position;
		rect = {(int)(pos.z*300.0f) - 8 + 330, -(int)(pos.x*300.0f) - 8 + 330, 16, 16};
		SDL_RenderDrawRect(renderer, &rect);
	}
	
	// destinations
	if(robotDest[0]){
		SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
		pos = robotDestPos[0];
		int x, y;
		x = (int)(pos.z*300.0f) + 330;
		y = -(int)(pos.x*300.0f) + 330;
		SDL_RenderDrawLine(renderer, x - 10, y - 10, x + 10, y + 10);
		SDL_RenderDrawLine(renderer, x + 10, y - 10, x - 10, y + 10);
	}
	if(robotDest[1]){
		SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
		pos = robotDestPos[1];
		int x, y;
		x = (int)(pos.z*300.0f) + 330;
		y = -(int)(pos.x*300.0f) + 330;
		SDL_RenderDrawLine(renderer, x - 10, y - 10, x + 10, y + 10);
		SDL_RenderDrawLine(renderer, x + 10, y - 10, x - 10, y + 10);
	}
	
	// robots
	RGB myColour = (ownCol == blue ? (RGB){0, 0, 255} : (RGB){255, 0, 0});
	RGB otherColour = (ownCol == blue ? (RGB){255, 0, 0} : (RGB){0, 0, 255});
	int x, y;
	
	SDL_SetRenderDrawColor(renderer, myColour.r, myColour.g, myColour.b, 255);
	x = (int)(myPos.z*300.0f) + 330;
	y = -(int)(myPos.x*300.0f) + 330;
	rect = {x - 12, y - 12, 24, 24};
	SDL_RenderDrawRect(renderer, &rect);
	rect = {x - 14, y - 14, 28, 28};
	SDL_RenderDrawRect(renderer, &rect);
	
	SDL_SetRenderDrawColor(renderer, otherColour.r, otherColour.g, otherColour.b, 255);
	x = (int)(otherPos.z*300.0f) + 330;
	y = -(int)(otherPos.x*300.0f) + 330;
	rect = {x - 12, y - 12, 24, 24};
	SDL_RenderDrawRect(renderer, &rect);
	rect = {x - 14, y - 14, 28, 28};
	SDL_RenderDrawRect(renderer, &rect);
}
