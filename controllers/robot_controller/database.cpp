// File:          database.cpp
// Date: 26/01/2021
// Description: Contains map database of rectangle coordinates and known locations of blocks
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#include "database.h"

bool DataBase::LogReading(vec position, double bearing, float distance, bool canConfirm, Key key){
	vec delta = {(float)(distance * cos(bearing)), (float)(distance * sin(bearing))};
	vec location = position + delta;

	// we're looking at a wall
	float tol = TOLERANCE * distance;
	if(location.z > SIDE_HLENGTH - tol) return true;
	if(location.z < -SIDE_HLENGTH + tol) return true;
	if(location.x > SIDE_HLENGTH - tol) return true;
	if(location.x < -SIDE_HLENGTH + tol) return true;
    
	// a block is being seen?
	double near[12];
	Colour cs[12]; unsigned short bs[12];
	unsigned short nearN = 0;
	float sqDist;
	for(int c=0; c<3; c++){ // only checks dunnos, reds and blues (not questions)
		for(int b=0; b<colourNs[c]; b++){
			sqDist = (location - blocks[c][b].position).SqMag();
			if(sqDist < BLOCK_WMAX*BLOCK_WMAX){ // reading is near a known block
				cs[nearN] = (Colour)c; bs[nearN] = b;
				near[nearN++] = sqDist;
			}
		}
	}
	if(nearN){ // is near at least one known block
		unsigned short index = FindMin(nearN, near); // find known block closest to
		//blocks[cs[index]][bs[index]] = {0.5 * (blocks[cs[index]][bs[index]].position + location), key}; // move position of block to average of previous thought position and new reading
		blocks[cs[index]][bs[index]].position = 0.5 * (blocks[cs[index]][bs[index]].position + location);
		blocks[cs[index]][bs[index]].seenBy = key;
		printf("Block position updated at %f, %f\n", blocks[cs[index]][bs[index]].position.z, blocks[cs[index]][bs[index]].position.x);
		return true; // tell robot that we agree with its reading
	} else { // is not near a known block
		// a pre-existing question is being seen?
		double near[colourNs[question]]; unsigned short qs[colourNs[question]];
		unsigned short nearN = 0;
		float sqDist;
		for(int q=0; q<colourNs[question]; q++){
			sqDist = (location - blocks[question][q].position).SqMag();
			if(sqDist < BLOCK_WMAX*BLOCK_WMAX){ // reading is near a pre-existing question
				if(!canConfirm) return true; // can't confirm its a block, but have logged this question already
					qs[nearN] = q;
					near[nearN++] = sqDist;
				}
			}
		if(nearN){ // not near a wall or a known block, but near a pre-existing question
			if(!canConfirm) return true; // can't confirm its a block, but have logged this question already
			unsigned short index = FindMin(nearN, near); // find pre-existing question closest to
			AddDunnoBlock(blocks[question][qs[index]]); // turn question into
			printf("Question confirmed at %f, %f : from %f, %f on bear %d\n", blocks[question][qs[index]].position.z, blocks[question][qs[index]].position.x, position.z, position.x, RD(bearing));
			RemoveQuestion(qs[index]);             //    a known block
			return true; // tell robot that we agree with its reading
		} else { // not near a wall, known block or pre-existing question
			unsigned short int primaryKey = GenerateKey(); // generate primary key for question
			AddQuestion({location, key, primaryKey, 0, question}); // add in a question
			return false; // tell robot to double check
		}
	}
}

bool DataBase::GetDestination(bool iamred, vec position, Colour *retCol, unsigned short *retInd){
    //Debug();
    Colour colour = (iamred ? red : blue);
    if(colourNs[colour]){
      double sqdists[colourNs[colour]];
      for(int i=0; i<colourNs[colour]; i++){
        sqdists[i] = (blocks[colour][i].position - position).SqMag();
      }
      unsigned short index = FindMin(colourNs[colour], sqdists);
      *retCol = colour;
      *retInd = index;
      return true;
    } else if(colourNs[dunno]){
      double sqdists[colourNs[dunno]];
      for(int i=0; i<colourNs[dunno]; i++){
        sqdists[i] = (blocks[dunno][i].position - position).SqMag();
      }
      unsigned short index = FindMin(colourNs[dunno], sqdists);
      *retCol = dunno;
      *retInd = index;
      return true;
    } else if(colourNs[question]){
      double sqdists[colourNs[question]];
      for(int i=0; i<colourNs[question]; i++){
        sqdists[i] = (blocks[question][i].position - position).SqMag();
      }
      unsigned short index = FindMin(colourNs[question], sqdists);
      *retCol = question;
      *retInd = index;
      return true;
    } else {
      return false;
    }
}

short unsigned int DataBase::GenerateKey()
{
	static unsigned int previous_key = 0; // create static var to store previous key value - such that no primary key is ever repeated
	previous_key++; // No overflow errors occur until previous_key = 65535 - well beyond the possible keys used in this program.
	return previous_key;
}

bool DataBase::VerifyPrimaryKey(unsigned short int pkey)
{
	bool found = false;
	for (int i = 0; i < 4 * 32; i++) {
		if (blocks[(int)i / 32][i % 32].primaryKey == pkey) found = true;
	}
	return found;
}

int DataBase::FindByPrimaryKey(unsigned short int pkey)
{
	int index = -1;
	for (int i = 0; i < 4 * 32; i++) {
		if (blocks[(int)i / 32][i % 32].primaryKey == pkey) index  = i;
	}
	return index;
}

int DataBase::FindByPosition(vec pos)
{
	vec pos_db;
	for (int i = 0; i < 4 * 32; i++) {
		Block block_db = blocks[(int)i / 32][i % 32];
		pos_db = block_db.position;
		// If both x' and y' (new block) correspond to existing x, y with uncertainty BLOCK_POS_UNCERTAINTY u ( (x - u) < x' < (x + u), (y - u) < y' < (y + u)), blocks are deemed equal.
		if (pos.x < pos_db.x + BLOCK_POS_UNCERTAINTY && pos.x > pos_db.x - BLOCK_POS_UNCERTAINTY
			&& pos.z < pos_db.z + BLOCK_POS_UNCERTAINTY && pos.z > pos_db.z - BLOCK_POS_UNCERTAINTY) return i;
	}
	return -1;

}

void DataBase::ModifyBlockByPrimaryKey(Block* block, bool forceChange)
{
	// find block in database
    int found_index = FindByPrimaryKey(block->primaryKey);
	if (found_index == -1) return; // block with primary key not in database

	Block block_db = blocks[(int)found_index / 32][found_index % 32];

	// replace block in database, only if forceChange is set to true, or if colour parameters do not match - representing a relevant change in value for the block.
	if (forceChange || block_db.blockColour != block->blockColour) {
		blocks[(int)found_index / 32][found_index % 32] = *block;
	}
	
}

void DataBase::ModifyBlockByIndex(Block* block, int index, bool forceChange)
{

	Block block_db = blocks[(int)index / 32][index % 32];

	// set foreign key and primary keys to match
	blocks[(int)index / 32][index % 32].foreignKey = block->foreignKey;
	block->primaryKey = block_db.primaryKey;

	// replace block in database, only if forceChange is set to true, or if colour parameters do not match - representing a relevant change in value for the block.
	if (forceChange || block_db.blockColour != block->blockColour) {
		blocks[(int)index / 32][index % 32] = *block;
	}

}

void DataBase::AddNewBlock(Block* block)
{
	Colour col = block->blockColour;
	unsigned short int primaryKey = GenerateKey(); // generate primary key for block
	block->primaryKey = primaryKey;

	if ((col == red || col == blue) && colourNs[col] >= 4)
	{
		block->blockColour = question;
		blocks[question][colourNs[col]] = *block;
	}
	else {
		blocks[col][colourNs[col]] = *block;
	}

}