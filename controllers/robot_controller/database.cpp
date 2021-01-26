// File:          database.cpp
// Date: 26/01/2021
// Description: Contains map database of rectangle coordinates and known locations of blocks
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#include "database.h"

bool DataBase::LogReading(vec position, double *bearing, float distance, bool canConfirm){
    vec delta = {(float)(distance * cos(*bearing)), (float)(distance * sin(*bearing))};
    vec location = position + delta;
    
    // snapping to walls
    float tol = TOLERANCE * distance;
    bool change = false;
    if(location.z > SIDE_HLENGTH - tol){
      location.z = SIDE_HLENGTH;
      change = true;
    } else if(location.z < -SIDE_HLENGTH + tol){
      location.z = -SIDE_HLENGTH;
      change = true;
    }
    if(location.x > SIDE_HLENGTH - tol){
      location.x = SIDE_HLENGTH;
      change = true;
    } else if(location.x < -SIDE_HLENGTH + tol){
      location.x = -SIDE_HLENGTH;
      change = true;
    }
    if(change){ // did snap to a wall
      *bearing = (location - position).Bearing(); // correct robot's bearing
      return true; // tell robot that we agree with its reading
    } else { // didn't snap to a wall
      // a block is being seen?
      double near[12];
      Colour cs[12]; unsigned short bs[12];
      unsigned short nearN = 0;
      float sqDist;
      for(int c=0; c<3; c++){ // only checks dunnos, reds and blues (not questions)
        for(int b=0; b<colourNs[c]; b++){
          sqDist = (location - blocks[c][b]).SqMag();
          if(sqDist < BLOCK_WMAX*BLOCK_WMAX){ // reading is near a known block
            cs[nearN] = (Colour)c; bs[nearN] = b;
            near[nearN++] = sqDist;
          }
        }
      }
      if(nearN){ // is near at least one known block
        unsigned short index = FindMin(nearN, near); // find known block closest to
        blocks[cs[index]][bs[index]] = 0.5 * (blocks[cs[index]][bs[index]] + location); // move position of block to average of previous thought position and new reading
        printf("Block position updated at %f, %f\n", blocks[cs[index]][bs[index]].z, blocks[cs[index]][bs[index]].x);
        return true; // tell robot that we agree with its reading
      } else { // is not near a known block
        // a pre-existing question is being seen?
        double near[colourNs[question]]; unsigned short qs[colourNs[question]];
        unsigned short nearN = 0;
        float sqDist;
        for(int q=0; q<colourNs[question]; q++){
          sqDist = (location - blocks[question][q]).SqMag();
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
          printf("Question confirmed at %f, %f : from %f, %f on bear %d\n", blocks[question][qs[index]].z, blocks[question][qs[index]].x, position.z, position.x, RD(*bearing));
          RemoveQuestion(qs[index]);             //    a known block
          return true; // tell robot that we agree with its reading
        } else { // not near a wall, known block or pre-existing question
          AddQuestion(location); // add in a question
          return false; // tell robot to double check
        }
      }
    }
}

bool DataBase::GetDestination(bool iamred, vec position, Colour *retCol, unsigned short *retInd){
    //Debug();
    Colour colour = (iamred ? red : blue);
    if(colourNs[colour]){
      double sqdists[colourNs[colour]];
      for(int i=0; i<colourNs[colour]; i++){
        sqdists[i] = (blocks[colour][i] - position).SqMag();
      }
      unsigned short index = FindMin(colourNs[colour], sqdists);
      *retCol = colour;
      *retInd = index;
      return true;
    } else if(colourNs[dunno]){
      double sqdists[colourNs[dunno]];
      for(int i=0; i<colourNs[dunno]; i++){
        sqdists[i] = (blocks[dunno][i] - position).SqMag();
      }
      unsigned short index = FindMin(colourNs[dunno], sqdists);
      *retCol = dunno;
      *retInd = index;
      return true;
    } else if(colourNs[question]){
      double sqdists[colourNs[question]];
      for(int i=0; i<colourNs[question]; i++){
        sqdists[i] = (blocks[question][i] - position).SqMag();
      }
      unsigned short index = FindMin(colourNs[question], sqdists);
      *retCol = question;
      *retInd = index;
      return true;
    } else {
      return false;
    }
}