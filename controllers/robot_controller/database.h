// File:          database.h
// Date: 26/01/2021
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

#ifndef DATABASE
#define DATABASE

enum DataCompare{agree, disagree, wall};
enum Colour{dunno, blue, red};

char names[2] = {'b', 'r'};

class DataBase {
public:
  DataBase(){  
    colourNs[dunno] = colourNs[blue] = colourNs[red] = 0;
  }
  ~DataBase(){}
  
  // Compares a distance sensor reading with the database, and stores the new information. Return agreement
  bool LogReading(vec position, double *bearing, float distance, bool canConfirm);
  
  bool GetDestination(bool iamred, vec position, Colour *retCol, unsigned short *retInd);
  bool GetBlock(Colour colour, unsigned short index, vec *ret){
    if(index >= colourNs[colour]) return false;
    *ret = blocks[colour][index];
    return true;
  }
  
  // returns whether the given vector position 'vec' lies within the arena
  bool WithinArena(vec pos, bool tol){
    return (pos.x >= -SIDE_HLENGTH - tol*TOLERANCE &&
            pos.x <=  SIDE_HLENGTH + tol*TOLERANCE &&
            pos.z >= -SIDE_HLENGTH - tol*TOLERANCE &&
            pos.z <=  SIDE_HLENGTH + tol*TOLERANCE);
  }
  
  void Debug(){
    printf("Dunnos: %d\n", colourNs[dunno]);
    printf("Blues: %d\n", colourNs[blue]);
    printf("Reds: %d\n", colourNs[red]);
    printf("Questions: %d\n", questionsN);
    //for(int i=0; i<colourNs[dunno]; i++){ printf("%f, %f\n", blocks[dunno][i].z, blocks[dunno][i].x); }
    //printf("Blues:\n");
    //for(int i=0; i<colourNs[blue]; i++){ printf("%f, %f\n", blocks[blue][i].z, blocks[blue][i].x); }
    //printf("Reds:\n");
    //for(int i=0; i<colourNs[red]; i++){ printf("%f, %f\n", blocks[red][i].z, blocks[red][i].x); }
    //printf("Questions:\n");
    //for(int i=0; i<questionsN; i++){ printf("%f, %f\n", questions[i].z, questions[i].x); }
  }
  
private:

  vec blocks[3][64];
  unsigned short colourNs[3];
  vec questions[64];
  unsigned short questionsN = 0;
  
  // Adds a block seen by a distance sensor into the database
  void AddDunnoBlock(vec position){
    Colour colour = dunno;
    if(colourNs[red] >= 4){
      if(colourNs[blue] >= 4){
        AddQuestion(position);
        return;
      }
      colour = blue;
    } else if(colourNs[blue] >= 4){
      colour = red;
    }
    blocks[colour][colourNs[colour]++] = position;
  }
  
  void AddQuestion(vec position){
    printf("Question added at %f, %f\n", position.z, position.x);
    questions[questionsN++] = position;
  }
  void RemoveQuestion(unsigned short index){
    for(int i=index; i<questionsN-1; i++){
      questions[i] = questions[i+1];
    }
    questionsN--;
  }
};

#endif