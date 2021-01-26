// File:          	other.cpp
// Date: 			26/01/2021
// Description: 	Defines basic mathematical functions
// Author: 			Edmund Prager, Richard Monteiro
// Modifications: 	

#include "header.h"

// converts from radians to degrees and rounds to the nearest integer
int RD(const double angle){
  return round(angle * 360.0f / 2 / PI);
}

// basically '%', but works on doubles, and makes the value between Pi and -PI, (Plus Pi and Minus Pi)
double MakePPMP(const double angle){
  double ret = angle;
  int sign = (ret > 0) - (ret < 0);
  if(sign == 0) return 0;
  while(sign*ret > PI){
    ret -= sign*2*PI;
  }
  return ret;
}

// returns the index of the smallest value in the array 'values', which is of length 'n'
unsigned int FindMin(int n, double *values){
  double min = values[0];
  unsigned int ret = 0;
  for(unsigned int i=1; i<n; i++){
    if(values[i] < min){
      min = values[i];
      ret = i;
    }
  }
  return ret;
}