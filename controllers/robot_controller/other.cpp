// File:          	other.cpp
// Date: 			26/01/2021
// Description: 	Defines basic mathematical functions
// Author: 			Edmund Prager, Richard Monteiro
// Modifications: 	

#include "header.h"

template <int n> bool CIR(long int i, unsigned int N, const char message[n]){
	if(i < 0 || i >= N){
		printf("Array index: %d out of range: %d; %s\n", i, N, message);
		return false;
	}
	return true;
}

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

// returns the index of the smallest value in the array 'values', which is of length 'n'
unsigned int FindMax(int n, double *values){
  double max = values[0];
  unsigned int ret = 0;
  for(unsigned int i=1; i<n; i++){
    if(values[i] > max){
      max = values[i];
      ret = i;
    }
  }
  return ret;
}

// returns the index of the position closest position in the array 'positions', which is of length 'n', to 'position'
unsigned int FindClosest(int n, vec *positions, vec position){
	double sqDists[n];
	for(int i=0; i<n; i++){
		sqDists[i] = (positions[i] - position).SqMag();
	}
	return FindMin(n, sqDists);
}


vec VecSum(unsigned int n, vec *vectors){
	vec ret;
	for(unsigned int i=0; i<n; i++){
		ret += vectors[i];
	}
	return ret;
}

float LargestDistance(unsigned int N, vec *vectors){
	double sqDists[N*(N-1)];
	unsigned int n = 0;
	for(int i=0; i<N; i++){
		for(int j=0; j<N; j++){
			if(i == j) continue;
			sqDists[n++] = (vectors[i] - vectors[j]).SqMag();
		}
	}
	return sqrt(sqDists[FindMin(n, sqDists)]);
}