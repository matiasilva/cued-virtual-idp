#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>

#define PI 3.141592654
#define SENSORS_N 2
#define WHEELS_N 2
// half the side length of the areana:
#define SIDE_HLENGTH 1.19
// for helping it when the distance sensor is approximately orthogonal to the wall:
#define TOLERANCE 0.07
// tolerance for blocks/other robot in the way
#define ANGLE_TOLERANCE 0.2
// maximum angular velocity it thinks it will ever reach (rad per sec)
#define MAX_OMEGA 4
// convert from difference in wheel speeds to angular velocity
#define TURN_FACTOR 0.666666667
// the largest possible horizontal width of a block
#define BLOCK_WMAX 0.0707107

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// ----- Basic mathematical functions -----
// converts to degrees and rounds to nearest integer
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

// returns the index of the smallest value in the array 'values' of length 'n'
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

// struct to hold a 2D vector
struct vec {
  float z, x;
  friend float operator*(vec vec1, const vec vec2){
    return vec1.z*vec2.z + vec1.x*vec2.x;
  }
  friend vec operator*(vec vec1, const float f){
    return {vec1.z*f, vec1.x*f};
  }
  friend vec operator*(const float f, vec vec1){
    return {vec1.z*f, vec1.x*f};
  }
  friend vec operator+(vec vec1, const vec vec2){
    return {vec1.z+vec2.z, vec1.x+vec2.x};
  }
  friend vec operator-(vec vec1, const vec vec2){
    return {vec1.z-vec2.z, vec1.x-vec2.x};
  }
  double Bearing(bool reversed=false){
    if(x == 0){
      if(z == 0) printf("Warning: bearing of zero vector taken.\n");
      return PI*(z < 0);
    } else if(z == 0){
      return MakePPMP(PI/2 + PI*(x < 0));
    }
    double ret = atan(x/z);
    if(reversed) ret += PI;
    if(z < 0) ret += PI;
    return MakePPMP(ret);
  }
  float SqMag(){
    return z*z + x*x;
  }
};