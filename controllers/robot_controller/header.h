#ifndef HEADER
#define HEADER

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
// half the side length of the arena:
#define SIDE_HLENGTH 1.19
// for helping it calibrate when pointing towards a wall:
#define TOLERANCE 0.07
// convert from difference in wheel speeds to angular velocity
#define TURN_FACTOR 0.666666667
// the largest possible horizontal width of a block
#define BLOCK_WMAX 0.0707107

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// ----- Basic mathematical functions (defined in other.cpp) -----
// converts to degrees and rounds to nearest integer
int RD(const double angle);

// basically '%', but works on doubles, and makes the value between Pi and -PI, (Plus Pi and Minus Pi)
double MakePPMP(const double angle);

// returns the index of the smallest value in the array 'values' of length 'n'
unsigned int FindMin(int n, double *values);

// ----- Data structures -----
// struct to hold a 2D vector
struct vec {
  float z, x; // z is east, x is north, so coordinates are defined as (z, x), not (x, y)
  friend float operator*(vec vec1, const vec vec2){ // vector dot product
    return vec1.z*vec2.z + vec1.x*vec2.x;
  }
  friend vec operator*(vec vec1, const float f){ // multiplication by scalar
    return {vec1.z*f, vec1.x*f};
  }
  friend vec operator*(const float f, vec vec1){ // multiplication by scalar
    return {vec1.z*f, vec1.x*f};
  }
  friend vec operator+(vec vec1, const vec vec2){ // vector addition
    return {vec1.z+vec2.z, vec1.x+vec2.x};
  }
  friend vec operator-(vec vec1, const vec vec2){ // vector subtraction
    return {vec1.z-vec2.z, vec1.x-vec2.x};
  }
  double Bearing(bool reversed=false){ // returns bearing of vector measured anti-clockwise from east (+ve z)
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
  float SqMag(){ // square magnitude of vector
    return z*z + x*x;
  }
};

// type of blocks stored in data base
enum Colour{dunno, blue, red, question};
// dunno:		pretty sure there's a block here, dont know the colour
// blue:		pretty sure there's a blue block here
// red:			pretty sure there's a red block here
// question:	there may be a block here

class Navigation; // deals with input and output
class DataBase; // deals with arena information

#endif