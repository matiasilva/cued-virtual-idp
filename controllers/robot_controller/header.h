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
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

// ========== comment this out for windows
#include <SDL2/SDL.h>
// ==========

#define PI 3.141592654
#define SENSORS_N 2
#define MOTORS_N 5 // left wheel, right wheel, arm, left claw, right claw
// half the side length of the arena:
#define SIDE_HLENGTH 1.19
// for helping it calibrate when pointing towards a wall:
#define TOLERANCE 0.07
// convert from difference in wheel speeds to angular velocity
#define TURN_FACTOR 0.666666667
// the largest possible horizontal width of a block
#define BLOCK_WMAX 0.0707107
// the amount of number transmitted in each packet through robot communication
#define PACKET_LENGTH 6
// the uncertainty in the location of the block - defined experimentally
#define BLOCK_POS_UNCERTAINTY 0.1

// All the webots classes are defined in the "webots" namespace
using namespace webots;

template <int n> bool CIR(long int i, unsigned int N, const char message[n]);

// ----- Basic mathematical functions (defined in other.cpp) -----
// converts to degrees and rounds to nearest integer
int RD(const double angle);

// basically '%', but works on doubles, and makes the value between Pi and -PI, (Plus Pi and Minus Pi)
double MakePPMP(const double angle);

// returns the index of the smallest value in the array 'values' of length 'n'
unsigned int FindMin(int n, double *values);

// returns the index of the largest value in the array 'values' of length 'n'
unsigned int FindMax(int n, double *values);


// ----- Data structures -----
// struct to hold a 2D vector
struct vec {
  float z, x; // z is east, x is north, so coordinates are defined as (z, x), not (x, y)
  friend float operator*(const vec& vec1, const vec& vec2){ // vector dot product
    return vec1.z*vec2.z + vec1.x*vec2.x;
  }
  friend vec operator*(const vec& vec1, const float f){ // multiplication by scalar
    return {vec1.z*f, vec1.x*f};
  }
  friend vec operator*(const float f, const vec& vec1){ // multiplication by scalar
    return {vec1.z*f, vec1.x*f};
  }
  friend vec operator/(const vec& vec1, const float f){ // division by scalar
    return {vec1.z/f, vec1.x/f};
  }
  friend vec operator+(const vec& vec1, const vec& vec2){ // vector addition
    return {vec1.z+vec2.z, vec1.x+vec2.x};
  }
  vec operator+=(const vec& vec1){
  	z += vec1.z;
  	x += vec1.x;
  	return *this;
  }
  friend vec operator-(const vec& vec1, const vec& vec2){ // vector subtraction
    return {vec1.z-vec2.z, vec1.x-vec2.x};
  }
  friend bool operator==(const vec& vec1, const vec& vec2){
  	return (vec1.z == vec2.z && vec1.x == vec2.x);
  }
  friend bool operator!=(const vec& vec1, const vec& vec2){
  	return (vec1.z != vec2.z || vec1.x != vec2.x);
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

struct RGB {
	unsigned char r, g, b;
	
	bool Red(){
		return (r > 1.4*g && r > 1.4*b);
	}
	bool Blue(){
		return (b > 1.4*g && b > 1.4*r);
	}
};


vec VecSum(unsigned int n, vec *vectors);
float LargestDistance(unsigned N, vec *vectors);

// type of blocks stored in data base
enum Colour{dunno, blue, red, question};
// dunno:		pretty sure there's a block here, dont know the colour
// blue:		pretty sure there's a blue block here
// red:			pretty sure there's a red block here

// type of control command for sending data
enum Control{addBlock, removeBlock, robotPos};
// add:         Add given data to database - by creating new block or modifying existing block
// remove:      Remove block from database - if it exists.
// robotPos:    Communication of other robot position.

// classes
class Test;

class Navigation; // deals with input and output
class DataBase; // deals with arena information
class Scan;
class Sensor;
class SensorDistance;
class SensorColour;
class SensorGPS;
class SensorCompass;
class SensorEmitter;
class SensorReceiver;

// state stuff
class StateManager;
// parent states
class State;
class TemporaryState;
class InsertedState;
// states:
class DefaultState;
class WaitState;
class DCheckingState;
class InitialScanState;
class MovingToState;
class FindingLostState;

// Visualiser
class Visualiser;


// ----- Mathematical functions involving custom data structures -----
unsigned int FindClosest(int n, vec *positions, vec position);

#endif