/*
 * File:          two_wheel_first.c
 * Date:          22/01/2021
 * Description:   
 * Author:        Edmund Prager
 * Modifications:
 */

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>

#define PI 3.141592654
#define TIME_STEP 64
#define SENSORS_N 2
#define WHEELS_N 2
// distance between the centre of the robot and the front distance centre:
#define LFRONT 0.1
// distance between the centre of the robot and the rear distance centre:
#define LREAR 0.1
// half the side length of the areana:
#define SIDE_HLENGTH 1.19
// for helping it when the distance sensor is approximately orthogonal to the wall:
#define TOLERANCE 0.01
// tolerance for blocks/other robot in the way
#define ANGLE_TOLERANCE 0.2

// apologies for the fact that I am really used to C++ and haven't programmed in just C for ages, I'm sure there are better ways to organise my variables, but C structs are the closest thing I know of to C++ structs/classes

// struct to hold a 2D vector
typedef struct vec {
  float x, z;
} vec;

// functions for the struct 'vec'
float VecDot    (vec A, vec B)  { return A.x*B.x + A.z*B.z; }
vec VecAdd      (vec A, vec B)  { return (vec){.x=A.x+B.x, .z=A.z+B.z}; }
vec VecSubtract (vec A, vec B)  { return (vec){.x=A.x-B.x, .z=A.z-B.z}; }
vec VecMultiply (float a, vec A){ return (vec){.x=a*A.x,   .z=a*A.z};   }

// struct to hold the 2D vector form of a 2D line
typedef struct line {
  vec point; //  a point on the line
  vec dirHat; // a unit vector in the direction of the line
  vec normHat; // a unit vector normal to the line
} line;

// stores the vector form of the four walls of the arena (that the distance sensors should sense)
struct line sides[4];

char dsNames[SENSORS_N][10] = {"ds_front", "ds_rear"};
char wlNames[WHEELS_N][10] = {"wheel1", "wheel2"};

float destination[3] = {0, 0, 0}; // unimportant

// converts to degrees and rounds to nearest integer
int RD(double angle){
  return round(angle * 360.0f / 2 / PI);
}

// basically '%', but works on doubles, and makes the value between Pi and -PI, (Plus Pi and Minus Pi)
double MakePPMP(double angle){
  int sign = (angle > 0) - (angle < 0);
  if(sign == 0) return 0;
  while(sign*angle > PI){
    angle -= sign*2*PI;
  }
  return angle;
}

// modified version of 'atan()' to return values in range -PI to PI, rather than -PI/2 to PI/2
double CustomATan(float x, float z){
  double ret = atan(z/x);
  if(x < 0) return MakePPMP(ret + PI);
  return ret;
}

// returns whether the given vector position 'vec' lies within the arena
bool WithinArena(vec pos){
  return (pos.x>=-SIDE_HLENGTH-TOLERANCE && pos.x<=SIDE_HLENGTH+TOLERANCE && pos.z>=-SIDE_HLENGTH-TOLERANCE && pos.z<=SIDE_HLENGTH+TOLERANCE);
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

// finds all the points, within the arena, where a circle centre 'centre' and radius 'r' intersects with the four walls of the arena, and stores the bearings of these points from 'centre' in 'angles'; returns the number of points found
int CircleIntersectArena(vec centre, float r, double *angles){
  /*printf("CIA: (%f, %f), r=%f\n", centre.x, centre.z, r); // for debugging*/
  int n = 0; // counts the number of intersections found
  float d, root; vec p, a, shortest, offa; // 'a' and 'b' are relative to 'centre'
  for(int i=0; i<4; i++){ // loops through each wall
    p = VecSubtract(sides[i].point, centre); // vector from centre to somewhere on wall
    d = VecDot(p, sides[i].normHat); // signed shortest distance from centre to wall
    shortest = VecMultiply(d, sides[i].normHat); // shortest vector from centre to wall
    if(fabs(r - fabs(d)) <= TOLERANCE){ // wall is approximately tangent to circle
      // ...so use the shortest vector:
      angles[n] = CustomATan(shortest.x, shortest.z); // store angle to intersection point
      /*printf( "Tang intersect %d at: (%f, %f), angle %d\n",
              i, centre.x+shortest.x, centre.z+shortest.z, RD(angles[n])); // for debugging*/
      n++;
    } else if(fabs(d) < r){ // wall is within circle
      root = sqrt(r*r - d*d); // offset of intersection points from centre, parallel to wall
      offa = VecMultiply(root, sides[i].dirHat); // vector offset in one direction
      a = VecAdd(shortest, offa); // one intersection point
      if(WithinArena(VecAdd(centre, a))){ // check it isn't intersecting outside of the arena
        angles[n] = CustomATan(a.x, a.z); // store angle to intersection point
        /*printf( "Intersect %d at: (%f, %f), angle %d\n",
                i, centre.x+a.x, centre.z+a.z, RD(angles[n])); // for debugging*/
        n++;
      }
      a = VecSubtract(shortest, offa); // another intersection point
      if(WithinArena(VecAdd(centre, a))){ // check it isn't intersecting outside of the arena
        angles[n] = CustomATan(a.x, a.z); // store angle to intersection point
        /*printf( "Intersect %d at: (%f, %f), angle %d\n",
                i, centre.x+a.x, centre.z+a.z, RD(angles[n])); // for debugging*/
        n++;
      }
    } // otherwise, wall is completely outside of circle
  }
  return n;
}

// finds the pair of angles in 'anglesA' and 'anglesB' that are closest to PI radians apart; stores the angle from 'anglesA' in 'result' and returns success
bool FindAngleBestMatch(int nA, double *anglesA, int nB, double *anglesB, double *result){
  if(nA == 0) return false; if(nB == 0) return false;
  double delta[nA*nB]; // stores 'closeness to PI radians apart' for every possible paining
  for(int a=0; a<nA; a++){ // loops through anglesA
    for(int b=0; b<nB; b++){ // loops through anglesB
      delta[b + nB*a] = fabs(fabs(anglesA[a] - anglesB[b]) - PI); // stores the 'closeness to PI apart' for this pairing
    }
  }
  unsigned int index = FindMin(nA*nB, delta); // gets index in 'delta' of closest pairing
  double best = delta[index]; // gets closeness of closest pairing
  if(best < ANGLE_TOLERANCE){ // checks we are close enough for validity (if a block is in the way of one/both of the distance sensors, we would generally expect this to be false)
    *result = anglesA[index/nB]; // stores the angleA in result
    return true; // success
  }
  return false; // failure
}

//The arguments of the main function can be specified by the "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  
  printf("Controller starting.\n");
  
  // setting all the values for the wall lines in vector form
  sides[0] = (line){
    .point=(vec){.x=SIDE_HLENGTH, .z=-SIDE_HLENGTH},
    .dirHat=(vec){.x=0, .z=1},
    .normHat=(vec){.x=-1, .z=0}};
  sides[1] = (line){
    .point=(vec){.x=SIDE_HLENGTH, .z=SIDE_HLENGTH},
    .dirHat=(vec){.x=-1, .z=0},
    .normHat=(vec){.x=0, .z=-1}};
  sides[2] = (line){
    .point=(vec){.x=-SIDE_HLENGTH, .z=SIDE_HLENGTH},
    .dirHat=(vec){.x=0, .z=-1},
    .normHat=(vec){.x=1, .z=0}};
  sides[3] = (line){
    .point=(vec){.x=-SIDE_HLENGTH, .z=-SIDE_HLENGTH},
    .dirHat=(vec){.x=1, .z=0},
    .normHat=(vec){.x=0, .z=1}};
  
  
  wb_robot_init();
  
  printf("Robot initialised.\n");
  
  // sensors
  WbDeviceTag distanceSensors[SENSORS_N];
  for(int i=0; i<SENSORS_N; i++){
    distanceSensors[i] = wb_robot_get_device(dsNames[i]);
    wb_distance_sensor_enable(distanceSensors[i], TIME_STEP);
  }
  
  // wheels
  WbDeviceTag wheels[WHEELS_N];
  for(int i=0; i<SENSORS_N; i++){
    wheels[i] = wb_robot_get_device(wlNames[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
  // gps
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  // Perform simulation steps of TIME_STEP milliseconds and leave the loop when the simulation is over
  while (wb_robot_step(TIME_STEP) != -1) {
    
    //printf("C\n");
    
    // reading gps
    double position[3]; // stores the gps reading
    memcpy(position, wb_gps_get_values(gps), 3*sizeof(double));
    
    // ,,,,,,,, unimportant
    // destination
    float deltaX = destination[0] - position[0];
    float deltaZ = destination[2] - position[2];
    // distance from destination
    float sqDistance = deltaX*deltaX + deltaZ*deltaZ;
    // angle to destination
    double targetAngle = CustomATan(deltaX, deltaZ);
    // ```````
    
    // sensing
    double distances[SENSORS_N];
    for(int i=0; i<SENSORS_N; i++){
      distances[i] = wb_distance_sensor_get_value(distanceSensors[i]);
    }
    
    // calculating stuff
    float leftSpeed;
    float rightSpeed;
    
    vec pos = (vec){.x=position[0], .z=-position[2]}; // gets gps reading in 2D vector form
    float rFront = distances[0] + LFRONT; // radius of circle projected for front sensor
    float rRear = distances[1] + LREAR; // radius of circle projected for rear sensor
    double anglesFront[8]; double anglesRear[8]; // store the possible bearings each sensor could be at
    int nFront = CircleIntersectArena(pos, rFront, anglesFront); // finds these possible bearings
    int nRear = CircleIntersectArena(pos, rRear, anglesRear); // finds these possible bearings
    double orientation; // unimportant for now
    if(FindAngleBestMatch(nFront, anglesFront, nRear, anglesRear, &orientation)){ // finds if our readings are consistent (probably no blocks in the way)
      // ,,,,,,,, unimportant
      double deltaTheta = MakePPMP(targetAngle - orientation);
      int sodt = (deltaTheta > 0) - (deltaTheta < 0);
      // ````````
      leftSpeed = -0.4;
      rightSpeed = 0.4;
    } else { // readings aren't consistent (there are blocks in the way)
      leftSpeed = 0.4;
      rightSpeed = -0.4;
    }
    
    // so, in theory, the robots should rotate until one of their sensors is on the edge of a block and it should wiggle, looking at the edge of the block, as this should be where the readings go consistent, inconsistent, consistent etc...
    
    wb_motor_set_velocity(wheels[0], leftSpeed);
    wb_motor_set_velocity(wheels[1], rightSpeed);
  };
  
  // Enter your cleanup code here
  
  wb_robot_cleanup();
  
  return 0;
}