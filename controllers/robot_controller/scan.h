#ifndef SCAN
#define SCAN

#include "header.h"
#include "sensor.h"

class Scan {
public:
	Scan(Robot *_robot, int _timeStep);
	
	// functions to read the sensors
	vec ReadPosition();
	double ReadBearing();
	vec ReadOrientation();
	float ReadLeftDistance();
	float ReadRightDistance();
	Colour ReadColour();
	
private:
	Robot *robot;
	int timeStep;
	
	// sensor objects
	SensorCompass *compass;
	SensorDistance *dsLeft;
	SensorDistance *dsRight;
	SensorColour *camera;
	SensorGPS *gps;
};

#endif