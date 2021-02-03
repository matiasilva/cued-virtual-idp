#ifndef SCAN
#define SCAN

#include "header.h"
#include "sensor.h"

class Scan {
public:
	Scan(Robot *_robot, int _timeStep);
	
	vec ReadPosition();
	double ReadBearing();
	float ReadFrontDistance();
	float ReadRearDistance();
	Colour ReadColour();
	
private:
	Robot *robot;
	int timeStep;

	SensorCompass *compass;
	SensorDistance *dsFront;
	SensorDistance *dsRear;
	SensorColour *camera;
	SensorGPS *gps;
};

#endif