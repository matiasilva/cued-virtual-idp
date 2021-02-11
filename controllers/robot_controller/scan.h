#ifndef SCAN
#define SCAN

#include "header.h"
#include "sensor.h"

class Scan {
public:
	Scan(Robot *_robot, int _timeStep);
	
	vec ReadPosition();
	double ReadBearing();
	float ReadLeftDistance();
	float ReadRightDistance();
	Colour ReadColour();
	
private:
	Robot *robot;
	int timeStep;

	SensorCompass *compass;
	SensorDistance *dsLeft;
	SensorDistance *dsRight;
	SensorColour *camera;
	SensorGPS *gps;
};

#endif