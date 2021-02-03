#include "scan.h"

Scan::Scan(Robot *_robot, int _timeStep){
	robot = _robot;
	timeStep = _timeStep;
	
	dsFront = new SensorDistance(robot, "ds_front", timeStep, 0.1, "ds_front");
	dsRear = new SensorDistance(robot, "ds_rear", timeStep, 0.1, "ds_rear");
	camera = new SensorColour(robot, "cs_colour", timeStep, "cs_colour");
	gps = new SensorGPS(robot, "gps", timeStep, "gps");
	compass = new SensorCompass(robot, "compass", timeStep, "compass");
}

vec Scan::ReadPosition(){
	return gps->getCoordinates();
}

double Scan::ReadBearing(){
	return compass->getOrientation().Bearing();
}

float Scan::ReadFrontDistance(){
	return dsFront->getDistance();
}

float Scan::ReadRearDistance(){
	return dsRear->getDistance();
}

Colour Scan::ReadColour(){
	int *rgb = camera->getColour();
	int blueness = rgb[2] - rgb[0];
	int deadZone = 20;
	blueness = (blueness > deadZone) - (blueness < -deadZone);
	if(blueness) return (blueness > 0 ? blue : red);
	return dunno;
}