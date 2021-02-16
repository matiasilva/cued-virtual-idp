#include "scan.h"

Scan::Scan(Robot *_robot, int _timeStep){
	robot = _robot;
	timeStep = _timeStep;
	
	dsLeft = new SensorDistance(robot, "ds_left", timeStep, 0.05, "ds_left");
	dsRight = new SensorDistance(robot, "ds_right", timeStep, 0.05, "ds_right");
	camera = new SensorColour(robot, "cs_colour", timeStep, "cs_colour");
	gps = new SensorGPS(robot, "gps", timeStep, "gps");
	compass = new SensorCompass(robot, "compass", timeStep, "compass");
}

vec Scan::ReadPosition(){
	return gps->getCoordinates();
}

double Scan::ReadBearing(){
	return compass->getBearing();
}
vec Scan::ReadOrientation(){
	return compass->getOrientation();
}

float Scan::ReadLeftDistance(){
	return dsLeft->getDistance();
}

float Scan::ReadRightDistance(){
	return dsRight->getDistance();
}

Colour Scan::ReadColour(){
	int *rgb = camera->getColour();
	int blueness = rgb[2] - rgb[0];
	int deadZone = 20;
	blueness = (blueness > deadZone) - (blueness < -deadZone);
	if(blueness) return (blueness > 0 ? blue : red);
	return dunno;
}