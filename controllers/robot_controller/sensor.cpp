// File:          sensor.cpp
// Date: 26/01/2021
// Description: Handling of sensor data
// Author: Edmund Prager, Richard Monteiro
// Modifications: None

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include "sensor.h"

// All the webots classes are defined in the "webots" namespace
using namespace webots;


// Sensor Constructor
Sensor::Sensor(Robot* _robot, string name_in_device)
{
  device_name = name_in_device;
  robot = _robot;
}

// SensorDistance Constructor
SensorDistance::SensorDistance(Robot *_robot, string name_in_device, int time_step, float offset, string name) : Sensor(_robot, name_in_device)
{
  
  // Initialise sensor with node
  sensor_name = name;
  sensor_object = robot->getDistanceSensor(device_name);
  sensor_offset = offset;
  sensor_object->enable(time_step);
}

//SensorDistance return measured distance method
double SensorDistance::getDistance()
{
  double value = sensor_object->getValue();



  return value+sensor_offset;
}

// SensorColour Constructor
SensorColour::SensorColour(Robot* _robot, string name_in_device, int time_step, string name) : Sensor(_robot, name_in_device)
{

	// Initialise sensor with node
	sensor_name = name;
	sensor_object = robot->getCamera(device_name);
	sensor_object->enable(time_step);
}

// SensorColour get RGB colour method
int * SensorColour::getColour()
{
	image = sensor_object->getImage();
	static int coloursRGB[3];
	coloursRGB[0] = sensor_object->imageGetRed(image, 1, 0, 0);
	coloursRGB[1] = sensor_object->imageGetGreen(image, 1, 0, 0);
	coloursRGB[2] = sensor_object->imageGetBlue(image, 1, 0, 0);

	return coloursRGB;

}

// SensorGPS Constructor
SensorGPS::SensorGPS(Robot* _robot, string name_in_device, int time_step, string name) : Sensor(_robot, name_in_device)
{
	// Initialise sensor with node
	sensor_name = name;
	sensor_object = robot->getGPS(device_name);
	sensor_object->enable(time_step);
}

// SensorGPS get coordinates of GPS device method
vec SensorGPS::getCoordinates()
{
	double values[3];
	vec coordinates;

	memcpy(values, sensor_object->getValues(), 3 * sizeof(double));
	coordinates = { (float)values[2], (float)values[0] };

	return coordinates;
}