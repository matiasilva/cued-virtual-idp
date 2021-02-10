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

// SensorMotor Constructor
SensorMotor::SensorMotor(Robot* _robot, string name_in_device, string name) : Sensor(_robot, name_in_device)
{
	// Initialise actuator with node
	sensor_name = name;
	velocity = 0;
	is_actuator = true;

	sensor_object = robot->getMotor(device_name);
	sensor_object->setPosition(INFINITY);
	sensor_object->setVelocity(velocity);
}

// SensorMotor method to set the wheel's velocity
void SensorMotor::setVelocity(double v)
{
	velocity = v;
	sensor_object->setVelocity(velocity);
}

// SensorCompass Constructor
SensorCompass::SensorCompass(Robot* _robot, string name_in_device, int time_step, string name) : Sensor(_robot, name_in_device)
{
	// Initialise sensor with node
	sensor_name = name;
	sensor_object = robot->getCompass(device_name);
	sensor_object->enable(time_step);
}

// SensorCompass get orientation of GPS device method
vec SensorCompass::getOrientation()
{
	double values[3];
	vec orientation;

	memcpy(values, sensor_object->getValues(), 3 * sizeof(double));
	orientation = { (float)values[0], (float)values[2] };

	return orientation;
}

// SensorCompass get bearing of robot method (radians)
double SensorCompass::getBearing()
{
	return getOrientation().Bearing(true);
}

// SensorCompass get bearing of robot method (degrees)
double SensorCompass::getBearingDeg()
{
	return getOrientation().Bearing(true) / PI * 180.0;
}

// SensorEmitter Constructor
SensorEmitter::SensorEmitter(Robot* _robot, string name_in_device, string name) : Sensor(_robot, name_in_device)
{
	// Initialise sensor with node
	sensor_name = name;
	sensor_object = robot->getEmitter(device_name);
}

// SensorEmitter send data method
void SensorEmitter::send(const void* data, int size)
{
	sensor_object->send(data, size);

}

// SensorReceiver Constructor
SensorReceiver::SensorReceiver(Robot* _robot, string name_in_device, int time_step, string name) : Sensor(_robot, name_in_device)
{
	// Initialise sensor with node
	sensor_name = name;
	sensor_object = robot->getReceiver(device_name);
	sensor_object->enable(time_step);
}

// SensorReceiver method for obtaining data from receiver queue
double * SensorReceiver::getData()
{
	int size = sensor_object->getQueueLength();

	if (size == 0) {
		queue_empty = true;
		return NULL;
	}
	else {
		static double data[PACKET_LENGTH];
		memcpy(data, sensor_object->getData(), PACKET_LENGTH * sizeof(double));
		sensor_object->nextPacket();

		if (size - 1 > 0) queue_empty = false;
		
		return data;
	}
}