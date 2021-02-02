// File:          sensor.h
// Date: 26/01/2021
// Author: Edmund Prager, Richard Monteiro
// Modifications: None


#ifndef SENSOR
#define SENSOR

#include "header.h"

using namespace std;

/**
   Sensor class assigning ay generic sensor/actuator object.
   Sensor is a parent class assigning common sensor/actuator attributes and methods, such as:
    1. sensor_name - string value to be displayed in command line representing the sensor;
    2. is_actuator - Boolean value determining whether sensor object is sensor or actuator;
    3. device_name - string representing sensor name as device in PROTO robot file;
    4. Robot - assigned robot to that sensor.

    Attributes are protected and not private as to allow for easy reference in sensor.cpp.
*/
class Sensor
{
protected:
    string sensor_name;
    bool is_actuator = false;
    string device_name;
    Robot* robot;
public:

    /**
        Sensor class constructor
        #param: Assigned robot (_robot) and device name (name_in_device) within that robot. 
    */
    Sensor(Robot* _robot, string name_in_device);
 
    
};

/**
   SensorDistance class representing a distance sensor object, derived from Sensor class.
   SensorDistance shares all attributes from Sensor, adding:
    1. sensor_object - Object representing Webots distance sensor, which is encapsulated by the newly defined SensorDistance object.
    2. sensor_offset - Float containing values for offset in distance value from sensor.

    Attributes are protected and not private as to allow for easy reference in sensor.cpp.
*/
class SensorDistance : public Sensor
{
protected:
    DistanceSensor* sensor_object;
    float sensor_offset;
public:
    /**
        SensorDistance class constructor
        #param: Assigned robot (_robot, type Robot, takes in address), device name (name_in_device, type string) within that robot.,
                time step used for simulation (time_step, type int), display name for sensor (name, type string) and
                offset for distance values (offset, type float).

        Method calls parent constructor.
    */
    SensorDistance(Robot* _robot, string name_in_device, int time_step, float offset, string name = "Distance Sensor");
    
    /**
        Intermediary "get" method which obtains distance measured from sensor.
        #returns: distance (double) measured by sensor, normalised and offset to centre of robot.
    */
    double getDistance();
};

/**
   SensorColour class representing a camera object, derived from Sensor class.
   SensorColour shares all attributes from Sensor, adding:
    1. sensor_object - Object representing Webots camera, which is encapsulated by the newly defined SensorColour object.
    2. image - Pointer char to hold image pixel value.

    Attributes are protected and not private as to allow for easy reference in sensor.cpp.
*/
class SensorColour : public Sensor
{
protected:
    Camera* sensor_object;
    const unsigned char* image;
public:
    /**
        SensorColour class constructor
        #param: Assigned robot (_robot, type Robot, takes in address), device name (name_in_device, type string) within that robot.,
                time step used for simulation (time_step, type int), display name for sensor (name, type string).

        Method calls parent constructor.
    */
    SensorColour(Robot* _robot, string name_in_device, int time_step, string name = "Color Sensor");

    /**
        Intermediary "get" method which obtains colour, in RGB, measured from camera.
        #returns: pointer to RGB array representing the pixel directly in front of the camera.
    */
    int * getColour();
};

/**
   SensorGPS class representing a GPS object, derived from Sensor class.
   SensorGPS shares all attributes from Sensor, adding:
    1. sensor_object - Object representing Webots GPS, which is encapsulated by the newly defined SensorGPS object.

    Attributes are protected and not private as to allow for easy reference in sensor.cpp.
*/
class SensorGPS : public Sensor
{
protected:
    GPS* sensor_object;
public:
    /**
        SensorGPS class constructor
        #param: Assigned robot (_robot, type Robot, takes in address), device name (name_in_device, type string) within that robot,
                time step used for simulation (time_step, type int), display name for sensor (name, type string).

        Method calls parent constructor.
    */
    SensorGPS(Robot* _robot, string name_in_device, int time_step, string name = "GPS Sensor");

    /**
        Intermediary "get" method which obtains coordinates, as 2D vector (vec object), measured from GPS.
        #returns: vec object with coordinates of GPS.
    */
    vec getCoordinates();
};

/**
   SensorMotor class representing a RotationalMotor object, derived from Sensor class - although it is an actuator.
   SensorMotor shares all attributes from Sensor, adding:
    1. sensor_object - Object representing Webots RotationalMotor, which is encapsulated by the newly defined SensorMotor object.
    2. velocity - attribute which keeps track of current set velocity of motor.

    Attributes are protected and not private as to allow for easy reference in sensor.cpp.
*/
class SensorMotor : public Sensor
{
protected:
    Motor* sensor_object;
    double velocity;
public:
    /**
        SensorMotor class constructor
        #param: Assigned robot (_robot, type Robot, takes in address), device name (name_in_device, type string) within that robot,
               display name for sensor (name, type string).

        Method calls parent constructor.
    */
    SensorMotor(Robot* _robot, string name_in_device, string name = "Motor");

    /**
        Intermediary "set" method that changes motor velocity
        #param: double representing new velocity to set the motor.
    */
    void setVelocity(double v);
};

/**
   SensorCompass class representing a compass object, derived from Sensor class.
   SensorCompass shares all attributes from Sensor, adding:
    1. sensor_object - Object representing Webots Compass, which is encapsulated by the newly defined SensorCompass object.

    Attributes are protected and not private as to allow for easy reference in sensor.cpp.
*/
class SensorCompass: public Sensor
{
protected:
    Compass* sensor_object;
public:
    /**
        SensorCompass class constructor
        #param: Assigned robot (_robot, type Robot, takes in address), device name (name_in_device, type string) within that robot,
                time step used for simulation (time_step, type int), display name for sensor (name, type string).

        Method calls parent constructor.
    */
    SensorCompass(Robot* _robot, string name_in_device, int time_step, string name = "Compass Sensor");

    /**
        Intermediary "get" method which obtains orientation, as 2D vector (vec object), measured from compass.
        #returns: vec object corresponding to world orientation.
    */
    vec getOrientation();

    /**
        Intermediary "get" method which obtains bearing (from North - positive x direction), as double, measured from compass.
        #returns:  double corresponding to world orientation bearing - in radians.
    */
    double getBearing();

    /**
        Intermediary "get" method which obtains bearing (from North - positive x direction), as double, measured from compass.
        #returns:  double corresponding to world orientation bearing - in degrees.
    */
    double getBearingDeg();
};

/**
   SensorEmitter class representing an Emitter object, derived from Sensor class.
   SensorEmitter shares all attributes from Sensor, adding:
    1. sensor_object - Object representing Webots Emitter, which is encapsulated by the newly defined SensorEmitter object.

    Attributes are protected and not private as to allow for easy reference in sensor.cpp.
*/
class SensorEmitter : public Sensor
{
protected:
    Emitter* sensor_object;
public:
    /**
        SensorEmitter class constructor
        #param: Assigned robot (_robot, type Robot, takes in address), device name (name_in_device, type string) within that robot,
                display name for sensor (name, type string).

        Method calls parent constructor.
    */
    SensorEmitter(Robot* _robot, string name_in_device, string name = "Emitter");

    /**
        Emitter method that "sends" data inputted.
        #parameters: data - Holds pointer for data value (which can be of any type), size - size of data to be sent in bytes (set to PACKET_LENGTH as default).
    */
    void send(const void *data, int size = PACKET_LENGTH * sizeof(double));
};


/**
   SensorReceiver class representing a Receiver object, derived from Sensor class.
   SensorReceiver shares all attributes from Sensor, adding:
    1. sensor_object - Object representing Webots Emitter, which is encapsulated by the newly defined SensorReceiver object.
    2. queue_empty - Represents whether reception queue is empty - in case of multiple data waiting to be received.

    Attributes are protected and not private as to allow for easy reference in sensor.cpp.
*/
class SensorReceiver : public Sensor
{
protected:
    Receiver* sensor_object;
    bool queue_empty;
public:
    /**
        SensorReceiver class constructor
        #param: Assigned robot (_robot, type Robot, takes in address), device name (name_in_device, type string) within that robot,
                display name for sensor (name, type string), time step used for simulation (time_step, type int)

        **Receiver only allows transfer of data composed of double type arrays.**

        Method calls parent constructor.
    */
    SensorReceiver(Robot* _robot, string name_in_device, int time_step, string name = "Receiver");

    /**
        Receiver getData method that "captures" data emitted by other robot.
        #returns: data in head of reception queue - data emitted (double type).
        If reception queue empty, returns NULL.
    */
    double * getData();

    /**
        Receiver getter method
        #returns: whether or not reception queue is empty | value in queue_empty attribute.
    */
    bool getQueueEmpty() { return queue_empty; }
};
#endif