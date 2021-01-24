/*
 * File:          epuck_go_forward.c
 * Date:          22/01/2021
 * Description:   Basic control of epuck robot
 * Author:        Edmund Prager
 * Modifications: 
 */

#include <webots/robot.h>
#include <webots/motor.h>

#define MAX_SPEED 6.28
#define TIME_STEP 64

// The arguments of the main function can be specified by the "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  // set to speed control
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  
  wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);
  
  // Perform simulation steps of TIME_STEP milliseconds and leave the loop when the simulation is over
  while (wb_robot_step(TIME_STEP) != -1) {
    /*w
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
