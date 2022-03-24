/*
 * File:          pos_communication.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <stdio.h>
#include <webots/supervisor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("EPUCK");
  
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  
  const double INITIAL[3] = { 0.031, 0.334, -0.00404 };
  wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);
  wb_supervisor_node_reset_physics(robot_node);
  
  
  
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *values = wb_supervisor_field_get_sf_vec3f(trans_field);
    printf("MY_ROBOT is at position: %g %g %g\n", values[0], values[1], values[2]);
    
    
    
    
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
