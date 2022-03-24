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

WbNodeRef epucks[2];
double start_translations[2][3];

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
  char name[20];
  for(int i=0;i<2;i++){
    sprintf(name, "EPUCK%d", i);
    epucks[i] = wb_supervisor_node_get_from_def(name);
  }
  
  
  start_translations[0][0] = 0.031;
  start_translations[0][1] = 0.334;
  start_translations[0][2] = -0.00404;
  start_translations[1][0] = -0.38;
  start_translations[1][1] = 0.0068;
  start_translations[1][2] = -0.00189;
  
  
  for(int i=0;i<2;i++){
 
    WbFieldRef trans_field = wb_supervisor_node_get_field(epucks[i], "translation");
    const double t[3] = {start_translations[i][0], start_translations[i][1], start_translations[i][2]};
    wb_supervisor_field_set_sf_vec3f(trans_field, t);
  }
  
  
  
  
  while (wb_robot_step(TIME_STEP) != -1) {
   
    
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
