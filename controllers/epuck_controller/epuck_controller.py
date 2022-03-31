"""epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
#from numpy import double
from webots.controller import Robot, Motor, Receiver
from epuck_move_to_destination import init_robot,moveToDestination

import struct
import ast


robotPath = []
# create the Robot instance.
robot = Robot()

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

path_counter = 0
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

init_robot(robot)

destinationCoordinate = [0.5,0.5]


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if(len(robotPath) == 0):
        path = robot.getCustomData()
        robotPath = ast.literal_eval(robot.getCustomData())
        moveToDestination(destinationCoordinate, robot)

    
        
    
    
    

    
    
    

# Enter here exit cleanup code.
