"""epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
#from numpy import double
from numpy import reciprocal
from webots.controller import Robot, Motor, Receiver, Supervisor,Emitter
from epuck_move_to_destination import Epuck_move

import struct
import ast
import math


robotPath = []
path_counter = 0
# create the Robot instance.
robot = Supervisor()

epuck_move = Epuck_move(robot, 1, 5.6)
emitter = Emitter("emitter") # takes as input the emitter object
reciever = Receiver("receiver")

robot_priority = 0


#  set the range of the robot
emitter.setRange(0.3)


def transmitMessage(message):
    #print("TRANSMITTING")

    # convert to bytes
    b = bytes(message, 'utf-8')
    
    emitter.send(b)

    #print("TRANSMITTED")
    pass

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

path_counter = 0
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)


destinationCoordinate = [0.5,0.5]

def getRobotBearing():
        north = robot.getDevice('compass').getValues()
        rad = math.atan2(north[0],north[2])
        bearing = (rad - 1.5708) / math.pi * 180.0
        if bearing < 0.0: bearing = bearing + 360.0

        return bearing



# takes as input the sampling period
reciever.enable(1)

m = " " # define recieving message



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
   

    

    if len(robotPath) == 0:
        data = robot.getCustomData()
        if data != '':
            #robotPath = list(ast.literal_eval(ast.literal_eval(robot.getCustomData())[1]))
            robotPath = [(-0.3999999999999999, 0.625), (-0.6499999999999999, 0.375), (-0.8999999999999999, 0.125)]
    
    # assign robot priority 
    robot_priority  = int(ast.literal_eval(robot.getCustomData())[0])

    # transmit priority
    transmitMessage(str(robot_priority)) 

    # if queue length is not equal to 0, we have received a message
    if reciever.getQueueLength() != 0:

        # get the data (in bytes)
        m = reciever.getData()

        # decode to string
        m=m.decode("utf-8")

        # delete head and set pointer to whatever is in the queue
        reciever.nextPacket()

        # print out the recieved message (in string format)
        print(m)

        # compare priorities
        if int(m) > robot_priority:
            print("I have low priority")

        else:
            print("I have high priority")

    
    else:
        m = " "


    
    
    if len(robotPath)>0:
        
        print("going to: " + str([robotPath[0][0], robotPath[0][1]]))
        done = epuck_move.moveToDestination([robotPath[0][0], robotPath[0][1]])

        if done:
            robotPath.pop(0)

    
    
    
    
    

# Enter here exit cleanup code.
