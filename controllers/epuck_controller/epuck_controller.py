"""epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
#from numpy import double
from sys import set_coroutine_origin_tracking_depth
from tracemalloc import start
from numpy import reciprocal
from webots.controller import Robot, Motor, Receiver, Supervisor,Emitter
from epuck_move_to_destination import Epuck_move

import struct
import ast
import math


robotPath = []
path_set = False
# create the Robot instance.
robot = Supervisor()

epuck_move = Epuck_move(robot, 1, 5.6)
emitter = Emitter("emitter") # takes as input the emitter object
reciever = Receiver("receiver")

state = 0

robot_priority = 0

robot_pause = False


robot.getSelf().getField("customData").setSFString(str(""))

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

def update_state(new_state):
    state = new_state
    epuck_move.state = new_state


def collision_avoid():
    #if robot direction is forward, it comes across another robot going into the same spot and traveling in opposite direction
    #lower priority robot will move left or right direction if it is empty
    #once other robot has passed move back and continue path
    next_node  = robotPath[0]
    new_next_node = (next_node[0], next_node[1] + 0.25)
    robotPath[0] = new_next_node


# takes as input the sampling period
reciever.enable(1)

m = " " # define recieving message

start_pause = 0.0
elapsed_pause = 0.0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
   

    

    if len(robotPath) == 0 and not path_set:
        data = robot.getCustomData()
        if data != '':
            print("recieved this data :")
            print(data)
            robotPath = list(ast.literal_eval(ast.literal_eval(robot.getCustomData())[1]))
            robot_priority  = int(ast.literal_eval(robot.getCustomData())[0])
            path_set = True
            
    
    # assign robot priority 
    

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

        # compare priorities
        if int(m) > robot_priority:

            # perform low priority actions/sequence
            #print("I have low priority")

            # check state, if state is currently not paused, start the timer...
            if state == 0:
                print("lower prio updating state")
                start_pause = robot.getTime()
                elapsed_pause = 0.0
                update_state(1)
                collision_avoid()
                epuck_move.reset()
            
            else:
                
                # calculate elapsed time
                elapsed_pause = robot.getTime() - start_pause


        

    
    else:
        m = " "

        update_state(0)

        # pass elapased pause time here?
        epuck_move.elapsed_pause = elapsed_pause
        # reset the pause timing
        
        start_pause = 0.0



    
    if len(robotPath)>0 and path_set:
        
        
        print("state:" + str(state) +" prio: " + str(robot_priority) + "moving to " + str([robotPath[0][0], robotPath[0][1]]))
        done = epuck_move.moveToDestination([robotPath[0][0], robotPath[0][1]])
        
        

        

        if done:
            robotPath.pop(0)

    if path_set and len(robotPath) == 0:
        robot.getSelf().getField("customData").setSFString(str("done"))

        

    
    
    
    
    

# Enter here exit cleanup code.
