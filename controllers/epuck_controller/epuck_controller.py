"""epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
#from numpy import double
from sys import set_coroutine_origin_tracking_depth
from tracemalloc import start
from numpy import reciprocal
from webots.controller import Robot, Motor, Receiver, Supervisor,Emitter
from epuck_move_to_destination import Epuck_move

from a_star import search

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

robots_in_range = {}



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

#origin cordinates
origin = (-1.4, -0.875)

#distance between each square
node_dist = 0.25  

        
def convert_to_maze_cords(cord):

    
    new_cord = (int((cord[0] - origin[0])/node_dist), int((cord[1] - origin[1])/node_dist))

    print(cord,new_cord)
    return new_cord

def convert_path(path):
    new_path = []

    for i in path:
        new_path.append((origin[0] + (node_dist*i[0]), origin[1] + (node_dist * i[1])))

    new_path.pop(0)

    return new_path

def populate_map_nodes():
    #cords of map nodes
    maze = []

    for _ in range(12):
        row = []
        for _ in range(8):
            row.append(0)
        
        maze.append(row)
    
    return maze

def collision_avoidance_routine(in_range_robots, lrobotPath):
    #if robot direction is forward, it comes across another robot going into the same spot and traveling in opposite direction
    #lower priority robot will move left or right direction if it is empty
    #once other robot has passed move back and continue path

    #find highest prio
    highest_prio = None
    for i in in_range_robots.keys():
        if highest_prio == None:
            highest_prio = i
        elif i > highest_prio:
            highest_prio = i

    #highest priority routine
    if robot_priority > highest_prio:
        robots_in_range_avoiding_coll = []

        #add paths of all collision avoding robots to list
        for v in in_range_robots.values():
            this_state = v[0]

            if this_state == 2:
                robots_in_range_avoiding_coll.append(v[1])
        
        #compare this robot's next two nodes to every in range collision avoiding robot's next node
        another_currently_avoiding_collision = False
        for path in robots_in_range_avoiding_coll:
            if len(lrobotPath) >=2 and (((lrobotPath[0][0], lrobotPath[0][1]) == path[0]) or ((lrobotPath[1][0], lrobotPath[1][1]) == path[0])):
                #pause
                another_currently_avoiding_collision = True 

            elif ((lrobotPath[0][0], lrobotPath[0][1]) == path[0]):
                #pause
                another_currently_avoiding_collision = True
        
        if another_currently_avoiding_collision:
            epuck_move.pause()
        else:
            #update state and follow path as normal
            return lrobotPath



    
    #lower than highest priortiy routine
    if robot_priority < highest_prio:

        # calculate new path based on surrounding robots paths
        
        blacklisted_nodes = []
        # loop thru in range robots
        for i in in_range_robots.keys():

            # 0 is state , 1 is path
            vals = in_range_robots[i]

            path = vals[1]
            state = vals[0]

            for coord in path:
                new_coord = convert_to_maze_cords(coord)
                blacklisted_nodes.append(new_coord)
            
        # generate new maze
        maze = populate_map_nodes()

        print("start", lrobotPath[0])

        new_start = convert_to_maze_cords(lrobotPath[0])
        new_end = convert_to_maze_cords(lrobotPath[-1])




        for blns in blacklisted_nodes:
            if blns != new_end and blns != new_start:

                maze[int(blns[0])][int(blns[1])] = 1

        #print(maze)

        #print(lrobotPath)


        # calculate new path
        new_path = search(maze,1,new_start,new_end)

        new_path = convert_path(new_path)
        
        
        return new_path
    
    return lrobotPath





def check_robots_in_range():
    
    if reciever.getQueueLength() == 0:
        return None

    robots_in_range = {}
    while(reciever.getQueueLength() != 0):
        m = reciever.getData()
        reciever.nextPacket()
        m=m.decode("utf-8").split('/')
        robots_in_range[int(m[0].strip())] = (int(m[1].strip()), list(ast.literal_eval(m[2].strip())))

    return robots_in_range


# takes as input the sampling period
reciever.enable(1)

m = " " # define recieving message

start_pause = 0.0
elapsed_pause = 0.0

in_range_timer = 0.0
current_node_done = True

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
    transmitMessage(str(robot_priority) +"/" + str(state) + "/" + str(robotPath)) 

    # if queue length is not equal to 0, we have received a message
    """if reciever.getQueueLength() != 0:

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


    if reciever.getQueueLength() != 0 and state !=3 and in_range_timer < 3:
        # get the data (in bytes)
        m = reciever.getData()

        # decode to string
        m=m.decode("utf-8")
        
        #print(m.split('/')[1])
        
        

        robots_in_range[int(m.split('/')[0].strip())] = list(ast.literal_eval(m.split('/')[2].strip()))"""

        

    robots_in_range = check_robots_in_range()
            
    if robots_in_range != None and state != 2 and len(robotPath)>0:
        
        epuck_move.reset()
        robotPath = collision_avoidance_routine(robots_in_range,robotPath)

        print("state:" + str(state) +" prio: " + str(robot_priority) + "moving to " + str(robotPath))


    
    if len(robotPath)>0 and path_set:
        
        
        #print("state:" + str(state) +" prio: " + str(robot_priority) + "moving to " + str([robotPath[0][0], robotPath[0][1]]))
        done = epuck_move.moveToDestination([robotPath[0][0], robotPath[0][1]])
        

        if done:
            robotPath.pop(0)
            if state == 2:
                update_state(0)

    if path_set and len(robotPath) == 0:
        robot.getSelf().getField("customData").setSFString(str("done"))

        

    
    
    
    
    

# Enter here exit cleanup code.
