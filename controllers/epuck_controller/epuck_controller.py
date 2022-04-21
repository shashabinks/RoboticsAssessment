"""epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
#from numpy import double

from tracemalloc import start
import black

from numpy import reciprocal
from webots.controller import Robot, Motor, Receiver, Supervisor,Emitter
from epuck_move_to_destination import Epuck_move

from a_star import search
import asyncio

import struct
import ast
import math
from threading import Thread, Timer
from time import sleep

#origin cordinates
origin = (-1.4, -0.875)

#distance between each square
node_dist = 0.25  

robot = Supervisor()
epuck_move = Epuck_move(robot, 1, 5.6)
emitter = Emitter("emitter") 
reciever = Receiver("receiver")

reciever.enable(1)        
emitter.setRange(0.3)


#helper functions
def convert_to_maze_cords(cord):

    new_cord = (int((cord[0] - origin[0])/node_dist), int((cord[1] - origin[1])/node_dist))

    return new_cord

def convert_path(path, pop = True):
    new_path = []

    for i in path:
        new_path.append((origin[0] + (node_dist*i[0]), origin[1] + (node_dist * i[1])))

    if pop:
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

#main class
class Epuck_controller:
    def __init__(self):
        self.emitter = None
        self.reciever = None
        self.robot = None 
        self.epuck_move = None
        self.robot_priority = 0
        self.state = 0
        self.end_point = None
        self.robotPath = []
        self.path_set = False
        self.already_avoided_robots = []
    

    def check_robots_in_range(self):
    
        if self.reciever.getQueueLength() == 0:
            return None

        robots_in_range = {}
        while(self.reciever.getQueueLength() != 0):
            m = self.reciever.getData()
            self.reciever.nextPacket()
            m=m.decode("utf-8").split('/')
            robots_in_range[int(m[0].strip())] = (int(m[1].strip()), list(ast.literal_eval(m[2].strip())))

        return robots_in_range

    def transmit_status_in_radius(self):
        #print("TRANSMITTING")

        # convert to bytes
        b = bytes(str(self.robot_priority) +"/" + str(self.state) + "/" + str(self.robotPath), 'utf-8')

        self.emitter.send(b)

        #print("TRANSMITTED")
        pass
    
    def collision_avoidance_routine(self, in_range_robots, robotPath, end_point):
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
        if self.robot_priority > highest_prio:
            robots_in_range_avoiding_coll = []

            #add paths of all collision avoding robots to list
            another_currently_avoiding_collision = False
            for k in in_range_robots.keys():
                this_state = in_range_robots[k][0]
                #print(this_state)
                if this_state == 2:
                    robots_in_range_avoiding_coll.append(k)
            
            
            #compare this robot's next two nodes to every in range collision avoiding robot's next node
            
            next_node = (robotPath[0][0], robotPath[0][1]) 
            for k in robots_in_range_avoiding_coll:
                path = in_range_robots[k][1]
                location = self.robot.getFromDef(f"EPUCK{i}").getField("translation").getSFVec3f()

                if len(robotPath) >=2 and ((convert_to_maze_cords(next_node) == convert_to_maze_cords(path[0])) or (convert_to_maze_cords(next_node) == convert_to_maze_cords(location))):
                    another_currently_avoiding_collision = True
                    

            
            if another_currently_avoiding_collision:

                self.epuck_move.reset()
                self.state = 1
                self.epuck_move.state = 1
                print(str(self.robot.getTime()) + "epucks currently avoiding collisions" + str(len(in_range_robots)))
            else:
                #update state and follow path as normal
                self.state = 0
                self.epuck_move.state = 0
                print(str(self.robot.getTime()) + "no epucks")
                return robotPath



        
        #lower than highest priortiy routine
        if self.robot_priority < highest_prio:

            #loop through all robots
            #add their paths and their robot location to blacklist, remove my end point
            # recalculate path with current location and fixed end point
            # figure out case where higher prio epuch is in my end point

            black_list = []

            for i in in_range_robots.keys():

                if i not in self.already_avoided_robots:

                    this_path = in_range_robots[i][1]

                    for node in this_path:
                        black_list.append(node)
                    
                    epuck_ref = self.robot.getFromDef(f"EPUCK{i}")

                    location = epuck_ref.getField("translation").getSFVec3f()

                    black_list.append(location)
                    self.already_avoided_robots.append(i)
            
            if len(black_list) == 0:
                print("low prio, no new bots")
                return robotPath

            maze = populate_map_nodes()
            
            for b_node in black_list:
                if b_node != end_point:
                    
                    converted_b_node = convert_to_maze_cords(b_node)
                    maze[converted_b_node[0]][converted_b_node[1]] = -1

            my_location = self.robot.getSelf().getField("translation").getSFVec3f()


            
            new_path = search(maze, 1, convert_to_maze_cords(my_location), convert_to_maze_cords(end_point))

            print("lower prio path: " + str(new_path))

            
            self.state = 2
            thread = Thread(target=self.timer_function, args=(5.0,))
            thread.start()
            self.epuck_move.reset()
            return convert_path(new_path)
        
        return robotPath


    def update_state(self, new_state):
        self.state = new_state
        self.epuck_move.state = new_state

    def check_for_other_robots(self):
        robots_in_range = self.check_robots_in_range()
    
        if robots_in_range != None and self.state != 2 and len(self.robotPath)>0:
            
            
            self.robotPath = self.collision_avoidance_routine(robots_in_range, self.robotPath, self.end_point)

            #print("state:" + str(state) +" prio: " + str(robot_priority) + "moving to " + str(robotPath))
        
        elif self.state == 1:
            self.state = 0
            self.epuck_move.state = 0
            self.epuck_move.reset()

    def follow_path(self):
        if len(self.robotPath)>0 and self.path_set:
            
            done = False

            if self.state != 1:
                
                done = self.epuck_move.moveToDestination([self.robotPath[0][0], self.robotPath[0][1]])

            if done:
                self.robotPath.pop(0)

        if self.path_set and len(self.robotPath) == 0:
            self.robot.getSelf().getField("customData").setSFString(str("done"))

    def timer_function(self, secs):
        sleep(secs)
        self.update_state(0)
    
        



epuck_controller = Epuck_controller()
epuck_controller.robot = robot
epuck_controller.epuck_move = epuck_move
epuck_controller.reciever = reciever
epuck_controller.emitter = emitter
# get the time step of the current world.
timestep = int(epuck_controller.robot.getBasicTimeStep())

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while epuck_controller.robot.step(timestep) != -1:
   

    if len(epuck_controller.robotPath) == 0 and not epuck_controller.path_set:
        data = epuck_controller.robot.getCustomData()
        if data != '':
            print("recieved this data :")
            print(data)
            epuck_controller.robotPath = list(ast.literal_eval(ast.literal_eval(data)[1]))
            epuck_controller.robot_priority  = int(ast.literal_eval(data)[0])
            epuck_controller.path_set = True
            epuck_controller.end_point = epuck_controller.robotPath[-1]
        
    

    # transmit priority
    epuck_controller.transmit_status_in_radius() 

    epuck_controller.check_for_other_robots()

    epuck_controller.follow_path()

    
    

        

    
    
    
    
    

# Enter here exit cleanup code.
