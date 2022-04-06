"""bot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from re import S
from sys import set_coroutine_origin_tracking_depth
from webots.controller import Supervisor, Emitter, Robot
import struct
from a_star import a_star
import random
import math


#number of epucks
num_epucks = 1

#origin cordinates
origin = (-1.4, -0.875)

#distance between each square
node_dist = 0.25

#map size
max_x = 11
max_y = 7

#epuck id : priority
epucks = {}

#cords of map nodes
map_nodes = []

def generate_random_start_end(num_points):
    used_nodes = []

    generated_points = {}

    for i in range(num_points):
        start_cord = (random.randint(0, max_x), random.randint(0, max_y))
        while start_cord in used_nodes:
            start_cord = (random.randint(0, max_x), random.randint(0, max_y))

        used_nodes.append(start_cord)    

        end_cord = (random.randint(0, max_x), random.randint(0, max_y))
        dist = math.sqrt((end_cord[0]-start_cord[0])**2 + (end_cord[1]-start_cord[1])**2)
        #minimum distance of 5 squares between start and end
        while end_cord in used_nodes or dist < node_dist*5 :
            end_cord = (random.randint(0, max_x), random.randint(0, max_y))
            dist = math.sqrt((end_cord[0]-start_cord[0])**2 + (end_cord[1]-start_cord[1])**2)

        used_nodes.append(end_cord) 


        generated_points[(origin[0] + (start_cord[0] * node_dist), origin[1] + (start_cord[1] * node_dist))] = (origin[0] + (end_cord[0] * node_dist), origin[1] + (end_cord[1] * node_dist))

    return generated_points






def populate_map_nodes():
    for x in range(max_x):
        for y in range(max_y):
            cords = (origin[0] + (node_dist * x), origin[1] + (node_dist * y))
            map_nodes.append(cords)




# create the Robot instance
supervisor = Supervisor()

emitter = Emitter(name="robot")


#path = a_star(map_squares, start, end, origin, square_dist)

print("generating end points and map nodes")
#generate random start and end locations
start_end_points = generate_random_start_end(5)
populate_map_nodes()

priorites = [i for i in range(num_epucks)]
print(priorites)
#find epuck references, generate paths, assign priorities
for i in range(num_epucks):
    priority_idx = random.randint(0, len(priorites)-1)
    epucks[i] = priorites[priority_idx]
    priorites.remove(priority_idx)

    #pick random start and end pair
    start_end__pair = random.choice(list(start_end_points))
    start_cord = start_end__pair
    end_cord = start_end_points[start_cord]

    del start_end_points[start_end__pair]
    #move robot to start pos
    epuck_ref = supervisor.getFromDef(f"EPUCK{i}")
    epuck_ref.getField("translation").setSFVec3f([start_cord[0], start_cord[1], 0])

    print(f"running a-star for epuck {i}")
    #generate path using a-star
    path = a_star(map_nodes, start_cord, end_cord, origin, node_dist)
    

    #create custom data array and send to epuck
    data = [str(epucks[i]), str(path)]

    print("data: " + str(data))

    epuck_ref.getField("customData").setSFString(str(data))


# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# 1. ID, 2. instruction 3. optional: array of cords 

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
