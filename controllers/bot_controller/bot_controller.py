"""bot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from webots.controller import Supervisor, Emitter, Robot, Node
import struct
from a_star import search
from os.path import exists
import random
import math
import json


#number of epucks
num_epucks = 6

#origin cordinates
origin = (-1.4, -0.875)

#distance between each square
node_dist = 0.25

#map size
max_x = 11
max_y = 7

#epuck id : priority
epucks = {}

epucks_intialised = False

#cords of map nodes
maze = []

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
        while end_cord in used_nodes or dist < 5 :
            end_cord = (random.randint(0, max_x), random.randint(0, max_y))
            dist = math.sqrt((end_cord[0]-start_cord[0])**2 + (end_cord[1]-start_cord[1])**2)

        used_nodes.append(end_cord) 


        generated_points[start_cord[0], start_cord[1]] = (end_cord[0], end_cord[1])

    return generated_points






def populate_map_nodes():
    for _ in range(12):
        row = []
        for _ in range(8):
            row.append(0)
        
        maze.append(row)


def convert_path(path):
    new_path = []

    for i in path:
        new_path.append((origin[0] + (node_dist*i[0]), origin[1] + (node_dist * i[1])))

    new_path.pop(0)

    return new_path


def covert_to_world_cords(cord):
    new_cord = [origin[0] + (cord[0] * node_dist), origin[1] + (cord[1] * node_dist), 0]
    return new_cord
# create the Robot instance
supervisor = Supervisor()


emitter = Emitter(name="robot")



#path = a_star(map_squares, start, end, origin, square_dist)


#generate random start and end locations
populate_map_nodes()



#find epuck references, generate paths, assign priorities



# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())


def epuck_intialise():
    priorites = [i for i in range(num_epucks)]
    

    for i in range(0,num_epucks):
    
        priority_idx = random.choice(priorites)
        #print(priority_idx)
        epucks[i] = priority_idx
        
        
        priorites.remove(priority_idx)

        #pick random start and end pair
        start_end__pair = random.choice(list(start_end_points))
        start_cord = start_end__pair
        end_cord = start_end_points[start_cord]

        del start_end_points[start_end__pair]
        #move robot to start pos
        epuck_ref = supervisor.getFromDef(f"EPUCK{i}")

        

        epuck_ref.getField("translation").setSFVec3f([origin[0] + (node_dist * start_cord[0]), origin[1] + (node_dist * start_cord[1]), 0])

        

        print(f"running a-star for epuck {i}")
        #generate path using a-star
        print(start_cord)
        print(end_cord)
        

        path = search(maze, 1, start_cord, end_cord)
        
        path = convert_path(path)

       
        #create custom data array and send to epuck
        data = [str(epucks[i]), str(path)]

        print("data: " + str(data))

        epuck_ref.getField("customData").setSFString(str(data))


num_collisions = 0

collided = {}

time_out_s = supervisor.getTime()

iteration = 0

while supervisor.step(timestep) != -1:

    
       
    if not epucks_intialised:
        start_end_points = generate_random_start_end(num_epucks)
        print("intialising epucks")
        epuck_intialise()
        
        
        epucks_intialised = True

    
    checked_pairs = []
    for i in range(len(epucks)):
        epuck_ref1 = supervisor.getFromDef(f"EPUCK{i}")
        for j in range(len(epucks)):
            if i != j and (i, j) not in checked_pairs and (j, i) not in checked_pairs:
                epuck_ref2 = supervisor.getFromDef(f"EPUCK{j}")

                vector1 = epuck_ref1.getField("translation").getSFVec3f()
                vector2 = epuck_ref2.getField("translation").getSFVec3f()

                dist = math.sqrt(math.pow((vector2[1] - vector1[1]), 2) + math.pow((vector2[0] - vector1[0]), 2))
                if(dist < 0.08):
                    if (i, j) not in collided:
                        num_collisions+=1
                        collided[(i, j)] = 3
                        collided[(j, i)] = 3
                    elif collided[(i, j)] <=0 and collided[(j, i)] <=0:
                        num_collisions+=1
                        collided[(i, j)] = 3
                        collided[(j, i)] = 3
                else:
                    if (i, j) in collided:
                        collided[(i, j)] = collided[(i, j)] - supervisor.getBasicTimeStep()
                        collided[(j, i)] = collided[(i, j)] - supervisor.getBasicTimeStep()

                checked_pairs.append((i, j))
                checked_pairs.append((j, i))


    if supervisor.getTime() + time_out_s > 360.0:

        path_done_counter = 0
        for i in range(len(epucks)):
            epuck_data = supervisor.getFromDef(f"EPUCK{i}").getField("customData").getSFString()
            
            if epuck_data == "done":
                path_done_counter+=1
            
        

        if exists(f'results_epucks{num_epucks}_basic.json'):
            with open(f'results_epucks{num_epucks}_basic.json', 'r') as f:
                data = json.load(f)
            

            current_iteration = data['results'][len(data['results'])-1]['iteration'] + 1
            

            result_for_this_iteration = {"iteration":current_iteration, "collision":num_collisions, "paths completed":path_done_counter}
            

            with open(f'results_epucks{num_epucks}_basic.json','r+') as f:
                
                data = json.load(f)
                data['results'].append(result_for_this_iteration)
                f.seek(0)
            
                json.dump(data, f, indent = 4)
        
        else:

            result_for_this_iteration = {"iteration": 1, "collision": num_collisions, "paths completed":path_done_counter}

            with open(f'results_epucks{num_epucks}_basic.json','w') as f:
                output = {}
                output['results'] = [result_for_this_iteration]
                json.dump(output, f, indent = 4)

        
            
            
        
        num_collisions = 0

        for i in range(len(epucks)):
            epuck_ref1 = supervisor.getFromDef(f"EPUCK{i}")
            epuck_ref1.restartController()

        
        supervisor.simulationReset()
        supervisor.simulationResetPhysics()
        supervisor.getSelf().restartController()
        
        


    

# Enter here exit cleanup code.