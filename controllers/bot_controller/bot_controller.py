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
from plyer import notification

from threading import Thread, Timer
import time


map_choices = {"basic": {"origin" : (-1.4, -0.875), "max_area": (11,7) , "path_size": 5 }, "medium": {"origin" : (-1.4 + 0.5, -0.875), "max_area": (7,7), "path_size": 5 * (2/3) }, "small": {"origin" : (-1.4 + 0.5, -0.875 + 0.5), "max_area": (7,3), "path_size": 5 * (1/3)}}

map_choice = "small"

min_path_size = map_choices[map_choice]["path_size"]

#origin cordinates
origin = map_choices[map_choice]["origin"]

# 8 x 8 origin = (-1.4 + 0.5, -0.875)
# 8 x 4 origin = (-1.4 + 0.5, -0.875 + 0.5)


#distance between each square
node_dist = 0.25

#map size
max_x = map_choices[map_choice]["max_area"][0]
max_y = map_choices[map_choice]["max_area"][1]



def force_restart():
    time.sleep(180)




#helper functions
def populate_map_nodes():
    maze = []
    for _ in range(12):
        row = []
        for _ in range(8):
            row.append(0)
        
        maze.append(row)
    return maze

def convert_path(path, pop=True):
    new_path = []

    for i in path:
        new_path.append((origin[0] + (node_dist*i[0]), origin[1] + (node_dist * i[1])))

    if pop:
        new_path.pop(0)

    return new_path

def covert_to_world_cords(cord):
    new_cord = [origin[0] + (cord[0] * node_dist), origin[1] + (cord[1] * node_dist), 0]
    return new_cord


# create the Robot instance
supervisor = Supervisor()

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())




class Bot_controller:
    def __init__(self, num_epucks, test_type):
        self.num_collisions = 0
        self.collided = {}
        self.supervisor = None
        self.time_out_s = None
        self.iteration = 0
        self.epucks = {}
        self.epucks_intialised = False
        self.maze = populate_map_nodes()
        self.start_end_points = self.generate_random_start_end(num_epucks)
        self.num_epucks = num_epucks
        self.test_type = test_type
    
    def epuck_intialise(self):
        for i in range(0, self.num_epucks):
        
            priority_idx = i
            #print(priority_idx)
            self.epucks[i] = priority_idx
            

            #pick random start and end pair
            start_end__pair = random.choice(list(self.start_end_points))
            start_cord = start_end__pair
            end_cord = self.start_end_points[start_cord]

            del self.start_end_points[start_end__pair]
            #move robot to start pos
            epuck_ref = self.supervisor.getFromDef(f"EPUCK{i}")

            epuck_ref.getField("translation").setSFVec3f([origin[0] + (node_dist * start_cord[0]), origin[1] + (node_dist * start_cord[1]), 0])
            

            print(f"running a-star for epuck {i}")
            #generate path using a-star
            print(start_cord)
            print(end_cord)
            

            path = search(self.maze, 1, start_cord, end_cord)
            
            path = convert_path(path)

        
            #create custom data array and send to epuck
            data = [str(self.epucks[i]), str(path)]

            print("data: " + str(data))

            epuck_ref.getField("customData").setSFString(str(data))

        thread = Thread(target=self.force_restart, args=())
        thread.start()

    def generate_random_start_end(self, num_points):
        used_nodes = []

        generated_points = {}

        for _ in range(num_points):
            start_cord = (random.randint(0, max_x), random.randint(0, max_y))
            while start_cord in used_nodes:
                start_cord = (random.randint(0, max_x), random.randint(0, max_y))

            used_nodes.append(start_cord)    

            end_cord = (random.randint(0, max_x), random.randint(0, max_y))
            dist = math.sqrt((end_cord[0]-start_cord[0])**2 + (end_cord[1]-start_cord[1])**2)
            #minimum distance of 5 squares between start and end
            while end_cord in used_nodes or dist < min_path_size:
                end_cord = (random.randint(0, max_x), random.randint(0, max_y))
                dist = math.sqrt((end_cord[0]-start_cord[0])**2 + (end_cord[1]-start_cord[1])**2)

            used_nodes.append(end_cord) 


            generated_points[start_cord[0], start_cord[1]] = (end_cord[0], end_cord[1])

        return generated_points
        
    def count_collisions(self):
        checked_pairs = []
        for i in range(len(self.epucks)):
            epuck_ref1 = self.supervisor.getFromDef(f"EPUCK{i}")
            for j in range(len(self.epucks)):
                if i != j and (i, j) not in checked_pairs and (j, i) not in checked_pairs:
                    epuck_ref2 = self.supervisor.getFromDef(f"EPUCK{j}")

                    vector1 = epuck_ref1.getField("translation").getSFVec3f()
                    vector2 = epuck_ref2.getField("translation").getSFVec3f()

                    dist = math.sqrt(math.pow((vector2[1] - vector1[1]), 2) + math.pow((vector2[0] - vector1[0]), 2))
                    if(dist < 0.08):
                        if (i, j) not in self.collided:
                            self.num_collisions+=1
                            self.collided[(i, j)] = 3
                            self.collided[(j, i)] = 3
                        elif self.collided[(i, j)] <=0 and self.collided[(j, i)] <=0:
                            self.num_collisions+=1
                            self.collided[(i, j)] = 3
                            self.collided[(j, i)] = 3
                    else:
                        if (i, j) in self.collided:
                            self.collided[(i, j)] = self.collided[(i, j)] - self.supervisor.getBasicTimeStep()
                            self.collided[(j, i)] = self.collided[(i, j)] - self.supervisor.getBasicTimeStep()

                    checked_pairs.append((i, j))
                    checked_pairs.append((j, i))

    def record_results(self):
        if self.supervisor.getTime() + self.time_out_s > 120.0:

            total_time_for_completed = 0
            path_done_counter = 0
            for i in range(len(self.epucks)):
                epuck_data = self.supervisor.getFromDef(f"EPUCK{i}").getField("customData").getSFString()
                
                splitted = str(epuck_data).split('|')
                if splitted[0].strip() == "done":
                    print(f"done in {splitted[1].strip()}")
                    path_done_counter+=1
                    total_time_for_completed+= float(splitted[1].strip())
                
            

            if exists(f'../../results/{self.test_type}/results_epucks{self.num_epucks}.json'):
                with open(f'../../results/{self.test_type}/results_epucks{self.num_epucks}.json', 'r') as f:
                    data = json.load(f)
                

                current_iteration = data['results'][len(data['results'])-1]['iteration'] + 1
                

                result_for_this_iteration = {"time": str(time.ctime()), "iteration":current_iteration, "collision":self.num_collisions, "paths completed":path_done_counter, "average path completed time": round((total_time_for_completed/path_done_counter), 2)}
                

                with open(f'../../results/{self.test_type}/results_epucks{self.num_epucks}.json','r+') as f:
                    
                    data = json.load(f)
                    data['results'].append(result_for_this_iteration)
                    f.seek(0)
                
                    json.dump(data, f, indent = 4)
                
                if current_iteration == 500:
                    notification.notify(
                        title = f'epucks count: {self.num_epucks}, exp: {self.test_type}',
                        message = '500 iterations reached',
                        app_icon = None,
                        timeout = 9999999,
                    )
                    supervisor.simulationQuit()
            
            else:

                result_for_this_iteration = {"time": str(time.ctime()), "iteration": 1, "collision": self.num_collisions, "paths completed":path_done_counter, "average path completed time": round((total_time_for_completed/path_done_counter), 2)}

                with open(f'../../results/{self.test_type}/results_epucks{self.num_epucks}.json','w') as f:
                    output = {}
                    output['results'] = [result_for_this_iteration]
                    json.dump(output, f, indent = 4)

            self.reset()
                
                
    

    def force_restart(self):
        time.sleep(60)
        self.reset()

    def reset(self):
        self.num_collisions = 0

        for i in range(len(self.epucks)):
            epuck_ref1 = self.supervisor.getFromDef(f"EPUCK{i}")
            epuck_ref1.getField("customData").setSFString("")
            epuck_ref1.restartController()

        
        self.supervisor.simulationReset()
        self.supervisor.simulationResetPhysics()
        self.supervisor.getSelf().restartController()

<<<<<<< Updated upstream
bot_controller = Bot_controller(num_epucks=6, test_type="8x4_map_size_basic_algo")
=======
bot_controller = Bot_controller(num_epucks=6, test_type="8x8_map_size_none_algo")
>>>>>>> Stashed changes
bot_controller.supervisor = supervisor

while supervisor.step(timestep) != -1:

    if not bot_controller.epucks_intialised:
        print("intialising epucks")
        bot_controller.epuck_intialise()
        bot_controller.time_out_s = bot_controller.supervisor.getTime()
        bot_controller.epucks_intialised = True

    

    bot_controller.count_collisions()

    bot_controller.record_results()
    
        
        


    

# Enter here exit cleanup code.