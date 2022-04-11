import random
import math


max_x = 11
max_y = 7



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
        while end_cord in used_nodes or dist < node_dist*5:
            end_cord = (random.randint(0, max_x), random.randint(0, max_y))
            dist = math.sqrt((end_cord[0]-start_cord[0])**2 + (end_cord[1]-start_cord[1])**2)

        used_nodes.append(end_cord) 


        generated_points[start_cord[0], start_cord[1]] = (end_cord[0], end_cord[1])

    return generated_points

print(generate_random_start_end(5))