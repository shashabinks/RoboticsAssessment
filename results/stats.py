import json
from unittest import result



with open('Adv_algo/results_epucks6_basic.json', 'r') as f:
    data = json.load(f)


with open('basic_algo/results_epucks6_basic.json', 'r') as f:
    data1 = json.load(f)


adv_coll_count = 0
adv_path_count = 0
basic_coll_count = 0
basic_path_count = 0

for iteration in data['results']:
    adv_coll_count+= iteration['collision']
    adv_path_count+= iteration['paths completed']


for iteration in data1['results']:
    basic_coll_count+= iteration['collision']
    basic_path_count+= iteration['paths completed']


print(f"adv algo coll: {adv_coll_count}" + f" path completed %: {((adv_path_count/100)/6) * 100}%")
print(f"basic algo coll: {basic_coll_count}" + f" path completed %: {((basic_path_count/100)/6) * 100}%")
