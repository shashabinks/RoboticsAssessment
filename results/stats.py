import json
from unittest import result





def collect_results(metric, path, num_epucks):
    with open(path, 'r') as f:
        data = json.load(f)


    metric_count = 0

    for iteration in data['results']:
        if iteration['iteration'] > 500:
            break
        metric_count+= iteration[metric]
        
    if metric == "collision":
        print(metric_count)

    if metric == "paths completed":
        print(f" path completed:{((metric_count/500)/num_epucks) * 100} %")

    if metric == "average path completed time":
        print(f" average time: {metric_count/500}")


path = "../final_results/8x4_map_size_basic_algo/results_epucks6.json"


collect_results("paths completed", path, 6)




