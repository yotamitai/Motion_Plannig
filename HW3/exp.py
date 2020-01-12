#!/usr/bin/env python

import argparse
from HW3.AStarPlanner import AStarPlanner
from IPython import embed
from HW3.MapEnvironment import MapEnvironment
from HW3.MultiHeuristicPlanner import MultiHeuristicPlanner, Node
import time
import numpy as np
import pickle
from matplotlib import pyplot as plt


def compute_distance(start, end):
    s, g = np.array(start), np.array(end)
    return np.linalg.norm(g - s)


def get_plan_cost(plan):
    cost = 0
    for i in range(len(plan) - 1):
        cost += compute_distance(plan[i], plan[i + 1])
    return cost


if __name__ == "__main__":
    maps = ['map1.txt', 'map2.txt', 'map3.txt']
    planners = ['mhastar', 'astar']
    hyper_parameters = {
        'mhastar': {
            'w1': [20, 15, 10, 5, 2],
            'w2': list(round(i, 2) for i in np.linspace(1, 2.5, num=7)),
            'start': (400, 500),
            'goal': (100, 120),
            'guidance': (400, 120)
        },
        'astar': {
            'w1': [1, 5, 10, 15, 20],
        }
    }

    results = []

    params = hyper_parameters['mhastar']
    for my_map in maps:
        for w1 in params['w1']:
            for w2 in params['w2']:
                print(f'Experiment: mhstar, w1: {w1}, w2: {w2}, map: {my_map}')
                planning_env = MapEnvironment(my_map, params['start'], params['goal'])
                planner = MultiHeuristicPlanner(planning_env, params['guidance'], w1, w2)
                t0 = time.time()
                plan, expanded_nodes = planner.Plan(params['start'], params['goal'])
                dt = time.time() - t0
                result = {
                    'plan': 'mhastar',
                    'w1': w1,
                    'w2': w2,
                    'time': dt,
                    # 'resulted_plan': plan,
                    'plan_cost': get_plan_cost(plan),
                    # 'expanded_nodes': expanded_nodes,
                    'num_expanded_nodes': len(expanded_nodes)
                }
                results.append(result)

                Node.g = {}
                Node.parent = {}
                Node.generated_nodes_cords = set()

                print(f'Results: time: {dt}, plan cost: {get_plan_cost(plan)}, expanded nodes: {len(expanded_nodes)}')

                my_map_image = np.loadtxt(my_map)
                plt.imshow(my_map_image, interpolation='nearest')
                for node in expanded_nodes:
                    plt.plot(node.cord[1], node.cord[0], "o", color='b')
                plt.plot(params['start'][1], params['start'][0], 'o', color='r')
                plt.plot(params['goal'][1], params['goal'][0], 'o', color='g')
                for i in range(np.shape(plan)[0] - 1):
                    x = [plan[i, 0], plan[i + 1, 0]]
                    y = [plan[i, 1], plan[i + 1, 1]]
                    plt.plot(y, x, 'k')
                image_name = '_'.join(['mhstar', my_map[:-4], f'w1_{w1}', f'w2_{w2}']) + '.png'
                plt.savefig('./images/' + image_name, bbox_inches='tight')
                plt.close()

        print()

    pickle.dump(results, open("results.p", "wb"))

    # planning_env.visualize_plan(plan)
