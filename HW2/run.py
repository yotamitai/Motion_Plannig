#!/usr/bin/env python

import argparse, numpy, time

from HW2.MapEnvironment import MapEnvironment
from HW2.RRTPlanner import RRTPlanner
from HW2.RRTStarPlanner import RRTStarPlanner
from HW2.AStarPlanner import AStarPlanner

from IPython import embed

"""
configurations:
-m map1.txt -p rrt -s 1 1 -g 4 5
-m map2.txt -p astar -s 321 148 -g 106 202
"""


def main(planning_env, planner, start, goal):
    # Notify.
    # input('Press any key to begin planning')

    # Plan.
    import time
    t0 = time.time()
    plan = planner.Plan(start, goal)
    t1 = time.time()
    dt = t1 - t0

    # Shortcut the path.
    # TODO (student): Do not shortcut when comparing the performance of algorithms. 
    # Comment this line out when collecting data over performance metrics.
    # plan_short = planner.ShortenPath(plan)

    # Visualize the final path.
    planning_env.visualize_plan(plan)
    # embed()
    return dt


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-m', '--map', type=str, default='map1.txt',
                        help='The environment to plan on')
    parser.add_argument('-p', '--planner', type=str, default='rrt',
                        help='The planner to run (star, rrt, rrtstar)')
    parser.add_argument('-s', '--start', nargs='+', type=int, required=True)
    parser.add_argument('-g', '--goal', nargs='+', type=int, required=True)

    args = parser.parse_args()

    # First setup the environment and the robot.
    planning_env = MapEnvironment(args.map, args.start, args.goal)

    num_tests = 1  # for running averages
    run_time_list = []
    for i in range(num_tests):
        # Next setup the planner
        if args.planner == 'astar':
            planner = AStarPlanner(planning_env)
        elif args.planner == 'rrt':
            planner = RRTPlanner(planning_env)
        elif args.planner == 'rrtstar':
            planner = RRTStarPlanner(planning_env)
        else:
            print('Unknown planner option: %s' % args.planner)
            exit(0)

        run_time_list.append(main(planning_env, planner, args.start, args.goal))
    print('\nRun times: ', end='')
    print(run_time_list)
    print(f'\nRun time avgerage over {num_tests} iterations:{sum(run_time_list)/len(run_time_list)}')
