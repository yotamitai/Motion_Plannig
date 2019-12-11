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
    plan = planner.Plan(start, goal)

    # Shortcut the path.
    # TODO (student): Do not shortcut when comparing the performance of algorithms. 
    # Comment this line out when collecting data over performance metrics.
    # plan_short = planner.ShortenPath(plan)

    # Visualize the final path.
    planning_env.visualize_plan(plan)
    embed()


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

    main(planning_env, planner, args.start, args.goal)
