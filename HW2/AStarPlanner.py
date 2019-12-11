import sys
import time
import numpy
from collections import namedtuple, defaultdict

class AStarPlanner(object):
    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.nodes = dict()
        self.INF = 10**6

    def Plan(self, start_config, goal_config):
        def _num_add(x,y):
            return tuple(numpy.array(x) + numpy.array(y))

        def _get_h(cord):
            return self.planning_env.compute_heuristic((cord, tuple(goal_config)))

        def build_plan(node, parent_dict):
            plan = [node]
            while True:
                parent = parent_dict[node]
                if parent:
                    plan.insert(0, parent_dict[node])
                    node = parent_dict[node]
                else:
                    break
            return plan

        def get_neighbors_and_distances(node):
            moves = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))
            neighbors = [_num_add(node, m) for m in moves]
            neighbors = [n for n in neighbors if self.planning_env.state_validity_checker(n)] # filter out non valid nodes
            distances = [numpy.linalg.norm((numpy.array(node) - numpy.array(n))) for n in neighbors]
            return neighbors, distances

        open_list = []
        parent_dict = {}
        gscore_dict = defaultdict(lambda: self.INF)
        fscore_dict = defaultdict(lambda: self.INF)

        s0 = tuple(start_config)
        sg = tuple(goal_config)

        open_list.append(s0)
        gscore_dict[s0] = 0
        parent_dict[s0] = None
        fscore_dict[s0] = _get_h(s0) + gscore_dict[s0]

        while open_list:
            # find the node that minimizes f and pop it
            f_list = [fscore_dict[node] for node in open_list]
            print(list(zip(open_list, f_list)))
            idx = numpy.argmin(f_list)
            current = open_list.pop(idx)
            # goal check
            if current == sg:
                return numpy.array(build_plan(current, parent_dict))
            neighbors, distances = get_neighbors_and_distances(current)
            for child, cost in zip(neighbors, distances):
                if (gscore_dict[current] + cost) < gscore_dict[child]:
                    gscore_dict[child] = gscore_dict[current] + cost
                    parent_dict[child] = current
                    fscore_dict[child] = _get_h(child) + gscore_dict[child]
                    if not(child in open_list):
                        open_list.append(child)
        return numpy.array([])

    def ShortenPath(self, path):

        # TODO (student): Postprocess the planner.
        return path
