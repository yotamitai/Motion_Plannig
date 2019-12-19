import sys
import time
import numpy
from typing import Optional
from priority_queue import AstarPriorityQueue
from matplotlib import pyplot as plt

class Node:
    """
    Helper class to hold the nodes of Astar.
    A node keeps track of potentially a parent node.
    A node has its cost (g_score , the cost of the cheapest path from start to it), 
    and the f-score (cost + heuristic to goal).
    """
    def __init__(self, config, 
                 parent_node: Optional['Node'] = None,
                 g_score: float = 0,
                 f_score: Optional[float] = None):
        
        self.config = config
        self.parent_node: Node = parent_node
        self.g_score = g_score
        self.f_score = f_score
        
        if self.parent_node is not None:
            self.g_score += self.parent_node.g_score

    def __lt__(self, other):
        return self.f_score < other.f_score
    
class AStarPlanner(object):    
    def __init__(self, planning_env, epsilon = 1):
        self.planning_env = planning_env
        self.nodes = dict()

        self.epsilon = epsilon #weighted astar
        self.open = AstarPriorityQueue()
        self.close = AstarPriorityQueue()
        
    def Plan(self, start_config, goal_config):
        
        plan = []
        #plan.append(start_config)
        #plan.append(goal_config)

        # TODO (student): Implement your planner here.
        initial_config = Node(tuple(start_config), None, 0)
        initial_config.f_score = self._calc_node_f_score(initial_config)
        self.open.Insert(initial_config, initial_config.f_score)
        
        while not self.open.IsEmpty():
            next_node = self.open.Pop()
            self.close.Insert(next_node,1) # priority has no meaning in close
            self.nodes[tuple(next_node.config)] = next_node
            plan.append(next_node.config)
            if next_node.parent_node is not None:
                plot_x = [next_node.parent_node.config[0], next_node.config[0]]
                plot_y = [next_node.parent_node.config[1], next_node.config[1]]
                plt.plot(plot_y, plot_x, 'r')
                print (len(self.close.elements))   
            if next_node.config == tuple(goal_config):
                break
            
            neighbours = self._get_neighbours(next_node.config)
            for neighbour in neighbours:
                cost = self.planning_env.compute_distance(next_node.config, neighbour)
                successor_node = Node(tuple(neighbour), next_node, cost)
                successor_node.f_score = self._calc_node_f_score(successor_node)
                
                new_g = successor_node.g_score
                if self.open.Contains(successor_node.config):  #the node is already in OPEN
                    already_found_node = self.open.GetByConfig(successor_node.config)
                    if new_g < already_found_node.g_score: # new parent is better
                        already_found_node.g_score = new_g
                        already_found_node.parent_node = successor_node.parent_node
                        already_found_node.f_score = self._calc_node_f_score(already_found_node)
                        
                        #f changed so need to reposition in OPEN
                        self.open.Remove(already_found_node.config) 
                        self.open.Insert(already_found_node, already_found_node.f_score)
                        
                    else: #old path is better - do nothing
                        pass
                else: # state not in OPEN maybe in CLOSED
                    if self.close.Contains(successor_node.config): #this node exists in CLOSED
                        already_found_node = self.close.GetByConfig(successor_node.config)
                        if new_g < already_found_node.g_score: # new parent is better
                            already_found_node.g_score = new_g
                            already_found_node.parent_node = successor_node.parent_node
                            already_found_node.f_score = self._calc_node_f_score(already_found_node)
                            
                            #move old node from CLOSED to OPEN
                            self.close.Remove(already_found_node.config)
                            self.nodes.pop(already_found_node.config)
                            self.open.Insert(already_found_node, already_found_node.f_score)
                        else: #old path is better - do nothing
                            pass
                    else:
                        #this is a new state - create a new node = insert new node to OPEN
                        self.open.Insert(successor_node, successor_node.f_score)
                        
        print('Number of expanded states:',len(self.close.elements))
        
        return numpy.array(plan)

    def ShortenPath(self, path):
        # TODO (student): Postprocess the planner.
        current = self.nodes[tuple(path[-1].tolist())]
        short_path = [current.config]
        while current.config != tuple(path[0].tolist()):
            current = self.nodes[tuple(current.parent_node.config)]
            short_path.append(current.config)
        short_path.reverse()
        return numpy.array(short_path)

    def _calc_node_f_score(self, node: Node):
        return node.g_score + (self.epsilon * self.planning_env.compute_heuristic(node.config))
        
    def _get_neighbours(self, config):
        # 8-connected neighbourhood structure
        rel_candidates = [[-1,-1], [0,-1], [1,-1], [-1, 0], [1, 0], [-1, 1], [0, 1], [1, 1]]
        candidates = [numpy.array(config) + numpy.array(x) for x in rel_candidates]
        
        neighbours = []
        for candidate in candidates:
            if self.planning_env.state_validity_checker(candidate) and\
               self.planning_env.edge_validity_checker(config, candidate):
                   neighbours.append(tuple(candidate.tolist()))
        
        return neighbours