import queue as q

import numpy as np
from matplotlib import pyplot as plt


def rangelen(l):
    return range(len(l))


class Node:
    g = {}
    parent = {}
    generated_nodes_cords = set()

    def __init__(self, cord, parent=None):
        self.cord = tuple(cord)
        self.x, self.y = cord[0], cord[1]
        self.generated_nodes_cords.add(self.cord)
        self.parent = parent

    def reset_class(self):
        self.g = {}
        parent = {}
        generated_nodes_cords = set()

    def __eq__(self, other):
        return self.cord == other.cord

    def __hash__(self):
        return hash(self.cord)

    def __repr__(self):
        return f"Node {self.cord}"


class OpenList:
    def __init__(self, key):
        self.list = {}
        self.key = key
        self.min_value_item = (None, np.inf)

    def add(self, item):
        item_to_add = (item, self.key(item))
        if self.list.get(item, np.inf) > self.key(item): 
            self.list[item] = self.key(item)
        if item_to_add[1] < self.min_value_item[1]:
            self.min_value_item = item_to_add

    def top(self):
        if self.list:
            # min_idx = np.argmin([item[1] for item in self.list])
            # return self.list[min_idx][0]
            return self.min_value_item[0]
        else:
            return None

    def get_min_key_value(self):
        if self.list:
            # min_idx = np.argmin([item[1] for item in self.list])
            # return self.list[min_idx][1]
            return self.min_value_item[1]
        else:
            return np.inf

    def _update_min_value_item(self):
        if self.list:
            self.min_value_item = min(list(self.list.items()), key=lambda n:n[1])
        else:
            self.min_value_item = (None, np.inf)

    def remove_item(self, item_to_remove):
        try:
            self.list.pop(item_to_remove)
            self._update_min_value_item()
        except:
            self._update_min_value_item()
            pass
        # idx = 0
        # while idx < len(self.list):
        #     if self.list[idx][0] == item_to_remove:
        #         self.list.pop(idx)
        #         continue
        #     else:
        #         idx += 1
        # self._update_min_value_item()

    def __repr__(self):
        return f"OpenList {self.list}"


class MultiHeuristicPlanner(object):
    def __init__(self, planning_env, guidance, w1=20, w2=1.5):
        """

        :param planning_env: The planning environment for the algorithm
        :param guidance: a list of tuples containing the user guidance points
        :param w1: inflation parameter of individual searches
        :param w2: The factor of comparison between anchor heuristic and inadmissible heuristics 
        """

        # self.guidance = guidance[0]
        self.guidance = (400,120) #ugly hack for the experiments
        self.planning_env = planning_env
        self.w1 = w1
        self.w2 = w2

        '''
        This function assume the input start_config and goal_config matches the start and goal
        in the environment object.
        '''

    def Plan(self, start_config, goal_config):
        """"""
        """
        def init():
            initialize goal node
            initialize start node
            initialize euclidean open list (insert start node to list)
            initialize inadmissible open list (insert start node to list)
            initialize euclidean closed list (insert start node to list)
            initialize inadmissible closed list (insert start node to list)

        def key(s,i):
            if i == 0:
                return g(s) + w1 * h_euc(s)
            if i == 1:
                return g(s) + w1 * h_inad(s)

        def expand_state(s):
            remove s from euc_open, inad_open
            for all s' ∈ SUCC(s) do:
                if s' was never generated then:
                    g(s') := ∞
                    bp(s') := NULL
                if g(s') > g(s) + cost(s,s') then:
                    g(s') := g(s) + cost(s,s')
                    bp(s') := s
                    if not(s' ∈ euc_closed) then:
                        euc_open.add(s' with key = key(s', i=0))
                        if not(s' ∈ inad_closed) then:
                            if key(s', i=1) < w_2 * key(s', i=0) do:
                                inad_open.insert(s' with key = key(s', i=1))



        init()
        while euc_open_list:
            if euc_open.min_key() < inad_open.min_key() * w2 then:
                if g(s_goal) ≤ inad_open.min_key() ≤ ∞  then:
                    terminate and return path pointed by bp(s_goal)
                else:
                    s ← inad_open.top()
                    expand_state(s)
                    insert s to inad_closed
            else
                if g(s_goal) ≤ euc_open.min_key() ≤ ∞ then
                    terminate and return path pointed by bp(sgoal)
                else
                    s ← euc_open.top()
                    expand_state(s)
                    insert s to euc_closed
        """

        def compute_distance(start_config, end_config):
            s, g = np.array(start_config), np.array(end_config)
            return np.linalg.norm(g - s)

        def cost(x: Node, y: Node):
            return compute_distance(x.cord, y.cord)

        def euc_key(s):
            return g[s] + self.w1 * get_euc_h(s)

        def inadmissible_key(s):
            return g[s] + self.w1 * get_inadmissible_h(s)

        def get_euc_h(s: Node):
            return compute_distance(s.cord, goal_config)

        def get_path(s: Node):
            path = []
            parent = bp.get(s, None)
            while parent:
                path.append(parent)
                parent = bp.get(parent, None)
            path = [item.cord for item in path]
            return path

        def get_inadmissible_h(s: Node):
            g_is_ancestor = self.guidance in get_path(s)
            if g_is_ancestor:
                return compute_distance(s.cord, goal_config)
            else:
                return compute_distance(goal_config, self.guidance) + compute_distance(s.cord, self.guidance)

        def succ(s: Node):
            parent_cords = s.cord
            moves = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))
            neighbors = [tuple(np.array(parent_cords) + np.array(m)) for m in moves]
            neighbors = [n for n in neighbors if
                         self.planning_env.state_validity_checker(n)]
            # neighbors = [Node(cord, parent=s) for cord in neighbors]
            return neighbors

        def expand(s: Node):
            expanded_nodes.append(s)
            if s.cord == self.guidance:
                pass
            # remove s from all open lists
            euc_open_list.remove_item(s)
            inadmissible_open_list.remove_item(s)
            # explore childs of s
            for child_cord in succ(s):
                if child_cord not in Node.generated_nodes_cords:  # initiate newly generated child
                    child_node = Node(child_cord)
                    g[child_node] = np.inf
                    bp[child_node] = None
                else:
                    child_node = Node(child_cord)
                # if cost to reach child is better than known before:
                # - update g[] and bp[]
                # - add to euc open list (if child wasn't in euc closed list
                # - add to inadmissible open lists
                if g.get(child_node, np.inf) > g[s] + cost(s, child_node):
                    g[child_node] = g[s] + cost(s, child_node)
                    bp[child_node] = s
                    if child_node not in euc_closed_list:
                        euc_open_list.add(child_node)
                        if child_node not in inadmissible_closed_list:
                            if inadmissible_key(child_node) < self.w2 * euc_key(child_node):
                                inadmissible_open_list.add(child_node)
                else:
                    pass

        goal = Node(goal_config)
        start = Node(start_config)
        g = Node.g  # maps node => lowest cost to get to it
        bp = Node.parent  # maps node => parent
        g[start] = 0
        bp[start] = None
        euc_open_list = OpenList(key=euc_key)
        euc_closed_list = []
        inadmissible_open_list = OpenList(key=inadmissible_key)
        inadmissible_closed_list = []

        euc_open_list.add(start)
        euc_closed_list.append(start)
        inadmissible_open_list.add(start)
        inadmissible_closed_list.append(start)

        count_iterations = 0
        expanded_nodes = []
        while euc_open_list.list:
            count_iterations += 1
            if count_iterations%100 == 0:
                print(f"search status: admissible open list: {len(euc_open_list.list)}, inadmissible open list: {len(inadmissible_open_list.list)}")
            if inadmissible_open_list.get_min_key_value() < \
                    self.w2 * euc_open_list.get_min_key_value():
                if g.get(goal, np.inf) < inadmissible_open_list.get_min_key_value():
                    if g.get(goal, np.inf) < np.inf:
                        return np.array(get_path(goal)), expanded_nodes
                else:
                    s = inadmissible_open_list.top()
                    expand(s)
                    inadmissible_closed_list.append(s)
            else:
                if g.get(goal, np.inf) < euc_open_list.get_min_key_value():
                    if g.get(goal, np.inf) < np.inf:
                        return np.array(get_path(goal)), expanded_nodes
                else:
                    s = euc_open_list.top()
                    expand(s)
                    euc_closed_list.append(s)

        # plan.append(start_config)
        # plan.append(goal_config)

        return np.array(plan), expanded_nodes
