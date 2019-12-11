import numpy
from HW2.RRTTree import RRTTree


class RRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

    def Plan(self, start_config, goal_config, epsilon=0.001):
        # run parameters
        seen = []
        seen.append(start_config)
        bais = 0.2  # TODO  bias = 0.05, 0.2
        epsilon = 20  # TODO: E1, E2
        # Initialize an empty plan.
        plan = []
        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)

        # get samples until goal is found
        new_vertex = []
        while new_vertex != goal_config:
            rand_vertex = self.get_sample(seen, bais, goal_config)
            nearest_vertex_id, nearest_vertex = self.tree.GetNearestVertex(rand_vertex)
            new_vertex = self.extend(nearest_vertex, rand_vertex, epsilon)
            if self.collision_free(nearest_vertex, new_vertex):
                seen.append(new_vertex)
                new_vertex_id = self.tree.AddVertex(new_vertex)
                self.tree.AddEdge(nearest_vertex_id, new_vertex_id)
                print(f'New sample added: ({new_vertex[0]},{new_vertex[1]})')
            else:
                new_vertex = []
        goal_vertex_id = new_vertex_id

        # get the plan from goal to start
        plan.append(goal_config)
        parent_vertex_id = self.tree.edges[goal_vertex_id]
        while parent_vertex_id:
            plan.append(self.tree.vertices[parent_vertex_id])
            parent_vertex_id = self.tree.edges[parent_vertex_id]
        plan.append(start_config)
        return numpy.array(plan[::-1])

    def extend(self, near, new, step_size):
        # Done: Implement an extend logic.
        if step_size:
            norm = self.planning_env.compute_distance(near, new)
            direction = [(new[0] - near[0]) / norm, (new[1] - near[1]) / norm]
            step = [int(round(near[0] + direction[0] * step_size)),
                    int(round(near[1] + direction[1] * step_size))]
            if round(self.planning_env.compute_distance(near, step)) < round(norm):
                return step
        return new

    def get_sample(self, seen, bais, goal_config):
        # goal_bias
        if numpy.random.sample() <= bais:
            new_sample = goal_config
        else:
            while True:
                # TODO: fix for weird map setup
                sample_x_coord = numpy.random.random_integers(1, self.planning_env.xlimit[1]) - 1
                sample_y_coord = numpy.random.random_integers(1, self.planning_env.ylimit[1]) - 1
                new_sample = [sample_x_coord, sample_y_coord]
                if new_sample not in seen:
                    break
        return new_sample

    def collision_free(self, near, new):
        # create a line between the coords and check which coords are in between
        line = set(zip([int(x) for x in numpy.linspace(near[0], new[0], 1000)],
                       [int(x) for x in numpy.linspace(near[1], new[1], 1000)]))
        # check if any of the coords along th line are obstacles
        if [1 for x, y in line if self.planning_env.map[x][y]]:
            return False
        else:
            return True
