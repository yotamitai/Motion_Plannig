import numpy
from HW2.RRTTree import RRTTree


class RRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

    def Plan(self, start_config, goal_config, epsilon=0.001):

        # Initialize an empty plan.
        plan = []

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)

        # get list of all samples (shuffle)

        seen = set()
        bais = 0.05  # TODO  bias = 0.05, 0.2

        # get samples until goal is found
        new_vertex = self.get_sample(seen, bais, goal_config)
        while new_vertex:
            nearest_vertex_id, nearest_vertex = self.tree.GetNearestVertex(new_vertex)
            new_vertex_id = self.tree.AddVertex(new_vertex)
            self.tree.AddEdge(nearest_vertex_id, new_vertex_id)
            if new_vertex == goal_config:
                goal_vertex_id = new_vertex_id
                break
            print(f'New sample added: ({new_vertex[0]},{new_vertex[1]})')
            new_vertex = self.get_sample(seen, bais, goal_config)

        # get the plan from goal to start
        plan.append(goal_config)
        parent_vertex_id = self.tree.edges[goal_vertex_id]
        while parent_vertex_id:
            plan.append(self.tree.vertices[parent_vertex_id])
            parent_vertex_id = self.tree.edges[parent_vertex_id]
        plan.append(start_config)

        return numpy.array(plan[::-1])

    def extend(self):
        # TODO (student): Implement an extend logic.
        pass

    def ShortenPath(self, path):
        # TODO (student): Postprocessing of the plan.
        return path

    def get_sample(self, seen, bais, goal_config):
        # goal_bias
        if numpy.random.sample() <= bais:
            new_sample = goal_config
        else:
            while True:
                sample_x_coord = numpy.random.random_integers(1, self.planning_env.xlimit[1])
                sample_y_coord = numpy.random.random_integers(1, self.planning_env.ylimit[1])
                if (sample_x_coord, sample_y_coord) not in seen:
                    new_sample = [sample_x_coord, sample_y_coord]
                    seen.add(tuple(new_sample))
                    break

        return new_sample
