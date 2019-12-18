import numpy
from HW2.RRTTree import RRTTree
from matplotlib import pyplot as plt



class RRTStarPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

    def Plan(self, start_config, goal_config, epsilon=0.001):
        # run parameters
        seen = []
        seen.append(start_config)
        bais = 0.2  # TODO  bias = 0.05, 0.2
        epsilon = 10  # TODO: E1, E2 # 0 = extend all the way
        k = 'log(n)' #TODO k = 3,5, 'log(n)'
        verbose = False
        # Initialize an empty plan.
        plan = []
        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)
        # initialize a dict of distances of vertices from the start
        root_dist = {0: 0}
        plt.plot(start_config[1], start_config[0], 'o', color='b')

        # get samples until goal is found
        new_vertex = []
        while new_vertex != goal_config:
            if k == 'log(n)':
                k = int(round(numpy.log(len(self.tree.vertices))))
                if k < 1:
                    k = 'log(n)'
            rand_vertex = self.get_sample(seen, bais, goal_config)
            nearest_vertex_id, nearest_vertex = self.tree.GetNearestVertex(rand_vertex)
            new_vertex = self.extend(nearest_vertex, rand_vertex, epsilon)
            if self.collision_free(nearest_vertex, new_vertex):
                seen.append(new_vertex)
                plt.plot(new_vertex[1], new_vertex[0], 'o', color='b')
                new_vertex_id = self.tree.AddVertex(new_vertex)
                root_dist[new_vertex_id] = self.planning_env.compute_distance(start_config, new_vertex)
                self.tree.AddEdge(nearest_vertex_id, new_vertex_id)
                if verbose:
                    print(f'New sample added: ({new_vertex[0]},{new_vertex[1]})')

                # Rewire
                if k == 'log(n)':
                    continue
                elif len(seen) > k:
                    nearest_vertices_id, nearest_vertices = self.tree.GetKNN(rand_vertex, k)
                    # sort the neighbours by distance to the root node
                    sorted_by_dist_neighbours_ids = [x[1] for x in
                                                     sorted([(root_dist[x], x) for x in nearest_vertices_id])]
                    for neighbour_id in sorted_by_dist_neighbours_ids:
                        neighbour_coord = self.tree.vertices[neighbour_id]
                        if self.collision_free(neighbour_coord, new_vertex):
                            distance = self.planning_env.compute_distance(neighbour_coord, new_vertex)
                            if self.get_plan_cost(neighbour_id) + distance < self.get_plan_cost(new_vertex_id):
                                self.tree.AddEdge(neighbour_id, new_vertex_id)
                                break  # TODO not sure about this break
                                # oren said that by checking the neighbours in a sorted fashion
                                # we can reduce the number of neighbours to check
            else:
                new_vertex = []
        goal_vertex_id = new_vertex_id

        # get the plan from goal to start
        plan.append(goal_config)
        plt.plot(goal_config[1], goal_config[0], 'o', color='b')
        parent_vertex_id = self.tree.edges[goal_vertex_id]
        plan_cost = abs(self.planning_env.compute_distance(self.tree.vertices[goal_vertex_id],
                                                           self.tree.vertices[parent_vertex_id]))
        while parent_vertex_id:
            plan.append(self.tree.vertices[parent_vertex_id])
            temp = parent_vertex_id
            parent_vertex_id = self.tree.edges[parent_vertex_id]
            plan_cost += abs(self.planning_env.compute_distance(self.tree.vertices[temp],
                                                                self.tree.vertices[parent_vertex_id]))
        plan.append(start_config)
        print(plan_cost, end=', ')
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
                sample_x_coord = numpy.random.random_integers(1, self.planning_env.xlimit[1]) - 1
                sample_y_coord = numpy.random.random_integers(1, self.planning_env.ylimit[1]) - 1
                new_sample = [sample_x_coord, sample_y_coord]
                if new_sample not in seen:
                    break
        return new_sample

    def collision_free(self, near, new):
        # create a line between the coords and check which coords are in between
        line = set(zip([int(x) for x in numpy.linspace(near[0] + 0.5, new[0] + 0.5, 1000)],
                       [int(x) for x in numpy.linspace(near[1] + 0.5, new[1] + 0.5, 1000)]))
        line.add(tuple(new))
        # check if any of the coords along th line are obstacles
        if [1 for x, y in line if self.planning_env.map[x][y]]:
            return False
        else:
            return True


    def ShortenPath(self, path):
        # TODO (student): Postprocessing of the plan.
        return path

    def get_plan_cost(self, vid):
        path_cost = 0
        while vid != 0:
            child_coord = self.tree.vertices[vid]
            vid = self.tree.edges[vid]
            parent_coord = self.tree.vertices[vid]
            path_cost += self.planning_env.compute_distance(parent_coord, child_coord)
        return path_cost
