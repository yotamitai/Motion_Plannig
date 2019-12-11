import numpy
from HW2.RRTTree import RRTTree


class RRTStarPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

    def Plan(self, start_config, goal_config, epsilon=0.001):
        # run parameters
        seen = []
        seen.append(start_config)
        bais = 0.2  # TODO  bias = 0.05, 0.2
        epsilon = 10  # TODO: E1, E2
        k = 3
        # Initialize an empty plan.
        plan = []
        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)

        # get samples until goal is found
        new_vertex = []
        while new_vertex != goal_config:
            rand_vertex = self.get_sample(seen, bais, goal_config)
            if len(seen) > k:
                nearest_vertices_id, nearest_vertices, nearest_vertices_dist = self.tree.GetKNN(rand_vertex, k)
            else:
                nearest_vertices = seen
                nearest_vertices_id = list(range(len(nearest_vertices)))
                nearest_vertices_dist = {i: self.planning_env.compute_distance(rand_vertex, x)
                                         for i, x in enumerate(self.tree.vertices)}

            sorted_nearest_vertices_dist = sorted([(y, x) for x, y in nearest_vertices_dist.items()])
            new_vertices = {tuple(x): self.extend(x, rand_vertex, epsilon) for x in nearest_vertices}
            best_neighbour_indx = self.collision_free(nearest_vertices, sorted_nearest_vertices_dist, new_vertices)
            if best_neighbour_indx is not False:  # vid can be 0, so need the 'is not False' syntax
                new_vertex = new_vertices[tuple(nearest_vertices[best_neighbour_indx])]
                seen.append(new_vertex)
                new_vertex_id = self.tree.AddVertex(new_vertex)
                self.tree.AddEdge(nearest_vertices_id[best_neighbour_indx], new_vertex_id)
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

    def collision_free(self, knn, knn_dist_dict, new):
        # create a line between the coords and check which coords are in between
        for val, vid in knn_dist_dict:
            line = set(zip([int(x) for x in numpy.linspace(self.tree.vertices[vid][0], new[tuple(knn[vid])][0], 1000)],
                           [int(x) for x in numpy.linspace(self.tree.vertices[vid][1], new[tuple(knn[vid])][1], 1000)]))
            # check if any of the coords along th line are obstacles
            if not [1 for x, y in line if self.planning_env.map[x][y]]:
                return vid
        return False

    def ShortenPath(self, path):
        # TODO (student): Postprocessing of the plan.
        return path
