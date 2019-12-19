import numpy
from HW2.RRTTree import RRTTree
from matplotlib import pyplot as plt
import time
import operator


class RRTStarPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

    def Plan(self, start_config, goal_config, epsilon=0.001):
        # run parameters
        seen = []
        best_plan = []
        seen.append(start_config)
        bais = 0.2  # TODO  bias = 0.05, 0.2
        epsilon = 10  # TODO: E1, E2 # 0 = extend all the way
        k = 'log(n)'  # TODO k = 8,5, 'log(n)'
        verbose = False
        plan_improvement_flag = False
        normal_run = False
        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)
        # initialize a dict of distances of vertices from the start
        root_dist = {0: 0}
        plt.plot(start_config[1], start_config[0], 'o', color='b')

        # get samples until goal is found
        new_vertex = True
        while new_vertex:
            if k == 'log(n)':
                k = int(round(numpy.log(len(self.tree.vertices))))
                if k < 1:
                    k = 'log(n)'
            rand_vertex = self.get_sample(seen, bais, goal_config, plan_improvement_flag)
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
                self.rewire(k, seen, new_vertex, new_vertex_id, start_config, plan_improvement_flag, best_plan)
            else:
                continue

            if new_vertex == goal_config:
                if normal_run:
                    """regular run"""
                    goal_vertex_id = new_vertex_id
                    plt.plot(goal_config[1], goal_config[0], 'o', color='b')
                    break

                """plan improvment over time"""
                if not plan_improvement_flag:
                    goal_vertex_id = new_vertex_id
                    seen.append(goal_config)
                    plt.plot(goal_config[1], goal_config[0], 'o', color='b')
                    start_time = time.time()
                    plan_improvement_flag = True
                    last_check_time = 0
                    time_sample = 1
                    run_time = 60
                    best_plan, plan_cost = self.get_plan(start_config, goal_config, goal_vertex_id)
                    time_list = [0, ]
                    plan_costs = [plan_cost]
                    continue

            if plan_improvement_flag:
                temp_time = time.time() - start_time
                if temp_time > last_check_time + time_sample:
                    best_plan, plan_cost = self.get_plan(start_config, goal_config, goal_vertex_id)
                    last_check_time = temp_time
                    plan_costs.append(plan_cost)
                    time_list.append(time_list[-1] + time_sample)
                    print(plan_cost)
                if temp_time >= run_time:
                    print(plan_costs)
                    print(time_list)
                    break

        plan, plan_cost = self.get_plan(start_config, goal_config, goal_vertex_id)
        print(plan_cost, end=', ')
        return numpy.array(plan[::-1])

    def rewire(self, k, seen, new_vertex, new_vertex_id, start_config, check_plan, plan):
        if k == 'log(n)':
            return
        elif len(seen) > k+1:
            distances = []
            nearest_vertices_id, nearest_vertices = self.tree.GetKNN(new_vertex, k+1)
            nearest_vertices_id = nearest_vertices_id.tolist()
            nearest_vertices_id.remove(new_vertex_id)
            for neighbour_id in nearest_vertices_id:
                neighbour_coord = self.tree.vertices[neighbour_id]
                if self.collision_free(neighbour_coord, new_vertex):
                    _, distance = self.get_plan(start_config, neighbour_coord, neighbour_id)
                    distances.append([distance + self.planning_env.compute_distance(neighbour_coord, new_vertex),
                                      neighbour_id])
            new_parent_id = sorted(distances)[0][1]

            # if check_plan:
            #     current_parent_coord = self.tree.vertices[self.tree.edges[new_vertex_id]]
            #     if current_parent_coord in plan and self.tree.vertices[new_parent_id] != current_parent_coord:
            #         plan_node_id = self.tree.edges[new_vertex_id]
            #         self.tree.AddEdge(new_vertex_id, plan_node_id)
            #         print()
                    # TODO not working...

            self.tree.AddEdge(new_parent_id, new_vertex_id)



    def get_plan(self, start_config, goal_config, goal_vertex_id):
        if start_config == goal_config:
            return [], 0
        # Initialize plan.
        plan = [goal_config]
        # get the plan from goal to start
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
        return plan, plan_cost

    def extend(self, near, new, step_size):
        if step_size:
            norm = self.planning_env.compute_distance(near, new)
            direction = [(new[0] - near[0]) / norm, (new[1] - near[1]) / norm]
            # try:
            step = [int(round(near[0] + direction[0] * step_size)),
                    int(round(near[1] + direction[1] * step_size))]
            # except Exception as e:
            #     print()
            if round(self.planning_env.compute_distance(near, step)) < round(norm):
                return step
        return new

    def get_sample(self, seen, bais, goal_config, plan_improvement_flag):
        # goal_bias
        if numpy.random.sample() <= bais and not plan_improvement_flag:
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

