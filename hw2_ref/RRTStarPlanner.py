import numpy
from hw2_ref.RRTTree import RRTTree
from matplotlib import pyplot as plt
import time

class RRTStarPlanner(object):

    def __init__(self, planning_env, bias = 0.2):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        
        self.bias = bias # bias of sampling to pick the goal

        self.lines = {} # hold the plot lines so we could remove them from the plot if RRtStar find a better path
        
    def Plan(self, start_config, goal_config, epsilon = 30, k=2):
        
        # Initialize an empty plan.
        plan = []
        c_space = list(zip(numpy.where(self.planning_env.map==0)[0], 
                           numpy.where(self.planning_env.map==0)[1]))
        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)
        plan.append(start_config)
        
        # TODO (student): Implement your planner here.
        connected_to_goal = False
        while not connected_to_goal:
            x_rand = self.sample_random_state(c_space,goal_config)
            
            nn_id, nn = self.tree.GetNearestVertex(x_rand)
      
            x_new = self.extend(x_rand, nn, epsilon)
            
            if self.planning_env.edge_validity_checker(nn, x_new, discrete=False):
                
                x_new_id = self.tree.AddVertex(x_new)
                self.tree.AddEdge(nn_id, x_new_id)
                
                plot_x = [nn[0], x_new[0]]
                plot_y = [nn[1], x_new[1]]
                line = plt.plot(plot_y, plot_x,'-or',markersize=1)
                self.lines[(nn_id, x_new_id)] = line
                
                if k is not None:
                    # Option 1: k is a constant
                    _k = min(k, len(self.tree.vertices) - 1)
                    knn_ids, knn = self.tree.GetKNN(x_new,_k)
                else:
                    # Option 2: k=O(log(n)) where n is the number of neighbours in the tree
                    k = int(numpy.floor(numpy.log(len(self.tree.edges))))
                    knn_ids, knn = self.tree.GetKNN(x_new,k)
                
                t = time.time()
                for _knn, _knn_id in zip(knn,knn_ids):
                    old_parent_id = self.rewire_rrt_star(_knn, _knn_id, x_new, x_new_id)
                    if old_parent_id is not None:
                        self.lines[(old_parent_id, x_new_id)].pop(0).remove()
                        plot_x = [_knn[0], x_new[0]]
                        plot_y = [_knn[1], x_new[1]]
                        line = plt.plot(plot_y, plot_x,'-or',markersize=1)
                        self.lines[(_knn_id, x_new_id)] = line
                
                for _knn, _knn_id in zip(knn,knn_ids):
                    old_parent_id = self.rewire_rrt_star(x_new, x_new_id, _knn, _knn_id)
                    if old_parent_id is not None:
                        self.lines[(old_parent_id, _knn_id)].pop(0).remove()
                        plot_x = [x_new[0], _knn[0]]
                        plot_y = [x_new[1], _knn[1]]
                        line = plt.plot(plot_y, plot_x,'-or',markersize=1)
                        self.lines[(x_new_id, _knn_id)] = line
                # print ('rewire:', time.time()-t,'sec')
                
                plan.append(x_new)
                
                if x_new == goal_config:
                    connected_to_goal = True
                    
        return numpy.array(plan)

    def rewire_rrt_star(self, x_potential_parent, x_potential_parent_id, x_child, x_child_id):
        if self.planning_env.edge_validity_checker(x_potential_parent, x_child, discrete=False):
            new_cost = self.planning_env.compute_distance(x_potential_parent, x_child)
            if self.get_cost_from_root(x_potential_parent_id) + new_cost < self.get_cost_from_root(x_child_id):
                old_parent_id = self.tree.edges[x_child_id]
                self.tree.UpdateEdge(x_potential_parent_id, x_child_id)
                return old_parent_id
        return None
    
    def get_cost_from_root(self, config_id):
        root_id = self.tree.GetRootID()
        current = self.tree.vertices[config_id]
        current_id = config_id
        
        cost = 0
        while current_id != root_id:
            parent_id = self.tree.edges[current_id]
            parent = self.tree.vertices[current_id] 
            
            cost += self.planning_env.compute_distance(parent, current)
            
            current_id = parent_id
            current = parent
            
        return cost
        
        
    def sample_random_state(self, c_space, goal):
        sample_from = numpy.random.random()
        
        if sample_from > self.bias: #sample from the entire space
            rnd_idx = numpy.random.randint(1, len(c_space))
            v = list(c_space[rnd_idx])
            while not self.planning_env.state_validity_checker(v):
                rnd_idx = numpy.random.randint(1, len(c_space))
                v = list(c_space[rnd_idx])
            return v
        
        else: # sample only from x_goal. in our case just return the single goal
            return list(goal)
        
    def extend(self, x_rand, x_near, epsilon):
        # TODO (student): Implement an extend logic.
        if epsilon < 0: #E1 nearest neighbor tries to extend all the way till the sampled point
            return x_rand
        
        # E2 nearest neighbor tries to extend to the sampled point by a step size epsilon
        # create a unit vector from nearest neighbor to x_rand
        vec = numpy.array([x_rand[0] - x_near[0], x_rand[1] - x_near[1]])
        if numpy.linalg.norm(vec) == 0:
            unit_vec = numpy.array([0,0])
        else:
            unit_vec = vec / numpy.linalg.norm(vec)
        
        # go epsilon in diretion of vec 
        x_new = epsilon*unit_vec + numpy.array(x_near)
        
        return x_new.tolist()
    
    def ShortenPath(self, path):
        # TODO (student): Postprocessing of the plan.
        current = self.tree.vertices[-1]
        current_id = len(self.tree.vertices) -1
        if current != self.planning_env.goal:
            print ("Path was not found!")
        short_path = [current]
        while current != path[0].tolist():
            for i,prev in enumerate(short_path[:-2]):
                if self.planning_env.edge_validity_checker(current, prev, discrete=False):
                    short_path = short_path[:i+1] + [short_path[-1]]
            current_id = self.tree.edges[current_id]
            current = self.tree.vertices[current_id] 
            short_path.append(current)
        short_path.reverse()
        return numpy.array(short_path)