import numpy
from RRTTree import RRTTree
from matplotlib import pyplot as plt

class RRTPlanner(object):

    def __init__(self, planning_env, bias = 0.05):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        
        self.bias = bias # bias of sampling to pick the goal
        
    def Plan(self, start_config, goal_config, epsilon = -1):
        
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
                plt.plot(plot_y, plot_x,'-or',markersize=1)
                plan.append(x_new)
        
                if x_new == goal_config:
                    connected_to_goal = True
                    
        return numpy.array(plan)

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
