import os
import numpy
from IPython import embed
from matplotlib import pyplot as plt

COLLISION_FREE = 0
COLLISION = 1

class MapEnvironment(object):
    
    def __init__(self, mapfile, start, goal):

        # Obtain the boundary limits.
        # Check if file exists.
        if not os.path.isfile(mapfile):
            raise ValueError('Map file does not exists');
            exit(0)
        self.map = numpy.loadtxt(mapfile)
        self.xlimit = [1, numpy.shape(self.map)[1]] # TODO (avk): Check if this needs to flip.
        self.ylimit = [1, numpy.shape(self.map)[0]]
        
        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)
        
        self.start = start
        self.goal = goal
        
        # Display the map
        plt.imshow(self.map, interpolation='nearest')
        
    def compute_distance(self, start_config, end_config):
        
        #
        # TODO: Implement a function which computes the distance between
        # two configurations.
        #
        
        # return euclidean distance between configurations (l2-norm) as the cost
        return numpy.linalg.norm(numpy.array(start_config) - numpy.array(end_config))


    def state_validity_checker(self, config):

        #
        # TODO: Implement a state validity checker
        # Return true if valid.
        #
        y, x = config
        
        if x < self.xlimit[0] or x >= self.xlimit[1]:
            return False
        
        if y < self.ylimit[0] or y >= self.ylimit[1]:
            return False
        
        if self.map[y][x] == COLLISION:
            return False
        
        return True

    def edge_validity_checker(self, config1, config2, discrete=True):

        #
        # TODO: Implement an edge validity checker
        #
        #
                
        if discrete:
            if self.map[config1[0]][config1[1]] == COLLISION_FREE and\
               self.map[config2[0]][config2[1]] == COLLISION_FREE:
                   return True
        else:
            if config1 == config2: 
                # if they are the same point, edge must be valid
                return True 
            else:
                vec = numpy.array([config2[0] - config1[0], config2[1] - config1[1]])
                vec_norm = numpy.linalg.norm(vec)
                unit_vec = vec / vec_norm
                
                discretize_path = 11 
                step_size = vec_norm / discretize_path 
                for step in numpy.arange(step_size, vec_norm, step_size):
                    sample = step*unit_vec + numpy.array(config1)
                    cell = [min(self.map.shape[0] - 1, int(numpy.ceil(sample[0] - 0.5))),
                            min(self.map.shape[1] - 1, int(numpy.ceil(sample[1] - 0.5)))]
                    if self.map[cell[0]][cell[1]] == COLLISION:
                        return False
                return True
    
    def compute_heuristic(self, config):
        
        #
        # TODO: Implement a function to compute heuristic.
        #
        
        # return euclidean distance from the goal (l2-norm) as the heuristic
        return numpy.linalg.norm(numpy.array(self.goal) - numpy.array(config))
        

    def visualize_plan(self, plan):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        plt.imshow(self.map, interpolation='nearest')
        
        for i in range(numpy.shape(plan)[0] - 1):
            x = [plan[i,0], plan[i+1, 0]]
            y = [plan[i,1], plan[i+1, 1]]
            plt.plot(y, x, 'w')
        
        plt.show()