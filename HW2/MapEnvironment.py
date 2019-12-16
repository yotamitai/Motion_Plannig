import numpy
# from IPython import embed
from matplotlib import pyplot as plt


class MapEnvironment(object):

    def __init__(self, mapfile, start, goal):

        # Obtain the boundary limits.
        # Check if file exists.
        self.map = numpy.loadtxt(mapfile)
        self.xlimit = [1, numpy.shape(self.map)[0]]  # TODO (avk): Check if this needs to flip.
        self.ylimit = [1, numpy.shape(self.map)[1]]

        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)

        # Display the map
        plt.imshow(self.map, interpolation='nearest')

    def compute_distance(self, start_config, end_config):

        # Done: Implement a function which computes the distance between two configurations.

        # Note: most probably eucledean distance.
        return numpy.sqrt((start_config[0] - end_config[0]) ** 2 + (start_config[1] - end_config[1]) ** 2)

    def state_validity_checker(self, config):
        # DONE: Implement a state validity checker
        # Return true if valid.
        in_range_x = self.xlimit[0] <= config[0] < self.xlimit[1]
        in_range_y = self.ylimit[0] <= config[1] < self.ylimit[1]
        if False in [in_range_x, in_range_y]:
            return False
        else:
            not_in_obstacle = not (bool(self.map[config[0], config[1]]))
            return not_in_obstacle

    def edge_validity_checker(self, config1, config2, rrt=None):
        # defined in each planner as collision_free()
        pass

    def compute_heuristic(self, config):

        #
        # TODO: Implement a function to compute heuristic.
        #
        start, goal = config
        h = numpy.linalg.norm((numpy.array(start) - numpy.array(goal)))
        return h

    def visualize_plan(self, plan):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        plt.imshow(self.map, interpolation='nearest')
        for i in range(numpy.shape(plan)[0] - 1):
            x = [plan[i, 0], plan[i + 1, 0]]
            y = [plan[i, 1], plan[i + 1, 1]]
            plt.plot(y, x, 'k')
        plt.show()
