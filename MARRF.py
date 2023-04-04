# Author: SimJoonYeol

import matplotlib.pyplot as plt

from STRRT import SpaceTimeRRT


class Conflict:
    def __init__(self):
        self.time = None
        self.robot1 = None
        self.robot2 = None

    def __str__(self):
        return "time: {}, robot1: {}, robot2: {}".format(self.time, self.robot1, self.robot2)


# Multi Agent Rapidly-exploring Random Forest
class MARRF:
    def __init__(self, starts, goals, width, height, robot_radii, lambda_factor, expand_distances, obstacles,
                 robot_num):
        # set parameters
        self.starts = starts
        self.goals = goals
        self.robot_radius = robot_radii
        self.robot_num = robot_num
        self.width = width
        self.height = height
        self.lambda_factor = lambda_factor
        self.expand_distances = expand_distances
        self.obstacles = obstacles
        self.space_time_rrts = [
            SpaceTimeRRT(start, goal, width, height, robot_radius, lambda_factor, expand_distance, obstacles) for
            start, goal, robot_radius, expand_distance in zip(starts, goals, robot_radii, expand_distances)]

    def planning(self):
        # planning all space-time RRTs
        paths = [space_time_rrt.planning() for space_time_rrt in self.space_time_rrts]
        self.get_first_conflict(paths)

    def get_first_conflict(self, paths):
        conflict = Conflict()
        max_t = max([len(path) for path in paths])



if __name__ == '__main__':
    # set parameters
    robot_num = 2
    starts = [(5, 5), (15, 15)]
    goals = [(15, 15), (5, 5)]
    robot_radii = [2 for _ in range(robot_num)]
    width = 20
    height = 20
    lambda_factor = 0.5
    expand_distances = [robot_radius for robot_radius in robot_radii]
    obstacles = []

    # run MARRF
    marrf = MARRF(starts, goals, width, height, robot_radii, lambda_factor, expand_distances, obstacles, robot_num)
    marrf.planning()
