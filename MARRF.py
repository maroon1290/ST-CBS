# Author: SimJoonYeol

import numpy as np
import math
import random
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
    def __init__(self, starts, goals, width, height, robot_radii, lambda_factor, expand_dis, obstacles, robot_num):
        # set parameters
        self.starts = starts
        self.goals = goals
        self.robot_radius = robot_radii
        self.robot_num = robot_num

        # set tree
        self.space_time_rrts = [
            SpaceTimeRRT(starts[i], goals[i], width, height, robot_radii[i], lambda_factor, expand_dis, obstacles)
            for i in range(robot_num)]

        paths = [space_time_rrt.planning() for space_time_rrt in self.space_time_rrts]
        conflict = self.get_first_conflict(paths)

    def get_first_conflict(self, paths):
        conflict = Conflict()
        for path in paths:
            for other_path in paths:
                if path == other_path:
                    continue
                for time in range(len(path)):
                    if path[time] == other_path[time]:
                        conflict.time = time
                        conflict.robot1 = path
                        conflict.robot2 = other_path
                        return conflict
