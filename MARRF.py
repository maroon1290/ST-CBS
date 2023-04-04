# Author: SimJoonYeol

import matplotlib.pyplot as plt
from itertools import combinations
import math
import heapq
from copy import deepcopy
import numpy as np
from STRRT import SpaceTimeRRT, CircleObstacle


class Conflict:
    def __init__(self):
        self.time = None
        self.robot1 = None
        self.robot2 = None
        self.robot1_node = None
        self.robot2_node = None

    def __str__(self):
        return "time: {}, robot1: {}, robot2: {}".format(self.time, self.robot1, self.robot2)


class HighLevelNode:
    def __init__(self):
        self.space_time_rrts = []
        self.cost = 0
        self.solutions = []

    def __lt__(self, other):
        return self.cost < other.cost


# Multi Agent Rapidly-exploring Random Forest
class MARRF:
    def __init__(self, starts, goals, width, height, robot_radii, lambda_factor, expand_distances, obstacles, robot_num):
        # set parameters
        self.starts = starts
        self.goals = goals
        self.robot_radii = robot_radii
        self.robot_num = robot_num
        self.width = width
        self.height = height
        self.lambda_factor = lambda_factor
        self.expand_distances = expand_distances
        self.obstacles = obstacles

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.solutions = None
        self.cost = None

    def planning(self):
        priority_queue = []

        init_node = HighLevelNode()
        init_node.space_time_rrts = [SpaceTimeRRT(start, goal, width, height, robot_radius, lambda_factor, expand_distance, obstacles) for start, goal, robot_radius, expand_distance in zip(starts, goals, robot_radii, expand_distances)]
        cost, solutions = self.planning_all_space_time_rrts(init_node)
        init_node.cost = cost
        init_node.solutions = solutions
        heapq.heappush(priority_queue, init_node)

        while priority_queue:
            high_level_node = heapq.heappop(priority_queue)
            print(high_level_node)
            conflict = self.get_first_conflict(high_level_node.solutions)
            if not conflict:
                self.solutions = high_level_node.solutions
                self.cost = high_level_node.cost
                break

            conflict_node_idx1 = high_level_node.space_time_rrts[conflict.robot1].nodes.index(conflict.robot1_node)
            conflict_node_idx2 = high_level_node.space_time_rrts[conflict.robot2].nodes.index(conflict.robot2_node)

            for conflict_node_idx, robot in zip([conflict_node_idx1, conflict_node_idx2], [conflict.robot1, conflict.robot2]):
                new_high_level_node = HighLevelNode()
                new_high_level_node.space_time_rrts = deepcopy(high_level_node.space_time_rrts)
                self.set_invalid_by_post_order(high_level_node.space_time_rrts[robot].nodes[conflict_node_idx])
                cost, solutions = self.planning_all_space_time_rrts(new_high_level_node)
                new_high_level_node.cost = cost
                new_high_level_node.solutions = solutions
                heapq.heappush(priority_queue, new_high_level_node)

        print("cost: ", self.cost)
        for solution in self.solutions:
            for node in solution:
                print(node.x, node.y, node.t)
            print("--------------------------------")
        self.draw_paths_3d_graph(self.solutions)

    def set_invalid_by_post_order(self, node):
        if node is None:
            return
        node.is_valid = False
        for child in node.children:
            self.set_invalid_by_post_order(child)

    @staticmethod
    def planning_all_space_time_rrts(high_level_node):
        solutions = []
        sum_of_cost = 0
        for space_time_rrt in high_level_node.space_time_rrts:
            cost, solution = space_time_rrt.planning()
            sum_of_cost += cost
            solutions.append(solution)
        return sum_of_cost, solutions

    def get_first_conflict(self, paths):
        conflict = Conflict()
        max_t = max([len(path) for path in paths])
        for t in range(max_t):
            for (robot1, path1), (robot2, path2) in list(combinations(enumerate(paths), 2)):
                if t >= len(path1) or t >= len(path2):
                    continue
                if self.is_conflict(path1[t], path2[t], self.robot_radii[robot1], self.robot_radii[robot2]):
                    conflict.time = t
                    conflict.robot1 = robot1
                    conflict.robot2 = robot2
                    conflict.robot1_node = path1[t]
                    conflict.robot2_node = path2[t]
                    return conflict
        return None

    def is_conflict(self, node1, node2, robot_radius1, robot_radius2):
        distance = math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
        if distance <= (robot_radius1 + robot_radius2):
            return True
        else:
            return False

    def draw_paths_3d_graph(self, paths):
        for path in paths:
            x = [node.x for node in path]
            y = [node.y for node in path]
            t = [node.t for node in path]
            self.ax.plot(x, y, t)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Time')
        for obstacle in self.obstacles:
            u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
            x = obstacle.x + obstacle.r * np.cos(u) * np.sin(v)
            y = obstacle.y + obstacle.r * np.sin(u) * np.sin(v)
            z = obstacle.r * np.cos(v)
            self.ax.plot_wireframe(x, y, z, color="blue")
        plt.show()


if __name__ == '__main__':
    # set parameters
    robot_num = 4
    starts = [(4, 10), (16, 10), (10, 4), (10, 16)]
    goals = [(16, 10), (4, 10), (10, 16), (10, 4)]
    robot_radii = [1.5 for _ in range(robot_num)]
    width = 20
    height = 20
    lambda_factor = 0.5
    expand_distances = [robot_radius * 2 for robot_radius in robot_radii]
    obstacles = [
        CircleObstacle(4.0, 4.0, 4),
        # CircleObstacle(10.0, 6.0, 2),
        CircleObstacle(16.0, 4.0, 4),
        CircleObstacle(4.0, 16.0, 4),
        # CircleObstacle(10.0, 18.0, 2.0),
        CircleObstacle(16.0, 16.0, 4),
    ]

    # run MARRF
    marrf = MARRF(starts, goals, width, height, robot_radii, lambda_factor, expand_distances, obstacles, robot_num)
    paths = marrf.planning()

