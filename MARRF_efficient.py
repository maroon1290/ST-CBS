# Author: SimJoonYeol

import heapq
import math
from copy import deepcopy
from itertools import combinations

import matplotlib.pyplot as plt
import numpy as np
import yaml

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from STRRT import SpaceTimeRRT, CircleObstacle, RectangleObstacle


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
        self.disable_nodes = []
        self.cost = 0
        self.solutions = []

    def __lt__(self, other):
        return self.cost < other.cost


# Multi Agent Rapidly-exploring Random Forest
class MARRF:
    def __init__(self, starts, goals, width, height, robot_radii, lambda_factor, expand_distances, obstacles,
                 robot_num):
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
        self.space_time_rrts = []

    def planning(self):
        priority_queue = []

        init_node = HighLevelNode()
        for start, goal, robot_radius, expand_distance in zip(self.starts, self.goals, self.robot_radii, self.expand_distances):
            space_time_rrt = SpaceTimeRRT(start, goal, self.width, self.height, robot_radius, self.lambda_factor,
                                          expand_distance, self.obstacles)
            self.space_time_rrts.append(space_time_rrt)
        init_node.cost, init_node.solutions = self.planning_all_space_time_rrts()
        heapq.heappush(priority_queue, init_node)

        while priority_queue:
            high_level_node = heapq.heappop(priority_queue)
            print(high_level_node)
            conflict = self.get_first_conflict(high_level_node.solutions)
            if not conflict:
                self.solutions = high_level_node.solutions
                self.cost = high_level_node.cost
                break

            conflict_node_idx1 = self.space_time_rrts[conflict.robot1].nodes.index(conflict.robot1_node)
            conflict_node_idx2 = self.space_time_rrts[conflict.robot2].nodes.index(conflict.robot2_node)

            for conflict_node_idx, robot in zip([conflict_node_idx1, conflict_node_idx2],
                                                [conflict.robot1, conflict.robot2]):
                new_high_level_node = HighLevelNode()
                new_high_level_node.disable_nodes = high_level_node.disable_nodes[:]
                new_high_level_node.disable_nodes.append(self.space_time_rrts[robot].nodes[conflict_node_idx])
                for disable_node in new_high_level_node.disable_nodes:
                    self.set_invalid_by_post_order(disable_node)
                cost, solutions = self.planning_all_space_time_rrts()
                for disable_node in new_high_level_node.disable_nodes:
                    self.set_valid_by_post_order(disable_node)
                new_high_level_node.cost = cost
                new_high_level_node.solutions = solutions
                heapq.heappush(priority_queue, new_high_level_node)

        self.draw_paths_3d_graph(self.solutions)
        return self.cost, self.solutions

    def set_invalid_by_post_order(self, disable_node):
        if disable_node is None or not disable_node.is_valid:
            return
        disable_node.is_valid = False
        for child in disable_node.children:
            self.set_invalid_by_post_order(child)

    def set_valid_by_post_order(self, enable_node):
        if enable_node is None or enable_node.is_valid:
            return
        enable_node.is_valid = True
        for child in enable_node.children:
            self.set_valid_by_post_order(child)

    def planning_all_space_time_rrts(self):
        solutions = []
        sum_of_cost = 0
        for space_time_rrt in self.space_time_rrts:
            cost, solution = space_time_rrt.planning()
            sum_of_cost += cost
            solutions.append(solution)
        return sum_of_cost, solutions

    def get_first_conflict(self, paths):
        conflict = Conflict()
        max_t = max([len(path) for path in paths])
        for t in range(max_t):
            for (robot1, path1), (robot2, path2) in list(combinations(enumerate(paths), 2)):
                if t == 0:
                    continue
                if t >= len(path1):
                    path1 = path1 + [path1[-1]] * (t - len(path1) + 1)
                if t >= len(path2):
                    path2 = path2 + [path2[-1]] * (t - len(path2) + 1)
                if self.is_conflict_continuous(path1[t - 1], path1[t], path2[t - 1], path2[t], self.robot_radii[robot1], self.robot_radii[robot2]):
                    conflict.time = t
                    conflict.robot1 = robot1
                    conflict.robot2 = robot2
                    conflict.robot1_node = path1[t]
                    conflict.robot2_node = path2[t]
                    return conflict
        return None

    @staticmethod
    def linear_interpolate(prev_node, next_node, radius):
        euclidean_distance = math.hypot(next_node.x - prev_node.x, next_node.y - prev_node.y)
        step_size = round(euclidean_distance / radius)
        x = np.linspace(prev_node.x, next_node.x, step_size)
        y = np.linspace(prev_node.y, next_node.y, step_size)
        return x, y

    @staticmethod
    def is_conflict_discrete(node1, node2, robot_radius1, robot_radius2):
        distance = math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
        if distance <= (robot_radius1 + robot_radius2):
            return True
        else:
            return False

    def is_conflict_continuous(self, prev_node1, next_node1, prev_node2, next_node2, robot_radius1, robot_radius2):
        x1, y1 = self.linear_interpolate(prev_node1, next_node1, robot_radius1)
        x2, y2 = self.linear_interpolate(prev_node2, next_node2, robot_radius2)
        max_step = max(len(x1), len(x2))
        for i in range(max_step):
            if i < len(x1) and i < len(x2):
                if math.hypot(x1[i] - x2[i], y1[i] - y2[i]) <= (robot_radius1 + robot_radius2):
                    return True
            elif i < len(x1):
                if math.hypot(x1[i] - next_node2.x, y1[i] - next_node2.y) <= (robot_radius1 + robot_radius2):
                    return True
            else:
                if math.hypot(x2[i] - next_node1.x, y2[i] - next_node1.y) <= (robot_radius1 + robot_radius2):
                    return True

    @staticmethod
    def create_cube(center_x, center_y, width, height, depth):
        x, y = center_x - width / 2, center_y - height / 2

        vertices = np.array([
            [x, y, 0],
            [x + width, y, 0],
            [x + width, y + height, 0],
            [x, y + height, 0],
            [x, y, depth],
            [x + width, y, depth],
            [x + width, y + height, depth],
            [x, y + height, depth]
        ])

        faces = [
            [vertices[0], vertices[1], vertices[5], vertices[4]],
            [vertices[7], vertices[6], vertices[2], vertices[3]],
            [vertices[0], vertices[1], vertices[2], vertices[3]],
            [vertices[7], vertices[6], vertices[5], vertices[4]],
            [vertices[7], vertices[3], vertices[0], vertices[4]],
            [vertices[1], vertices[2], vertices[6], vertices[5]]
        ]

        return faces

    def draw_paths_3d_graph(self, paths):
        max_time = max([len(path) for path in paths])
        self.ax.cla()
        self.ax.set_xlim3d(0, self.width)
        self.ax.set_ylim3d(0, self.height)
        self.ax.set_zlim3d(0, max_time)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('T')
        self.ax.set_title('Multi Agent Rapidly Random Forest')
        for path in paths:
            x = [node.x for node in path]
            y = [node.y for node in path]
            t = [node.t for node in path]
            self.ax.plot(x, y, t)
        for obstacle in self.obstacles:
            # Circle Obstacle
            if type(obstacle) == CircleObstacle:
                u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
                x = obstacle.x + obstacle.r * np.cos(u) * np.sin(v)
                y = obstacle.y + obstacle.r * np.sin(u) * np.sin(v)
                z = obstacle.r * np.cos(v)
                self.ax.plot_wireframe(x, y, z, color="blue")

            # Rectangle Obstacle
            if type(obstacle) == RectangleObstacle:
                cube_faces = self.create_cube(obstacle.x, obstacle.y, obstacle.width, obstacle.height, max_time)
                face_collection = Poly3DCollection(cube_faces, facecolor='b', alpha=0.1, linewidths=1, edgecolors='k')
                self.ax.add_collection3d(face_collection)
        plt.show()


if __name__ == '__main__':
    # read config.yaml
    with open("configs/deadlock_config.yaml", "r") as file:
        config = yaml.safe_load(file)

    # make obstacles
    obstacles = []
    for config_obstacle in config["obstacles"]:
        if config_obstacle["type"] == "CircleObstacle":
            obstacle = CircleObstacle(config_obstacle["x"], config_obstacle["y"], config_obstacle["radius"])
        elif config_obstacle["type"] == "RectangleObstacle":
            obstacle = RectangleObstacle(config_obstacle["x"], config_obstacle["y"], config_obstacle["width"], config_obstacle["height"])
        else:
            raise ValueError("invalid obstacle type")
        obstacles.append(obstacle)

    # run MARRF
    marrf = MARRF(
        config["starts"],
        config["goals"],
        config["width"],
        config["height"],
        config["robot_radii"],
        config["lambda_factor"],
        config["expand_distances"],
        obstacles,
        config["robot_num"]
    )

    cost, solutions = marrf.planning()
    print("cost: ", cost)
    for solution in solutions:
        for node in solution:
            print(f"[{node.x}, {node.y}, {node.t}],")
        print("--------------------------------")

    # save solutions to yaml
    with open("solutions.yaml", "w") as file:
        solutions_list = []
        for solution in solutions:
            solution_list = []
            for node in solution:
                solution_list.append([node.x, node.y, node.t])
            solutions_list.append(solution_list)
        yaml.dump(solutions_list, file)
