# Author: SimJoonYeol

import heapq
import math
import os
import time
from copy import deepcopy
from itertools import combinations

import matplotlib.pyplot as plt
import numpy as np
import yaml

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from USTRRTstar import USTRRRTstar, CircleObstacle, RectangleObstacle


class Conflict:
    def __init__(self):
        self.time = None
        self.robot1 = None
        self.robot2 = None
        self.robot1_key = None
        self.robot2_key = None

    def __str__(self):
        return "time: {}, robot1: {}, robot2: {}".format(self.time, self.robot1, self.robot2)


class HighLevelNode:
    def __init__(self):
        self.trees = []
        self.space_costs = []
        self.time_costs = []
        self.space_time_costs = []
        self.solutions = []
        self.sum_of_space_costs = None
        self.sum_of_time_costs = None
        self.sum_of_space_time_costs = None

    def __lt__(self, other):
        return self.sum_of_space_time_costs < other.sum_of_space_time_costs

    def set_sum_of_space_costs(self):
        self.sum_of_space_costs = sum(self.space_costs)

    def set_sum_of_time_costs(self):
        self.sum_of_time_costs = sum(self.time_costs)

    def set_sum_of_space_time_costs(self):
        self.sum_of_space_time_costs = sum(self.space_time_costs)


# Multi Agent Rapidly-exploring Random Forest
class STCBS:
    def __init__(self, starts, goals, width, height, robot_radii, lambda_factor, expand_distances, obstacles, robot_num, neighbor_radius, max_iter):
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
        self.neighbor_radius = neighbor_radius
        self.max_iter = max_iter

        # self.fig = plt.figure(figsize=(10, 10))
        # self.ax = self.fig.add_subplot(111, projection='3d')
        self.animation = False
        self.draw_result = False

        self.solutions = None
        self.sum_of_space_costs = None
        self.space_makespan = None
        self.sum_of_time_costs = None
        self.time_makespan = None
        self.sum_of_space_time_costs = None
        self.space_time_makespan = None

    def planning(self):
        priority_queue = []

        init_node = HighLevelNode()
        for start, goal, robot_radius, expand_distance in zip(self.starts, self.goals, self.robot_radii, self.expand_distances):
            space_time_rrt = USTRRRTstar(start, goal, self.width, self.height, robot_radius, self.lambda_factor, expand_distance, self.obstacles, self.neighbor_radius, self.max_iter)
            init_node.trees.append(space_time_rrt)
        init_node.space_costs, init_node.time_costs, init_node.space_time_costs, init_node.solutions = self.planning_all_space_time_rrts(init_node.trees)
        init_node.set_sum_of_space_time_costs()
        heapq.heappush(priority_queue, init_node)

        cur_iter = 0
        while priority_queue:
            high_level_node = heapq.heappop(priority_queue)

            if self.animation:
                self.draw_paths_3d_graph(high_level_node.solutions)

            conflict = self.get_first_conflict(high_level_node)

            if not conflict:
                self.solutions = high_level_node.solutions
                high_level_node.set_sum_of_space_costs()
                high_level_node.set_sum_of_time_costs()
                high_level_node.set_sum_of_space_time_costs()
                self.sum_of_space_costs = high_level_node.sum_of_space_costs
                self.space_makespan = max(high_level_node.space_costs)
                self.sum_of_time_costs = high_level_node.sum_of_time_costs
                self.time_makespan = max(high_level_node.time_costs)
                self.sum_of_space_time_costs = high_level_node.sum_of_space_time_costs
                self.space_time_makespan = max(high_level_node.space_time_costs)
                break

            for robot, key in [(conflict.robot1, conflict.robot1_key), (conflict.robot2, conflict.robot2_key)]:
                new_high_level_node = deepcopy(high_level_node)
                conflict_node = list(filter(lambda node: node.key == key,
                                            new_high_level_node.trees[robot].node_list))[0]

                new_high_level_node.trees[robot].max_iter /= 2

                infection_nodes = []
                for node in new_high_level_node.trees[robot].node_list:
                    if node == conflict_node:
                        continue

                    if node.t == conflict_node.t and node.space_distance(conflict_node) < self.robot_radii[robot] * 2:
                        node.is_infected = True
                        infection_nodes.append(node)

                for infection_node in infection_nodes:
                    while infection_node.children:
                        child = infection_node.children.popleft()
                        self.prune_children(child, new_high_level_node.trees[robot].node_list)

                while conflict_node.children:
                    child = conflict_node.children.popleft()
                    self.prune_children(child, new_high_level_node.trees[robot].node_list)

                conflict_node.is_valid = False
                space_cost, time_cost, space_time_cost, solution = new_high_level_node.trees[robot].planning()
                new_high_level_node.space_costs[robot] = space_cost
                new_high_level_node.time_costs[robot] = time_cost
                new_high_level_node.space_time_costs[robot] = space_time_cost
                new_high_level_node.solutions[robot] = solution
                new_high_level_node.set_sum_of_space_costs()
                new_high_level_node.set_sum_of_time_costs()
                new_high_level_node.set_sum_of_space_time_costs()
                heapq.heappush(priority_queue, new_high_level_node)
            cur_iter += 1

        if self.draw_result:
            self.draw_paths_3d_graph(self.solutions)
        return self.space_makespan, self.sum_of_space_costs, self.time_makespan, self.sum_of_time_costs, self.space_time_makespan, self.sum_of_space_time_costs, self.solutions

    def prune_children(self, node, node_list: set):
        while node.children:
            child = node.children.popleft()
            self.prune_children(child, node_list)

        if node in node_list:
            node_list.remove(node)

    def planning_all_space_time_rrts(self, trees):
        space_costs = []
        time_costs = []
        space_time_costs = []
        solutions = []
        for tree in trees:
            space_cost, time_cost, space_time_cost, solution = tree.planning()
            space_costs.append(space_cost)
            time_costs.append(time_cost)
            space_time_costs.append(space_time_cost)
            solutions.append(solution)
        return space_costs, time_costs, space_time_costs, solutions

    def get_first_conflict(self, high_level_node):
        paths = high_level_node.solutions
        conflict = Conflict()
        max_t = max([len(path) for path in paths])
        padded_paths = [path + [path[-1]] * (max_t - len(path)) for path in paths]

        for t in range(max_t):
            for (robot1, path1), (robot2, path2) in list(combinations(enumerate(padded_paths), 2)):
                if t == 0:
                    continue
                if self.is_conflict_continuous(path1[t - 1], path1[t], path2[t - 1], path2[t], self.robot_radii[robot1],
                                               self.robot_radii[robot2]):
                    conflict.time = t
                    conflict.robot1 = robot1
                    conflict.robot2 = robot2
                    conflict.robot1_key = path1[t].key
                    conflict.robot2_key = path2[t].key
                    return conflict
        return None

    @staticmethod
    def linear_interpolate(prev_node, next_node, radius):
        dx = next_node.x - prev_node.x
        dy = next_node.y - prev_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        interpolated_x = [prev_node.x]
        interpolated_y = [prev_node.y]
        n_expand = math.floor(d / radius)
        for _ in range(n_expand):
            interpolated_x.append(interpolated_x[-1] + radius * math.cos(theta))
            interpolated_y.append(interpolated_y[-1] + radius * math.sin(theta))

        d = math.hypot(next_node.x - interpolated_x[-1], next_node.y - interpolated_y[-1])
        if d <= radius:
            interpolated_x.append(next_node.x)
            interpolated_y.append(next_node.y)
        return interpolated_x, interpolated_y

    def is_conflict_continuous(self, prev_node1, next_node1, prev_node2, next_node2, robot_radius1, robot_radius2):
        x1, y1 = self.linear_interpolate(prev_node1, next_node1, robot_radius1)
        x2, y2 = self.linear_interpolate(prev_node2, next_node2, robot_radius2)
        max_step = max(len(x1), len(x2))
        # fill the shorter path
        x1 = np.append(x1, [x1[-1]] * (max_step - len(x1)))
        y1 = np.append(y1, [y1[-1]] * (max_step - len(y1)))
        x2 = np.append(x2, [x2[-1]] * (max_step - len(x2)))
        y2 = np.append(y2, [y2[-1]] * (max_step - len(y2)))
        for i in range(max_step):
            if math.hypot(x1[i] - x2[i], y1[i] - y2[i]) <= (robot_radius1 + robot_radius2):
                return True
        return False

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
        self.ax.set_yticklabels([])
        self.ax.set_xticklabels([])
        self.ax.set_zticklabels([])
        self.ax.set_title('High Level of ST-CBS')
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
        plt.pause(0.1)


if __name__ == '__main__':
    config_name = "CluttedEnvironment_5_0"
    # read config.yaml
    with open(os.path.join("configs", config_name + ".yaml"), "r") as file:
        config = yaml.safe_load(file)
    print(config["lambda_factor"])
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

    # run ST-CBS
    st_cbs = STCBS(
        config["starts"],
        config["goals"],
        config["width"],
        config["height"],
        config["robot_radii"],
        config["lambda_factor"],
        config["expand_distances"],
        obstacles,
        config["robot_num"],
        config["neighbor_radius"],
        config["max_iter"]
    )

    start_time = time.time()
    space_makespan, sum_of_space_costs, time_makespan, sum_of_time_costs, space_time_makespan, sum_of_space_time_costs, solutions = st_cbs.planning()
    print("Time: ", time.time() - start_time)
    print("Space Makespan: ", space_makespan)
    print("Sum of Space Costs: ", sum_of_space_costs)
    print("Time Makespan: ", time_makespan)
    print("Sum of Time Costs: ", sum_of_time_costs)
    print("Space-Time Makespan: ", space_time_makespan)
    print("Sum of Space-Time Costs: ", sum_of_space_time_costs)
    print(str(space_makespan) + ", " + str(sum_of_space_costs) + ", " + str(time_makespan) + ", " + str(sum_of_time_costs) + ", " + str(space_time_makespan) + ", " + str(sum_of_space_time_costs) + ", " + str(time.time() - start_time))

    for solution in solutions:
        for node in solution:
            print(f"[{node.x}, {node.y}, {node.t}],")
        print("--------------------------------")

    # save solutions to yaml
    with open(f"solutions/{config_name}_solutions.yaml", "w") as file:
        solutions_list = []
        for solution in solutions:
            solution_list = []
            for node in solution:
                solution_list.append([node.x, node.y, node.t])
            solutions_list.append(solution_list)
        yaml.dump(solutions_list, file)
