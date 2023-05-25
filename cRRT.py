import math
import random

from shapely.geometry import Point, box
from itertools import combinations
from USTRRTstar import RectangleObstacle, CircleObstacle
import time
import os
import yaml


class SubNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Node:
    def __init__(self):
        self.x_list = []
        self.y_list = []
        self.parent = None


class cRRT:

    def __init__(self):
        self.starts = config["starts"]
        self.goals = config["goals"]
        self.width = config["width"]
        self.height = config["height"]
        self.robot_radii = config["robot_radii"]
        self.robot_radius = self.robot_radii[0]
        self.lambda_factor = config["lambda_factor"]
        self.expand_distances = config["expand_distances"]
        self.expand_distance = self.expand_distances[0]
        self.obstacles = obstacles
        self.robot_num = config["robot_num"]
        self.neighbor_radius = config["neighbor_radius"]
        self.max_iter = config["max_iter"]
        self.makespan = None
        self.sum_of_costs = None
        self.solutions = None

        self.node_list = []

    def planning(self):
        start_node = Node()
        start_node.x_list = [start[0] for start in self.starts]
        start_node.y_list = [start[0] for start in self.starts]
        self.node_list.append(start_node)
        while True:
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_distance)

            if not new_node:
                continue

            if self.is_collide(nearest_node, new_node, self.obstacles):
                self.node_list.append(new_node)
                near_goal_flag = True
                for robot in range(self.robot_num):
                    dx = new_node.x_list[robot] - self.goals[robot][0]
                    dy = new_node.y_list[robot] - self.goals[robot][1]
                    d = math.hypot(dx, dy)
                    if d > self.expand_distance:
                        near_goal_flag = False
                        break

                if near_goal_flag:
                    final_node = self.steer(new_node, self.goals, self.expand_distance)
                    if self.is_collide(new_node, final_node, self.obstacles):
                        self.node_list.append(final_node)
                        self.makespan, self.sum_of_costs, self.solutions = self.generate_final_course(final_node)
                        break

        return None

    def generate_final_course(self, goal_node):
        path = [[goal_node.x_list, goal_node.y_list]]
        node = goal_node
        sum_of_costs = 0
        while node.parent is not None:
            for robot in range(self.robot_num):
                dx = node.x_list[robot] - node.parent.x_list[robot]
                dy = node.y_list[robot] - node.parent.y_list[robot]
                d = math.hypot(dx, dy)
                sum_of_costs += d
            node = node.parent
            path.append([node.x_list, node.y_list])
        path.append([node.x_list, node.y_list])
        return path, sum_of_costs, None

    def get_random_node(self):
        # random sampling for self.robot_num times
        random_node = Node()
        random_node.x_list = [random.uniform(0, self.width) for _ in range(self.robot_num)]
        random_node.y_list = [random.uniform(0, self.height) for _ in range(self.robot_num)]
        return random_node

    def is_collide(self, from_node, to_node, obstacles):
        for robot in range(self.robot_num):
            sub_from_node = SubNode(from_node.x_list[robot], from_node.y_list[robot])
            sub_to_node = SubNode(to_node.x_list[robot], to_node.y_list[robot])
            for obstacle in obstacles:
                if obstacle.is_collide(sub_from_node, sub_to_node, self.robot_radius):
                    return True
        return False


    def get_nearest_node_index(self, node_list, random_node):
        dlist = []
        for node in node_list:
            d_sum = 0
            for robot in range(self.robot_num):
                dx = node.x_list[robot] - random_node.x_list[robot]
                dy = node.y_list[robot] - random_node.y_list[robot]
                d_sum += math.hypot(dx * dx, dy * dy)
            dlist.append(d_sum)
        minind = dlist.index(min(dlist))
        return minind

    def steer(self, from_node, to_node, expand_dis):
        new_node = Node()
        new_node.x_list = from_node.x_list[:]
        new_node.y_list = from_node.y_list[:]

        n_expand = math.floor(expand_dis / self.robot_radius)

        for _ in range(n_expand):
            for robot in range(self.robot_num):
                dx = to_node.x_list[robot] - from_node.x_list[robot]
                dy = to_node.y_list[robot] - from_node.y_list[robot]
                d = math.hypot(dx, dy)
                theta = math.atan2(dy, dx)
                if new_node.x_list[robot] == to_node.x_list[robot] and new_node.y_list[robot] == to_node.y_list[robot]:
                    continue

                new_node.x_list[robot] += self.robot_radius * math.cos(theta)
                new_node.y_list[robot] += self.robot_radius * math.sin(theta)

                d = math.hypot(to_node.x_list[robot] - new_node.x_list[robot], to_node.y_list[robot] - new_node.y_list[robot])
                if d <= self.robot_radius:
                    new_node.x_list[robot] = to_node.x_list[robot]
                    new_node.y_list[robot] = to_node.y_list[robot]

            # collision checking
            for robot1, robot2 in combinations(range(self.robot_num), 2):
                if robot1 == robot2:
                    continue

                d = math.hypot(new_node.x_list[robot1] - new_node.x_list[robot2], new_node.y_list[robot1] - new_node.y_list[robot2])
                if d <= 2 * self.robot_radius:
                    return None

        new_node.parent = from_node
        return new_node



if __name__ == '__main__':
    config_name = "OpenEnvironment_5_0"
    # read config.yaml
    with open(os.path.join("configs", config_name + ".yaml"), "r") as file:
        config = yaml.safe_load(file)

    # make obstacles
    obstacles = []
    for config_obstacle in config["obstacles"]:
        if config_obstacle["type"] == "CircleObstacle":
            obstacle = CircleObstacle(config_obstacle["x"], config_obstacle["y"], config_obstacle["radius"])
        elif config_obstacle["type"] == "RectangleObstacle":
            obstacle = RectangleObstacle(config_obstacle["x"], config_obstacle["y"], config_obstacle["width"],
                                         config_obstacle["height"])
        else:
            raise ValueError("invalid obstacle type")
        obstacles.append(obstacle)

    # run p
    c_rrt = cRRT()

    start_time = time.time()
    makespan, sum_of_costs, solutions = c_rrt.planning()
    print("Time: ", time.time() - start_time)
    print("Sum of costs: ", sum_of_costs)
    print("Makespan: ", makespan)
    for solution in solutions:
        for node in solution:
            print(f"[{node.x}, {node.y}, {node.t}],")
        print("--------------------------------")

    # save solutions to yaml
    with open(f"cRRT_solutions/{config_name}_solutions.yaml", "w") as file:
        solutions_list = []
        for solution in solutions:
            solution_list = []
            for node in solution:
                solution_list.append([node.x, node.y, node.t])
            solutions_list.append(solution_list)
        yaml.dump(solutions_list, file)
