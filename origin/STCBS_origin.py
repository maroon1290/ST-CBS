# Author: SimJoonYeol

import heapq
import math
from copy import deepcopy
import yaml
from itertools import combinations
import matplotrecorder

import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from USTRRTstar_origin import USTRRRTstar
from RectangleObstacle import RectangleObstacle
from HighLevelNode_origin import HighLevelNode
from utils import Utils
from Node_origin import Node

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=30., azim=120)

class Conflict:
    def __init__(self):
        self.time = None
        self.agent1 = None
        self.agent2 = None

    def __str__(self):
        return "time: {}, agent1: {}, agent2: {}".format(self.time, self.robot1, self.robot2)


# Multi Agent Rapidly-exploring Random Forest
class STCBS:
    def __init__(self,
                 robot_num: int,
                 start_points: list,
                 goal_points: list,
                 dimensions: int,
                 space_limits: list,
                 robot_radii: list,
                 obstacles: list,
                 lambda_factor: float,
                 max_expand_distance: float,
                 neighbor_radius: float,
                 max_iter: int):
        # set parameters
        self.robot_num = robot_num
        self.start_points = start_points
        self.goal_points = goal_points
        self.dimensions = dimensions
        self.space_limits = space_limits
        self.robot_radii = robot_radii
        self.obstacles = obstacles
        self.lambda_factor = lambda_factor
        self.max_expand_distance = max_expand_distance
        self.neighbor_radius = neighbor_radius
        self.max_iter = max_iter

    def planning(self):
        conflict_tree = []

        root_node = HighLevelNode()
        for i in range(self.robot_num):
            tree = USTRRRTstar(self.start_points[i],
                               self.goal_points[i],
                               self.dimensions,
                               self.space_limits,
                               self.obstacles,
                               self.robot_radii[i],
                               self.lambda_factor,
                               self.max_expand_distance,
                               self.neighbor_radius,
                               self.max_iter)
            root_node.trees.append(tree)

        for tree in root_node.trees:
            path = tree.planning()
            root_node.add_path_to_solution(path)
            root_node.add_space_cost(tree.space_cost)
            root_node.add_time_cost(tree.time_cost)
            root_node.add_space_time_cost(tree.space_time_cost)
        root_node.calculate_all_sum_of_costs()
        # self.coordinate_solution(root_node)
        heapq.heappush(conflict_tree, root_node)

        cur_iter = 0
        while conflict_tree:
            cur_iter += 1
            print(f"cur_iter: {cur_iter}")
            high_level_node = heapq.heappop(conflict_tree)

            self.draw_paths_3d_graph(high_level_node.solution)

            conflict = self.get_first_conflict(high_level_node)

            if conflict is None:
                return high_level_node.solution

            for agent in [conflict.agent1, conflict.agent2]:
                new_high_level_node = deepcopy(high_level_node)
                if len(new_high_level_node.solution[agent]) <= conflict.time:
                    continue
                conflict_node = new_high_level_node.solution[agent][conflict.time]
                new_high_level_node.add_conflict_node(agent, conflict_node)
                new_high_level_node.trees[agent].conflict_node_list = new_high_level_node.conflict_nodes[agent]
                conflict_node.is_conflict = True
                conflict_node.is_invalid = True

                # prune the neighbors of the conflict node
                for node in new_high_level_node.trees[agent].node_list:
                    if node == conflict_node:
                        continue

                    if node.time == conflict_node.time and Utils.calculate_space_distance(node, conflict_node) < self.robot_radii[agent] * 2:
                        self.prune_children(node, new_high_level_node.trees[agent])

                # prune the children of the conflict node
                while conflict_node.children:
                    child = conflict_node.children.pop()
                    self.prune_children(child, new_high_level_node.trees[agent])

                # planning conflict robot for the new node
                # new_high_level_node.trees[agent].max_iter = int(new_high_level_node.trees[agent].max_iter / 2)
                new_path = new_high_level_node.trees[agent].planning()
                if new_path is None:
                    continue
                new_high_level_node.update_space_cost(agent, new_high_level_node.trees[agent].space_cost)
                new_high_level_node.update_time_cost(agent, new_high_level_node.trees[agent].time_cost)
                new_high_level_node.update_space_time_cost(agent, new_high_level_node.trees[agent].space_time_cost)
                new_high_level_node.update_path_in_solution(agent, new_path)
                new_high_level_node.calculate_all_sum_of_costs()
                # self.coordinate_solution(new_high_level_node)
                heapq.heappush(conflict_tree, new_high_level_node)

    def prune_children(self, node: Node, tree: USTRRRTstar):
        while node.children:
            child = node.children.pop()
            self.prune_children(child, tree)

        if node in tree.node_list:
            tree.node_list.remove(node)

    # TODO: implemented
    def coordinate_solution(self, high_level_node: HighLevelNode):
        max_time_step = 0
        for path in high_level_node.solution:

            max_time_step = max(max_time_step, len(path))

        for i in range(self.robot_num):
            while len(high_level_node.solution[i]) < max_time_step:
                pre_node = high_level_node.solution[i][-1]
                new_node = deepcopy(pre_node)
                new_node.time += 1
                new_node.parent = pre_node
                pre_node.add_child(new_node)
                new_node.space_cost = pre_node.space_cost + Utils.calculate_space_distance(pre_node, new_node)
                new_node.time_cost = pre_node.time_cost + Utils.calculate_time_distance(pre_node, new_node)
                new_node.space_time_cost = pre_node.space_time_cost + Utils.calculate_space_time_distance(pre_node, new_node, self.lambda_factor)
                high_level_node.trees[i].node_list.append(new_node)
                high_level_node.solution[i].append(new_node)

    # TODO: implemented
    def get_first_conflict(self, high_level_node: HighLevelNode):
        for (agent1, path1), (agent2, path2) in list(combinations(enumerate(high_level_node.solution), 2)):
            max_time_step = max(len(path1), len(path2))
            for time in range(max_time_step):
                from_node1 = path1[min(time, len(path1) - 1)]
                to_node1 = path1[min(time + 1, len(path1) - 1)]
                from_node2 = path2[min(time, len(path2) - 1)]
                to_node2 = path2[min(time + 1, len(path2) - 1)]
                if self.check_agents_collision(from_node1, to_node1, from_node2, to_node2):
                    conflict = Conflict()
                    conflict.agent1 = agent1
                    conflict.agent2 = agent2
                    conflict.time = time + 1
                    return conflict
        return None

    # TODO: implemented
    def check_agents_collision(self, from_node1: Node, to_node1: Node, from_node2: Node, to_node2: Node):
        num_steps = max(math.ceil(Utils.calculate_space_distance(from_node1, to_node1) / self.robot_radii[0]),
                        math.ceil(Utils.calculate_space_distance(from_node2, to_node2) / self.robot_radii[1]))
        for i in range(1, num_steps + 1):
            t = i / num_steps
            x1 = from_node1.config_point[0] * (1 - t) + to_node1.config_point[0] * t
            y1 = from_node1.config_point[1] * (1 - t) + to_node1.config_point[1] * t

            x2 = from_node2.config_point[0] * (1 - t) + to_node2.config_point[0] * t
            y2 = from_node2.config_point[1] * (1 - t) + to_node2.config_point[1] * t

            if Utils.calculate_space_distance_xy(x1, y1, x2, y2) < self.robot_radii[0] + self.robot_radii[1]:
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

    def draw_paths_3d_graph(self, solution):
        max_time = max([len(path) for path in solution])
        ax.cla()
        ax.set_xlim3d(0, self.space_limits[0])
        ax.set_ylim3d(0, self.space_limits[1])
        ax.set_zlim3d(0, max_time)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('T')
        ax.set_yticklabels([])
        ax.set_xticklabels([])
        ax.set_zticklabels([])
        ax.set_title('High Level of ST-CBS')
        for path in solution:
            x = [node.config_point[0] for node in path]
            y = [node.config_point[1] for node in path]
            t = [node.time for node in path]
            ax.plot(x, y, t, marker='o')
        for obstacle in self.obstacles:
            # Rectangle Obstacle
            if type(obstacle) == RectangleObstacle:
                cube_faces = self.create_cube(obstacle.x, obstacle.y, obstacle.width, obstacle.height, max_time)
                face_collection = Poly3DCollection(cube_faces, facecolor='b', alpha=0.05, linewidths=1, edgecolors='k')
                ax.add_collection3d(face_collection)
        matplotrecorder.save_frame()
        plt.pause(0.1)


if __name__ == '__main__':
    with open("../BaseName.yaml", 'r') as stream:
        basename_yaml = yaml.load(stream, Loader=yaml.FullLoader)
        basename = basename_yaml["basename"]
    with open(f'../configs/{basename}.yaml') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    obstacles = []
    for rect_obstacle in config["rectangleObstacles"]:
        obstacles.append(RectangleObstacle(rect_obstacle[0], rect_obstacle[1], rect_obstacle[2], rect_obstacle[3]))
    # run ST-CBS
    st_cbs = STCBS(
        robot_num=config["robotNum"],
        start_points=config["startPoints"],
        goal_points=config["goalPoints"],
        dimensions=config["dimensions"],
        space_limits=config["spaceLimits"],
        robot_radii=config["robotRadii"],
        obstacles=obstacles,
        lambda_factor=config["lambdaFactor"],
        max_expand_distance=config["maxExpandDistance"],
        neighbor_radius=config["neighborRadius"],
        max_iter=config["maxIteration"],
    )

    solution = st_cbs.planning()
    for path in solution:
        for node in path:
            print(f"x: {node.config_point[0]}, y: {node.config_point[1]}, time: {node.time}")
        print("-----------------")

    dump_list = []
    for path in solution:
        dump_list.append([])
        for node in path:
            dump_list[-1].append([node.config_point[0], node.config_point[1], node.time])
    with open(f"../solutions/{basename}_solution.yaml", 'w') as f:
        yaml.dump(dump_list, f)

    st_cbs.draw_paths_3d_graph(solution)