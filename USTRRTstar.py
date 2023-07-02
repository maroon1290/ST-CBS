import math
import time
import random

from Node import Node
from utils import Utils
from RectangleObstacle import RectangleObstacle

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.view_init(elev=30., azim=120)


class USTRRRTstar:
    def __init__(self,
                 start_point: list,
                 goal_point: list,
                 dimension: int,
                 space_limits: list,
                 obstalces: list,
                 robot_radius: float,
                 lambda_factor: float,
                 max_expand_dis: float,
                 neighbor_radius: float,
                 max_iter: int, ):
        self.start_point = start_point
        self.goal_point = goal_point
        self.dimension = dimension
        self.space_limits = space_limits
        self.obstacles = obstalces
        self.robot_radius = robot_radius
        self.lambda_factor = lambda_factor
        self.max_expand_dis = max_expand_dis
        self.neighbor_radius = neighbor_radius
        self.max_iter = max_iter

        self.max_time = 1
        self.space_cost = float("inf")
        self.time_cost = float("inf")
        self.space_time_cost = float("inf")
        self.node_list = list()
        self.conflict_node_list = list()

        self.start_node = Node(start_point, 0)
        self.start_node.space_cost = 0
        self.start_node.time_cost = 0
        self.start_node.space_time_cost = 0
        self.node_list.append(self.start_node)
        self.goal_node = Node(goal_point, -1)
        self.last_node = None

        # set figure
        self.animation = False
        self.draw_result = False

    def planning(self):
        for i in range(self.max_iter):
            rand_vector, random_time = self.get_random_state()
            random_node = Node(rand_vector, random_time)
            nearest_node = self.get_nearest_node(random_node)
            new_node = self.steer(nearest_node, random_node)

            # check new node collide with obstacle area
            if new_node is None:
                continue

            # check new node in area
            in_area_flag = True
            for j in range(self.dimension):
                if new_node.config_point[j] + self.robot_radius > self.space_limits[j] or \
                        new_node.config_point[j] - self.robot_radius < 0:
                    in_area_flag = False
            if in_area_flag is False:
                continue

            # check new node collide with conflict area
            conflict = False
            for conflict_node in self.conflict_node_list:
                if new_node.time == conflict_node.time and \
                        Utils.calculate_space_distance(new_node, conflict_node) < self.robot_radius * 2:
                    conflict = True
                    break
            if conflict:
                continue

            # insert new node to node list
            self.node_list.append(new_node)

            # choose parent
            lower_neighbor_nodes = self.get_lower_neighbor_nodes(new_node)
            self.choose_parent(new_node, lower_neighbor_nodes)

            # rewire
            higher_neighbor_nodes = self.get_higher_neighbor_nodes(new_node)
            self.rewire(new_node, higher_neighbor_nodes)

            if new_node.time + 1 >= self.max_time:
                self.max_time += 1

            if self.animation and i % 10 == 0:
                self.draw_nodes_edge_3d_graph()

            if self.is_near_goal(new_node):
                goal_node = self.steer(new_node, self.goal_node)
                if goal_node is not None:
                    goal_node.parent = new_node
                    new_node.add_child(goal_node)
                    goal_node.space_cost = new_node.space_cost + Utils.calculate_space_distance(new_node, goal_node)
                    goal_node.time_cost = new_node.time_cost + Utils.calculate_time_distance(new_node, goal_node)
                    goal_node.space_time_cost = new_node.space_time_cost + \
                                                Utils.calculate_space_time_distance(new_node, goal_node,
                                                                                    self.lambda_factor)
                    if self.last_node is None or goal_node.space_time_cost < self.last_node.space_time_cost:
                        if self.last_node is not None:
                            self.node_list.remove(self.last_node)
                        self.last_node = goal_node
                        self.node_list.append(goal_node)

        path = self.generate_path()
        self.space_cost = self.last_node.space_cost
        self.time_cost = self.last_node.time_cost
        self.space_time_cost = self.last_node.space_time_cost
        if self.draw_result:
            self.draw_path_3d_graph(path)
        return path

    # TODO : implemented
    def check_agent_collision(self, from_node: Node, to_node: Node):
        space_distance = Utils.calculate_space_distance(from_node, to_node)

        num_steps = math.ceil(space_distance / self.robot_radius)

        for i in range(num_steps):
            t = i / num_steps
            x = from_node.config_point[0] * (1 - t) + to_node.config_point[0] * t
            y = from_node.config_point[1] * (1 - t) + to_node.config_point[1] * t

            for obstacle in self.obstacles:
                if obstacle.check_agent_collision(x, y, self.robot_radius):
                    return True
        return False

    # TODO : Implemented
    def get_random_state(self):
        random_vector = list()
        for i in range(self.dimension):
            random_vector.append(random.uniform(0, self.space_limits[i]))
        random_time = random.randint(1, self.max_time)
        return random_vector, random_time

    # TODO : Implemented
    def get_nearest_node(self, random_node):
        nearest_node = None
        min_space_time_distance = float("inf")
        for node in self.node_list:
            if node.is_invalid or node == self.last_node:
                continue
            if random_node.time <= node.time:
                continue

            space_time_distance = Utils.calculate_space_time_distance(random_node, node, self.lambda_factor)
            if space_time_distance < min_space_time_distance:
                nearest_node = node
                min_space_time_distance = space_time_distance
        return nearest_node

    # TODO : Implemented
    def steer(self, from_node: Node, to_node: Node):
        expand_distance = Utils.calculate_space_distance(from_node, to_node)

        if expand_distance > self.max_expand_dis:
            expand_distance = self.max_expand_dis

        # TODO it should be updated for n-dimension
        theta = math.atan2(to_node.config_point[1] - from_node.config_point[1],
                           to_node.config_point[0] - from_node.config_point[0])
        new_config_vector = [
            from_node.config_point[0] + expand_distance * math.cos(theta),
            from_node.config_point[1] + expand_distance * math.sin(theta)
        ]

        new_node = Node(new_config_vector, from_node.time + 1)

        if self.check_agent_collision(from_node, new_node):
            return None

        return new_node

    # TODO : Implemented
    def is_near_goal(self, node: Node):
        return Utils.calculate_space_distance(node, self.goal_node) <= self.max_expand_dis

    # TODO : Implemented
    def get_lower_neighbor_nodes(self, center_node):
        lower_neighbor_nodes = []
        for node in self.node_list:
            if node.is_invalid or node == self.last_node:
                continue
            if center_node.time - node.time != 1:
                continue

            space_distance = Utils.calculate_space_distance(center_node, node)
            space_time_distance = Utils.calculate_space_time_distance(center_node, node, self.lambda_factor)

            if space_time_distance <= self.neighbor_radius and space_distance <= self.max_expand_dis + 0.01:
                lower_neighbor_nodes.append(node)

        return lower_neighbor_nodes

    # TODO : Implemented
    def get_higher_neighbor_nodes(self, center_node):
        higher_neighbor_nodes = []
        for node in self.node_list:
            if node.is_invalid or node == self.last_node:
                continue
            if center_node.time - node.time != -1:
                continue

            space_distance = Utils.calculate_space_distance(center_node, node)
            space_time_distance = Utils.calculate_space_time_distance(center_node, node, self.lambda_factor)

            if space_time_distance <= self.neighbor_radius and space_distance <= self.max_expand_dis + 0.01:
                higher_neighbor_nodes.append(node)

        return higher_neighbor_nodes

    # TODO : Implemented
    def choose_parent(self, new_node: Node, lower_neighbor_nodes: list):
        parent_node = None
        min_space_time_cost = float("inf")

        for lower_neighbor_node in lower_neighbor_nodes:
            new_space_time_cost = lower_neighbor_node.space_time_cost + \
                                  Utils.calculate_space_time_distance(lower_neighbor_node, new_node, self.lambda_factor)
            if new_space_time_cost < min_space_time_cost and not self.check_agent_collision(lower_neighbor_node,
                                                                                            new_node):
                parent_node = lower_neighbor_node
                min_space_time_cost = new_space_time_cost
        new_node.parent = parent_node
        parent_node.add_child(new_node)
        new_node.space_cost = parent_node.space_cost + Utils.calculate_space_distance(parent_node, new_node)
        new_node.time_cost = parent_node.time_cost + Utils.calculate_time_distance(parent_node, new_node)
        new_node.space_time_cost = min_space_time_cost

    # TODO : Implemented
    def rewire(self, new_node: Node, higher_neighbor_nodes: list):
        for higher_neighbor_node in higher_neighbor_nodes:
            new_space_time_cost = new_node.space_time_cost + \
                                  Utils.calculate_space_time_distance(new_node, higher_neighbor_node,
                                                                      self.lambda_factor)
            if new_space_time_cost < higher_neighbor_node.space_time_cost and not self.check_agent_collision(new_node,
                                                                                                             higher_neighbor_node):
                higher_neighbor_node.parent.remove_child(higher_neighbor_node)
                higher_neighbor_node.parent = new_node
                new_node.add_child(higher_neighbor_node)
                higher_neighbor_node.space_cost = new_node.space_cost + Utils.calculate_space_distance(new_node,
                                                                                                       higher_neighbor_node)
                higher_neighbor_node.time_cost = new_node.time_cost + Utils.calculate_time_distance(new_node,
                                                                                                    higher_neighbor_node)
                higher_neighbor_node.space_time_cost = new_space_time_cost

    # TODO : Implemented
    def generate_path(self):
        path = []
        current_node = self.last_node
        while current_node is not None:
            path.append(current_node)
            current_node = current_node.parent
        path.reverse()
        return path

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

    def draw_nodes_edge_3d_graph(self):
        ax.cla()
        ax.set_xlim3d(0, self.space_limits[0])
        ax.set_ylim3d(0, self.space_limits[1])
        ax.set_zlim3d(0, self.max_time)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('T')
        ax.set_title('Low Level of ST-CBS')
        for node in self.node_list:
            if node.parent is not None:
                x = [node.config_point[0], node.parent.config_point[0]]
                y = [node.config_point[1], node.parent.config_point[1]]
                z = [node.time, node.parent.time]
                if node.is_invalid:
                    ax.plot(x, y, z, color='red')
                elif node.is_conflict:
                    ax.plot(x, y, z, color='blue')
                else:
                    ax.plot(x, y, z, color='black')
        ax.scatter(self.start_node.config_point[0], self.start_node.config_point[1], self.start_node.time, color='green')
        ax.scatter(self.goal_node.config_point[0], self.goal_node.config_point[1], self.goal_node.time, color='red')
        for obstacle in self.obstacles:
            # Rectangle Obstacle
            if type(obstacle) == RectangleObstacle:
                cube_faces = self.create_cube(obstacle.x, obstacle.y, obstacle.width, obstacle.height, self.max_time)
                face_collection = Poly3DCollection(cube_faces, facecolor='b', alpha=0.01, linewidths=1, edgecolors='k')
                ax.add_collection3d(face_collection)
        plt.pause(0.1)

    def draw_path_3d_graph(self, path):
        ax.cla()
        ax.set_xlim3d(0, self.space_limits[0])
        ax.set_ylim3d(0, self.space_limits[1])
        ax.set_zlim3d(0, self.max_time)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('T')
        ax.set_title('Low Level of ST-CBS')
        for node in path:
            if node.parent is not None:
                x = [node.config_point[0], node.parent.config_point[0]]
                y = [node.config_point[1], node.parent.config_point[1]]
                z = [node.time, node.parent.time]
                ax.plot(x, y, z, color='red')
        ax.scatter(self.start_node.config_point[0], self.start_node.config_point[1], self.start_node.time, color='green')
        ax.scatter(self.goal_node.config_point[0], self.goal_node.config_point[1], self.goal_node.time, color='red')
        for obstacle in self.obstacles:
            # Rectangle Obstacle
            if type(obstacle) == RectangleObstacle:
                cube_faces = self.create_cube(obstacle.x, obstacle.y, obstacle.width, obstacle.height, self.max_time)
                face_collection = Poly3DCollection(cube_faces, facecolor='b', alpha=0.1, linewidths=1, edgecolors='k')
                ax.add_collection3d(face_collection)

        plt.show()


if __name__ == '__main__':
    obstacles = [
        RectangleObstacle(5, 5, 2, 2),
    ]
    space_time_rrt = USTRRRTstar(
        start_point=[2, 2],
        goal_point=[8, 8],
        dimension=2,
        space_limits=[10, 10],
        obstalces=[
            RectangleObstacle(5, 5, 2, 2),
        ],
        robot_radius=1.0,
        lambda_factor=0.5,
        max_expand_dis=3,
        neighbor_radius=5,
        max_iter=500,
    )
    path = space_time_rrt.planning()
    print("space cost : ", space_time_rrt.space_cost)
    print("time cost : ", space_time_rrt.time_cost)
    print("space time cost : ", space_time_rrt.space_time_cost)
    print("path : ")
    for node in path:
        print(f"x: {node.config_point[0]}, y: {node.config_point[1]}, t: {node.time}")

