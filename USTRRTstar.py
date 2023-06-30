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
                 space_limit: list,
                 obstalces: list,
                 robot_radius: float,
                 lambda_factor: float,
                 max_expand_dis: float,
                 neighbor_radius: float,
                 max_iter: int, ):
        self.start_point = start_point
        self.goal_point = goal_point
        self.dimension = dimension
        self.space_limit = space_limit
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
        self.goal_node = Node(goal_point, -1)
        self.last_node = None

        # set figure
        self.animation = True
        self.draw_result = False

    def planning(self):
        self.last_node = None
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
                if new_node.config_point[j] + self.robot_radius > self.space_limit[j] or \
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
                    self.node_list.append(goal_node)
                    self.last_node = goal_node
                    break

        path = self.generate_path()
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

            for obstacle in obstacles:
                if obstacle.check_agent_collision(x, y, self.robot_radius):
                    return True
        return False

    # TODO : Implemented
    def get_random_state(self):
        random_vector = list()
        for i in range(self.dimension):
            random_vector.append(random.uniform(0, self.space_limit[i]))
        random_time = random.randint(1, self.max_time)
        return random_vector, random_time

    # TODO : Implemented
    def get_nearest_node(self, random_node):
        nearest_node = None
        min_space_time_distance = float("inf")
        for node in self.node_list:
            if node.is_invalid:
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
            if node.is_invalid:
                continue
            if center_node.time - node.time != 1:
                continue

            space_distance = Utils.calculate_space_distance(center_node, node)
            space_time_distance = Utils.calculate_space_time_distance(center_node, node, self.lambda_factor)

            if space_time_distance <= self.neighbor_radius and space_distance <= self.max_expand_dis:
                lower_neighbor_nodes.append(node)

        return lower_neighbor_nodes

    # TODO : Implemented
    def get_higher_neighbor_nodes(self, center_node):
        higher_neighbor_nodes = []
        for node in self.node_list:
            if node.is_invalid:
                continue
            if center_node.time - node.time != -1:
                continue

            space_distance = Utils.calculate_space_distance(center_node, node)
            space_time_distance = Utils.calculate_space_time_distance(center_node, node, self.lambda_factor)

            if space_time_distance <= self.neighbor_radius and space_distance <= self.max_expand_dis:
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
        ax.set_xlim3d(0, self.width)
        ax.set_ylim3d(0, self.height)
        ax.set_zlim3d(0, self.max_time)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('T')
        ax.set_title('Low Level of ST-CBS')
        for node in self.node_list:
            if node.parent is not None:
                x = [node.x, node.parent.x]
                y = [node.y, node.parent.y]
                z = [node.t, node.parent.t]
                if node.is_valid is False:
                    ax.plot(x, y, z, color='red')
                elif node.is_infected:
                    print("blue")
                    ax.plot(x, y, z, color='blue')
                else:
                    ax.plot(x, y, z, color='black')
        ax.scatter(self.start.x, self.start.y, self.start.t, color='green')
        ax.scatter(self.goal.x, self.goal.y, self.goal.t, color='red')
        for obstacle in self.obstacles:
            # Circle Obstacle
            if type(obstacle) == CircleObstacle:
                u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
                x = obstacle.x + obstacle.r * np.cos(u) * np.sin(v)
                y = obstacle.y + obstacle.r * np.sin(u) * np.sin(v)
                z = obstacle.r * np.cos(v)
                ax.plot_wireframe(x, y, z, color="blue")

            # Rectangle Obstacle
            if type(obstacle) == RectangleObstacle:
                cube_faces = self.create_cube(obstacle.x, obstacle.y, obstacle.width, obstacle.height, self.max_time)
                face_collection = Poly3DCollection(cube_faces, facecolor='b', alpha=0.01, linewidths=1, edgecolors='k')
                ax.add_collection3d(face_collection)
        plt.pause(0.1)

    def draw_path_3d_graph(self, path):
        ax.cla()
        ax.set_xlim3d(0, self.width)
        ax.set_ylim3d(0, self.height)
        ax.set_zlim3d(0, self.max_time)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('T')
        ax.set_title('Low Level of ST-CBS')
        for node in path:
            if node.parent is not None:
                x = [node.x, node.parent.x]
                y = [node.y, node.parent.y]
                z = [node.t, node.parent.t]
                ax.plot(x, y, z, color='red')
        ax.scatter(path[0].x, path[0].y, path[0].t, color='green')
        ax.scatter(path[-1].x, path[-1].y, path[-1].t, color='red')
        for obstacle in self.obstacles:
            # Circle Obstacle
            if type(obstacle) == CircleObstacle:
                u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
                x = obstacle.x + obstacle.r * np.cos(u) * np.sin(v)
                y = obstacle.y + obstacle.r * np.sin(u) * np.sin(v)
                z = obstacle.r * np.cos(v)
                ax.plot_wireframe(x, y, z, color="blue")

            # Rectangle Obstacle
            if type(obstacle) == RectangleObstacle:
                cube_faces = self.create_cube(obstacle.x, obstacle.y, obstacle.width, obstacle.height, self.max_time)
                face_collection = Poly3DCollection(cube_faces, facecolor='b', alpha=0.1, linewidths=1, edgecolors='k')
                ax.add_collection3d(face_collection)

        plt.pause(1)


if __name__ == '__main__':
    start = [2, 2]
    goal = [8, 8]
    obstacles = [
        RectangleObstacle(5, 5, 2, 2),
    ]
    space_time_rrt = USTRRRTstar(start=start, goal=goal, width=10.0, height=10.0, robot_radius=1,
                                 lambda_factor=0.5, expand_dis=3, obstacles=obstacles, near_radius=3.0, max_iter=500)
    start_time = time.time()
    space_cost, time_cost, space_time_cost, path = space_time_rrt.planning()
    print(f"Time cost: {time.time() - start_time}s")
    for node in path:
        print(node.x, node.y, node.t)
    print(space_cost, time_cost, space_time_cost)
