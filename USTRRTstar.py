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

            # check if new node is out of width and height
            right_of_new_node = new_node.x + self.robot_radius
            left_of_new_node = new_node.x - self.robot_radius
            top_of_new_node = new_node.y + self.robot_radius
            bottom_of_new_node = new_node.y - self.robot_radius
            if right_of_new_node > self.width or left_of_new_node < 0 or top_of_new_node > self.height or bottom_of_new_node < 0:
                continue

            invalid_flag = False
            # check collide with invalid nodes
            for invalid_node in self.node_list:
                if invalid_node.is_valid:
                    continue

                if invalid_node.t == new_node.t and invalid_node.space_distance(new_node) <= self.robot_radius:
                    invalid_flag = True
                    break

            if invalid_flag:
                continue

            # check collision
            if not self.is_collide(nearest_node, new_node, self.obstacles):
                # connect new node to tree
                self.node_list.add(new_node)

                # get neighbor nodes
                neighbor_nodes = self.get_neighbor_nodes(new_node, self.near_radius)

                # rewire
                self.choose_parent(new_node, neighbor_nodes)
                self.rewire(new_node, neighbor_nodes)

                if new_node.t + 1 > self.max_time:
                    self.max_time = new_node.t + 1

                if self.animation and i % 10 == 0:
                    self.draw_nodes_edge_3d_graph()

                if self.is_near_goal(new_node):
                    goal_node = self.steer(new_node, self.goal)
                    if self.is_collide(new_node, goal_node, self.obstacles):
                        continue
                    goal_node.t = new_node.t + 1
                    goal_node.parent = new_node
                    new_node.children.append(goal_node)
                    goal_node.space_time_cost = new_node.space_time_cost + self.get_space_time_distance(new_node,
                                                                                                        goal_node)

                    if self.last_node.space_time_cost > goal_node.space_time_cost:
                        self.node_list.add(goal_node)
                        self.last_node = goal_node

        path = self.get_final_path()
        space_cost = self.get_space_cost(path)
        time_cost = self.get_time_cost(path)
        space_time_cost = self.get_space_time_cost(path)
        if self.draw_result:
            self.draw_path_3d_graph(path)
        return space_cost, time_cost, space_time_cost, path

    def is_collide(self, from_node, to_node, obstacles):
        for obstacle in obstacles:
            if obstacle.is_collide(from_node, to_node, self.robot_radius):
                return True
        return False

    def get_space_time_distance(self, node1: Node, node2: Node):
        space_distance = node1.space_distance(node2)
        time_distance = node1.time_distance(node2)
        space_time_distance = self.lambda_factor * space_distance + (1 - self.lambda_factor) * time_distance
        return space_time_distance

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


        # d = from_node.space_distance(to_node)
        # if d > self.expand_dis:
        #     d = self.expand_dis
        # theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        # new_node = Node(
        #     from_node.x + d * math.cos(theta),
        #     from_node.y + d * math.sin(theta),
        #     from_node.t + 1
        # )
        # new_node.parent = from_node
        # new_node.space_time_cost = from_node.space_time_cost + self.get_space_time_distance(from_node, new_node)
        return new_node

    def is_near_goal(self, node: Node):
        d = node.space_distance(self.goal)
        return d <= self.expand_dis

    def get_neighbor_nodes(self, center_node, radius):
        nodes_in_radius = []
        for node in self.node_list:
            if node == center_node:
                continue
            if self.get_space_time_distance(center_node, node) <= radius:
                nodes_in_radius.append(node)
        return nodes_in_radius

    def choose_parent(self, new_node, nearby_nodes):
        for nearby_node in nearby_nodes:
            if nearby_node == new_node.parent:
                continue
            if nearby_node.is_valid is False:
                continue
            if nearby_node.is_infected:
                continue
            if new_node.t - nearby_node.t != 1:
                continue
            potential_cost = nearby_node.space_time_cost + self.get_space_time_distance(new_node, nearby_node)
            if potential_cost < new_node.space_time_cost and not self.is_collide(new_node, nearby_node, self.obstacles):
                new_node.parent = nearby_node
                new_node.space_time_cost = potential_cost
        new_node.parent.children.append(new_node)

    def rewire(self, new_node, nearby_nodes):
        for nearby_node in nearby_nodes:
            if nearby_node == new_node.parent:
                continue
            if nearby_node.is_valid is False:
                continue
            if nearby_node.is_infected:
                continue
            if nearby_node.t - new_node.t != 1:
                continue
            potential_cost = nearby_node.space_time_cost + self.get_space_time_distance(new_node, nearby_node)
            if potential_cost < nearby_node.space_time_cost and not self.is_collide(new_node, nearby_node,
                                                                                    self.obstacles):
                nearby_node.parent.children.remove(nearby_node)
                nearby_node.parent = new_node
                nearby_node.space_time_cost = potential_cost
                new_node.children.append(nearby_node)

    def get_final_path(self):
        path = []
        last_node = self.last_node
        while last_node is not None:
            path.append(last_node)
            last_node = last_node.parent
        path.reverse()
        return path

    @staticmethod
    def get_space_cost(path):
        space_cost = 0
        for i in range(len(path) - 1):
            space_cost += path[i].space_distance(path[i + 1])
        return space_cost

    def get_time_cost(self, path):
        time_cost = 0
        for i in range(len(path) - 1):
            time_cost += path[i].time_distance(path[i + 1])
        return time_cost

    def get_space_time_cost(self, path):
        space_time_cost = 0
        for i in range(len(path) - 1):
            space_time_cost += self.get_space_time_distance(path[i], path[i + 1])
        return space_time_cost

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
        CircleObstacle(5, 5, 2),
    ]
    space_time_rrt = USTRRRTstar(start=start, goal=goal, width=10.0, height=10.0, robot_radius=1,
                                 lambda_factor=0.5, expand_dis=3, obstacles=obstacles, near_radius=3.0, max_iter=500)
    start_time = time.time()
    space_cost, time_cost, space_time_cost, path = space_time_rrt.planning()
    print(f"Time cost: {time.time() - start_time}s")
    for node in path:
        print(node.x, node.y, node.t)
    print(space_cost, time_cost, space_time_cost)
