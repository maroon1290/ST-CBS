import math
import random
import time
from abc import *
from collections import deque
from shapely.geometry import Point, box

import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# fig = plt.figure(figsize=(10, 10))
# ax = fig.add_subplot(111, projection='3d')
# ax.view_init(elev=30., azim=120)


class Node:
    def __init__(self, x, y, t=0, parent=None):
        self.x = x
        self.y = y
        self.t = t
        self.parent = parent
        self.children = deque()
        self.is_valid = True
        self.space_time_cost = None
        self.key = self.generate_key(x, y, t)

    @staticmethod
    def generate_key(x, y, t):
        return str(x) + '_' + str(y) + '_' + str(t)

    def space_distance(self, other):
        return math.hypot(other.x - self.x, other.y - self.y)

    def time_distance(self, other):
        return abs(other.t - self.t)


def linear_interpolate(from_node, to_node, radius):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    d = math.hypot(dx, dy)
    theta = math.atan2(dy, dx)
    interpolated_x = [from_node.x]
    interpolated_y = [from_node.y]
    n_expand = math.floor(d / radius)
    for _ in range(n_expand):
        interpolated_x.append(interpolated_x[-1] + radius * math.cos(theta))
        interpolated_y.append(interpolated_y[-1] + radius * math.sin(theta))

    d = math.hypot(to_node.x - interpolated_x[-1], to_node.y - interpolated_y[-1])
    if d <= radius:
        interpolated_x.append(to_node.x)
        interpolated_y.append(to_node.y)
    return interpolated_x, interpolated_y


class ObstacleBase(metaclass=ABCMeta):
    @abstractmethod
    def is_collide(self, from_node, to_node, radius):
        pass


class CircleObstacle(ObstacleBase):
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def is_collide(self, from_node, to_node, radius):
        x, y = linear_interpolate(from_node, to_node, radius)
        for i in range(len(x)):
            distance = math.sqrt((x[i] - self.x) ** 2 + (y[i] - self.y) ** 2)
            if distance <= (radius + self.r):
                return True
        return False


class RectangleObstacle(ObstacleBase):
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def is_collide(self, from_node, to_node, radius):
        x, y = linear_interpolate(from_node, to_node, radius)
        for i in range(len(x)):
            circle = Point(x[i], y[i]).buffer(radius)
            rectangle = box(self.x - self.width / 2.0, self.y - self.height / 2.0, self.x + self.width / 2.0,
                            self.y + self.height / 2.0)

            if rectangle.intersects(circle):
                return True
        return False


class USTRRRTstar:
    def __init__(self, start, goal, width, height, robot_radius, lambda_factor, expand_dis, obstacles, near_radius, max_iter):
        # set start and goal
        self.start = Node(start[0], start[1])
        self.start.space_time_cost = 0
        self.goal = Node(goal[0], goal[1])
        self.goal.space_time_cost = float("inf")

        # set map size
        self.width = width
        self.height = height
        self.max_time = 1
        self.obstacles = obstacles

        # set parameters
        self.robot_radius = robot_radius
        self.lambda_factor = lambda_factor
        self.expand_dis = expand_dis
        self.max_iter = max_iter

        # set nodes
        self.node_list = set()
        self.node_list.add(self.start)
        self.near_radius = near_radius
        self.last_node = None

        # set figure
        self.animation = False
        self.draw_result = False

    def planning(self):
        self.last_node = Node(0, 0, 0, None)
        self.last_node.space_time_cost = float("inf")
        i = 0
        while True:
            i += 1
            if i > self.max_iter and self.last_node.space_time_cost != float("inf"):
                break
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(rand_node)
            new_node = self.steer(nearest_node, rand_node)

            # check if new node is out of width and height
            right_of_new_node = new_node.x + self.robot_radius
            left_of_new_node = new_node.x - self.robot_radius
            top_of_new_node = new_node.y + self.robot_radius
            bottom_of_new_node = new_node.y - self.robot_radius
            if right_of_new_node > self.width or left_of_new_node < 0 or top_of_new_node > self.height or bottom_of_new_node < 0:
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
                    goal_node.space_time_cost = new_node.space_time_cost + self.get_space_time_distance(new_node, goal_node)

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

    def get_random_node(self):
        return Node(
            np.random.uniform(0, self.width),
            np.random.uniform(0, self.height),
            np.random.uniform(1, self.max_time))

    def get_nearest_node(self, rand_node):
        return min(self.node_list, key=lambda node: self.get_space_time_distance(rand_node, node) if (rand_node.t > node.t and node.is_valid) else float('inf'))

    def steer(self, from_node: Node, to_node: Node):
        d = from_node.space_distance(to_node)
        if d > self.expand_dis:
            d = self.expand_dis
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node = Node(
            from_node.x + d * math.cos(theta),
            from_node.y + d * math.sin(theta),
            from_node.t + 1
        )
        new_node.parent = from_node
        new_node.space_time_cost = from_node.space_time_cost + self.get_space_time_distance(from_node, new_node)
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
            if nearby_node.t - new_node.t != 1:
                continue
            potential_cost = nearby_node.space_time_cost + self.get_space_time_distance(new_node, nearby_node)
            if potential_cost < nearby_node.space_time_cost and not self.is_collide(new_node, nearby_node, self.obstacles):
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
        ax.set_title('Low Level of MARRF')
        for node in self.node_list:
            if node.parent is not None:
                x = [node.x, node.parent.x]
                y = [node.y, node.parent.y]
                z = [node.t, node.parent.t]
                if node.is_valid:
                    ax.plot(x, y, z, color='black')
                else:
                    ax.plot(x, y, z, color='red')
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
                face_collection = Poly3DCollection(cube_faces, facecolor='b', alpha=0.1, linewidths=1, edgecolors='k')
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
        ax.set_title('Low Level of MARRF')
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
    start = [1, 2.5]
    goal = [4, 2.5]
    obstacles = [
        RectangleObstacle(1, 4, 2, 2),
        RectangleObstacle(4, 4, 2, 2),
        RectangleObstacle(2.5, 1, 5, 2),
    ]
    space_time_rrt = USTRRRTstar(start=start, goal=goal, width=5.0, height=5.0, robot_radius=0.4,
                                  lambda_factor=0.5, expand_dis=0.5, obstacles=obstacles, near_radius=5.0, max_iter=500)
    space_cost, time_cost, space_time_cost, path = space_time_rrt.planning()
    for node in path:
        print(node.x, node.y, node.t)
    print(space_cost, time_cost, space_time_cost)
