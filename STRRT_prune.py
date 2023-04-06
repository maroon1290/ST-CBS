import math
import random
import time
from abc import *

import matplotlib.pyplot as plt
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


class Node:
    def __init__(self, x, y, t=0, parent=None):
        self.x = x
        self.y = y
        self.t = t
        self.parent = parent
        self.children = []
        self.is_valid = True


def linear_interpolate(from_node, to_node, radius):
    euclidean_distance = math.hypot(to_node.x - from_node.x, to_node.y - from_node.y)
    step_size = round(euclidean_distance / radius)
    x = np.linspace(from_node.x, to_node.x, step_size)
    y = np.linspace(from_node.y, to_node.y, step_size)
    return x, y


class ObstacleBase(metaclass=ABCMeta):
    @abstractmethod
    def is_collide_discrete(self, circle_x, circle_y, circle_r):
        pass

    @abstractmethod
    def is_collide_continuous(self, from_node, to_node, radius):
        pass


class CircleObstacle(ObstacleBase):
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def is_collide_discrete(self, circle_x, circle_y, circle_r):
        distance = math.sqrt((circle_x - self.x) ** 2 + (circle_y - self.y) ** 2)
        if distance <= (circle_r + self.r):
            return True
        else:
            return False

    def is_collide_continuous(self, from_node, to_node, radius):
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

    def is_collide_discrete(self, circle_x, circle_y, circle_r):
        if (circle_x + circle_r < self.x - self.width / 2) or \
                (circle_x - circle_r > self.x + self.width / 2) or \
                (circle_y + circle_r < self.y - self.height / 2) or \
                (circle_y - circle_r > self.y + self.height / 2):
            return False
        else:
            return True

    def is_collide_continuous(self, from_node, to_node, radius):
        x, y = linear_interpolate(from_node, to_node, radius)
        for i in range(len(x)):
            if (x[i] + radius < self.x - self.width / 2) or \
                    (x[i] - radius > self.x + self.width / 2) or \
                    (y[i] + radius < self.y - self.height / 2) or \
                    (y[i] - radius > self.y + self.height / 2):
                continue
            else:
                return True


class SpaceTimeRRT:
    def __init__(self, start, goal, width, height, robot_radius, lambda_factor, expand_dis, obstacles, collision_check_type):
        # set start and goal
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])

        # set map size
        self.width = width
        self.height = height
        self.max_time = 1
        self.obstacles = obstacles

        # set parameters
        self.robot_radius = robot_radius
        self.lambda_factor = lambda_factor
        self.expand_dis = expand_dis
        self.collision_check_type = collision_check_type

        # set nodes
        self.nodes = set()
        self.nodes.add(self.start)
        self.last_node = None

        # set figure
        self.animation = False
        self.draw_result = False

    def planning(self):
        while True:
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(rand_node)
            new_node = self.steer(nearest_node, rand_node)

            if self.collision_check_type == "discrete" and self.is_collision_discrete(new_node, self.obstacles):
                continue
            elif self.collision_check_type == "continuous" and self.is_collision_continuous(nearest_node, new_node, self.obstacles):
                continue

            print("node num: ", len(self.nodes))

            new_node.parent = nearest_node
            nearest_node.children.append(new_node)
            self.nodes.add(new_node)
            if new_node.t + 1 > self.max_time:
                self.max_time = new_node.t + 1

            if self.animation:
                self.draw_nodes_edge_3d_graph()

            if self.is_near_goal(new_node):
                goal_node = self.steer(new_node, self.goal)
                if self.collision_check_type == "discrete" and self.is_collision_discrete(goal_node, self.obstacles):
                    continue
                elif self.collision_check_type == "continuous" and self.is_collision_continuous(new_node, goal_node, self.obstacles):
                    continue
                goal_node.parent = new_node
                new_node.children.append(goal_node)
                goal_node.t = new_node.t + 1
                self.nodes.add(goal_node)
                self.last_node = goal_node
                break

        path = self.get_final_path()
        cost = self.get_cost(path)
        if self.draw_result:
            self.draw_path_3d_graph(path)
        return cost, path

    def is_collision_discrete(self, node, obstacles):
        for obstacle in obstacles:
            if obstacle.is_collide_discrete(node.x, node.y, self.robot_radius):
                return True
        return False

    def is_collision_continuous(self, from_node, to_node, obstacles):
        for obstacle in obstacles:
            if obstacle.is_collide_continuous(from_node, to_node, self.robot_radius):
                return True
        return False

    @staticmethod
    def get_space_distance(node1, node2):
        space_difference = math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
        return space_difference

    @staticmethod
    def get_time_distance(node1, node2):
        time_difference = abs(node1.t - node2.t)
        return time_difference

    def get_space_time_distance(self, node1, node2):
        space_distance = self.get_space_distance(node1, node2)
        time_distance = self.get_time_distance(node1, node2)
        space_time_distance = self.lambda_factor * space_distance + (1 - self.lambda_factor) * time_distance
        return space_time_distance

    def get_random_node(self):
        x = random.uniform(0, self.width)
        y = random.uniform(0, self.height)
        t = random.randint(1, self.max_time)
        return Node(x, y, t)

    def get_nearest_node(self, rand_node):
        dlist = [(self.get_space_time_distance(node, rand_node), node) if node.t < rand_node.t and node.is_valid else (float('inf'), node)
                 for node in self.nodes]
        dlist.sort(key=lambda x: x[0])
        return dlist[0][1]

    def steer(self, from_node, to_node):
        d = self.get_space_distance(from_node, to_node)
        if d > self.expand_dis:
            d = self.expand_dis
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        x = from_node.x + d * math.cos(theta)
        y = from_node.y + d * math.sin(theta)
        t = from_node.t + 1
        return Node(x, y, t, from_node)

    def is_near_goal(self, node):
        d = self.get_space_distance(node, self.goal)
        return d <= self.expand_dis

    def get_final_path(self):
        path = []
        last_node = self.last_node
        while last_node is not None:
            path.append(last_node)
            last_node = last_node.parent
        path.reverse()
        return path

    def get_cost(self, path):
        cost = 0
        for i in range(len(path) - 1):
            cost += self.get_space_distance(path[i], path[i + 1])
        return cost

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
        ax.set_title('Space-Time RRT')
        for node in self.nodes:
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
        plt.pause(0.01)

    def draw_path_3d_graph(self, path):
        ax.cla()
        ax.set_xlim3d(0, self.width)
        ax.set_ylim3d(0, self.height)
        ax.set_zlim3d(0, self.max_time)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('T')
        ax.set_title('Space-Time RRT')
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

        plt.pause(0.1)


if __name__ == '__main__':
    start = (4, 10)
    goal = (16, 10)
    obstacles = [
        RectangleObstacle(10, 4, 20, 8),
        RectangleObstacle(4, 16, 8, 8),
        RectangleObstacle(10, 18, 4, 4),
        RectangleObstacle(16, 16, 8, 8),
    ]
    space_time_rrt = SpaceTimeRRT(start=start, goal=goal, width=20.0, height=20.0, robot_radius=1.5,
                                  lambda_factor=0.5, expand_dis=3.0, obstacles=obstacles, collision_check_type='discrete')
    cost, path = space_time_rrt.planning()
    for node in path:
        print(node.x, node.y, node.t)
    print('cost:', cost)
