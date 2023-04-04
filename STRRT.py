import math
import random

import matplotlib.pyplot as plt
import numpy as np


class Node:
    def __init__(self, x, y, t=0, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.t = t


class CircleObstacle:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def is_collide(self, circle_x, circle_y, circle_r):
        distance = math.sqrt((circle_x - self.x) ** 2 + (circle_y - self.y) ** 2)
        if distance <= (circle_r + self.r):
            return True
        else:
            return False


class RectangleObstacle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def is_collide(self, circle_x, circle_y, circle_r):
        if (circle_x + circle_r < self.x - self.width / 2) or \
                (circle_x - circle_r > self.x + self.width / 2) or \
                (circle_y + circle_r < self.y - self.height / 2) or \
                (circle_y - circle_r > self.y + self.height / 2):
            return False
        else:
            return True


class SpaceTimeRRT:
    def __init__(self, start, goal, width, height, robot_radius, lambda_factor, expand_dis, obstacles):
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

        # set nodes
        self.nodes = []

        # set figure
        self.animation = False
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def planning(self):
        self.nodes.append(self.start)

        while True:
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(rand_node)
            new_node = self.steer(nearest_node, rand_node)
            if self.is_collision(new_node, self.obstacles):
                continue
            new_node.parent = nearest_node
            self.nodes.append(new_node)
            if new_node.t + 1 > self.max_time:
                self.max_time = new_node.t + 1

            if self.animation:
                self.draw_nodes_edge_3d_graph()

            if self.is_near_goal(new_node):
                goal_node = self.steer(new_node, self.goal)
                if self.is_collision(goal_node, self.obstacles):
                    continue
                goal_node.parent = new_node
                goal_node.t = new_node.t + 1
                self.nodes.append(goal_node)
                break

        path = self.get_final_path()
        self.draw_path_3d_graph(path)
        return path

    def is_collision(self, node, obstacles):
        for obstacle in obstacles:
            if obstacle.is_collide(node.x, node.y, self.robot_radius):
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
        dlist = [(self.get_space_time_distance(node, rand_node), node) if node.t < rand_node.t else (float('inf'), node)
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
        last_node = self.nodes[-1]
        while last_node is not None:
            path.append(last_node)
            last_node = last_node.parent
        path.reverse()
        return path

    def draw_nodes_edge_3d_graph(self):
        self.ax.cla()
        self.ax.set_xlim3d(0, self.width)
        self.ax.set_ylim3d(0, self.height)
        self.ax.set_zlim3d(0, self.max_time)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('T')
        self.ax.set_title('Space-Time RRT')
        for node in self.nodes:
            if node.parent is not None:
                x = [node.x, node.parent.x]
                y = [node.y, node.parent.y]
                z = [node.t, node.parent.t]
                self.ax.plot(x, y, z, color='black')
        self.ax.scatter(self.start.x, self.start.y, self.start.t, color='green')
        self.ax.scatter(self.goal.x, self.goal.y, self.goal.t, color='red')
        for obstacle in self.obstacles:
            u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
            x = obstacle.x + obstacle.r * np.cos(u) * np.sin(v)
            y = obstacle.y + obstacle.r * np.sin(u) * np.sin(v)
            z = obstacle.r * np.cos(v)
            self.ax.plot_wireframe(x, y, z, color="blue")

        plt.pause(0.01)

    def draw_path_3d_graph(self, path):
        self.ax.cla()
        self.ax.set_xlim3d(0, self.width)
        self.ax.set_ylim3d(0, self.height)
        self.ax.set_zlim3d(0, self.max_time)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('T')
        self.ax.set_title('Space-Time RRT')
        for node in path:
            if node.parent is not None:
                x = [node.x, node.parent.x]
                y = [node.y, node.parent.y]
                z = [node.t, node.parent.t]
                self.ax.plot(x, y, z, color='red')
        self.ax.scatter(path[0].x, path[0].y, path[0].t, color='green')
        self.ax.scatter(path[-1].x, path[-1].y, path[-1].t, color='red')
        for obstacle in self.obstacles:
            u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
            x = obstacle.x + obstacle.r * np.cos(u) * np.sin(v)
            y = obstacle.y + obstacle.r * np.sin(u) * np.sin(v)
            z = obstacle.r * np.cos(v)
            self.ax.plot_wireframe(x, y, z, color="blue")

        plt.show()


if __name__ == '__main__':
    start = (0.0, 0.0)
    goal = (8.0, 8.0)
    obstacles = [
        CircleObstacle(5.0, 5.0, 1.0),
    ]
    space_time_rrt = SpaceTimeRRT(start=start, goal=goal, width=10.0, height=10.0, robot_radius=1,
                                  lambda_factor=0.5, expand_dis=1, obstacles=obstacles)
    path = space_time_rrt.planning()
    for node in path:
        print(node.x, node.y, node.t)
