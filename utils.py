from Node import Node
import math


class Utils:
    @staticmethod
    def calculate_space_distance(from_node: Node, to_node: Node):
        return math.hypot(from_node.config_point[0] - to_node.config_point[0], from_node.config_point[1] - to_node.config_point[1])

    @staticmethod
    def calculate_time_distance(from_node: Node, to_node: Node):
        return abs(from_node.time - to_node.time)

    @staticmethod
    def calculate_space_time_distance(from_node: Node, to_node: Node, lambda_factor: float):
        space_distance = Utils.calculate_space_distance(from_node, to_node)
        time_distance = Utils.calculate_time_distance(from_node, to_node)
        return lambda_factor * space_distance + (1 - lambda_factor) * time_distance

    @staticmethod
    def calculate_space_distance_xy(x1: float, y1: float, x2: float, y2: float):
        return math.hypot(x1 - x2, y1 - y2)