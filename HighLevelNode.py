from __future__ import annotations
import Node


class HighLevelNode:
    def __init__(self):
        self.conflict_nodes = dict()
        self.space_costs = list()
        self.time_costs = list()
        self.space_time_costs = list()
        self.solution = list()
        self.sum_of_space_costs = 0
        self.sum_of_time_costs = 0
        self.sum_of_space_time_costs = 0

    def __lt__(self, other: HighLevelNode):
        return self.sum_of_space_time_costs < other.sum_of_space_time_costs

    def calculate_sum_of_space_costs(self):
        self.sum_of_space_costs = sum(self.space_costs)

    def calculate_sum_of_time_costs(self):
        self.sum_of_time_costs = sum(self.time_costs)

    def calculate_sum_of_space_time_costs(self):
        self.sum_of_space_time_costs = sum(self.space_time_costs)

    def calculate_all_sum_of_costs(self):
        self.calculate_sum_of_space_costs()
        self.calculate_sum_of_time_costs()
        self.calculate_sum_of_space_time_costs()

    def add_conflict_node(self, agent: int, conflict_node: Node):
        self.conflict_nodes[agent] = conflict_node
