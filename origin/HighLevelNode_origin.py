from __future__ import annotations
from Node_origin import Node
from USTRRTstar_origin import USTRRRTstar


class HighLevelNode:
    def __init__(self):
        self.trees = list()
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

    def init_conflict_nodes(self, num_of_agents: int):
        for i in range(num_of_agents):
            self.conflict_nodes[i] = list()

    def add_tree(self, tree: USTRRRTstar):
        self.trees.append(tree)

    def add_space_cost(self, space_cost: float):
        self.space_costs.append(space_cost)

    def add_time_cost(self, time_cost: float):
        self.time_costs.append(time_cost)

    def add_space_time_cost(self, space_time_cost: float):
        self.space_time_costs.append(space_time_cost)

    def add_path_to_solution(self, path: list):
        self.solution.append(path)

    def add_conflict_node(self, agent: int, conflict_node: Node):
        if agent not in self.conflict_nodes:
            self.conflict_nodes[agent] = list()
        self.conflict_nodes[agent].append(conflict_node)

    def update_space_cost(self, agent: int, space_cost: float):
        self.space_costs[agent] = space_cost

    def update_time_cost(self, agent: int, time_cost: float):
        self.time_costs[agent] = time_cost

    def update_space_time_cost(self, agent: int, space_time_cost: float):
        self.space_time_costs[agent] = space_time_cost

    def update_path_in_solution(self, agent: int, path: list):
        self.solution[agent] = path
