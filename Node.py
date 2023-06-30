from __future__ import annotations


class Node:
    def __init__(self, config_point: list, time: int):
        self.config_point = config_point
        self.time = time
        self.parent = None
        self.children = set()
        self.is_invalid = False
        self.is_conflict = False
        self.space_cost = float("inf")
        self.time_cost = float("inf")
        self.space_time_cost = float("inf")

    def add_child(self, child: Node):
        self.children.add(child)

    def remove_child(self, child: Node):
        self.children.remove(child)
