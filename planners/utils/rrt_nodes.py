from typing import List, Tuple, Optional


from utils.types import RobotState


class RrtNode:
    def __init__(self, idx: int, robot_state: RobotState, parent_idx: int):
        self.idx = idx
        self.robot_state = robot_state
        self.parent_idx = parent_idx
        self.child_indexes = []

    def add_child(self, child_idx: int):
        self.child_indexes.append(child_idx)

    def get_robot_state(self) -> RobotState:
        return self.robot_state

    def get_parent_idx(self) -> int:
        return self.parent_idx

    def get_idx(self) -> int:
        return self.idx


class RrtStarNode(RrtNode):
    def __init__(
        self,
        idx: int,
        robot_state: RobotState,
        parent_idx: int,
        cost: float,
        cost_from_parent: float,
    ):
        self.idx = idx
        self.robot_state = robot_state
        self.parent_idx = parent_idx
        self.cost = cost
        self.cost_from_parent = cost_from_parent
        self.child_indexes = []

    def update_parent_and_cost(
        self, new_parent_idx: int, new_cost: float, new_cost_from_parent: float
    ):
        self.parent_idx = new_parent_idx
        self.cost = new_cost
        self.cost_from_parent = new_cost_from_parent

    def update_cost(self, updated_cost):
        self.cost = updated_cost

    def remove_child(self, child_idx):
        self.child_indexes.remove(child_idx)

    def get_cost_from_parent(self) -> float:
        return self.cost_from_parent

    def get_cost(self) -> float:
        return self.cost
