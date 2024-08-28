from typing import List, Tuple, Optional

import numpy as np
from scipy.spatial import KDTree

from utils.types import RobotState
from algorithms.utils.rrt_nodes import RrtNode, RrtStarNode


class RrtTree:
    def __init__(self):
        self.nodes: List[RrtNode] = []
        self.robot_states_to_idx_map = {}

        self.kd_tree: KDTree = None

    def reset_tree(self):
        self.nodes: List[RrtNode] = []
        self.robot_states_to_idx_map = {}

        self.kd_tree: KDTree = None

    def add_node(self, robot_state: RobotState, parent_idx: Optional[int]) -> int:
        """
        Returns
        -------
        The index of the added node.
        """
        idx = len(self.nodes)
        node = RrtNode(idx=idx, robot_state=robot_state, parent_idx=parent_idx)
        self.nodes.append(node)
        if parent_idx is not None:
            self.nodes[parent_idx].add_child(idx)
        self.robot_states_to_idx_map[robot_state] = idx
        self.kd_tree = KDTree([node.get_robot_state() for node in self.nodes])
        return idx

    def add_root(self, robot_state: RobotState):
        assert len(self.nodes) == 0
        self.add_node(robot_state, None)

    def get_robot_state_of_node(self, idx: int) -> RobotState:
        return self.nodes[idx].get_robot_state()

    def find_nearest_node(self, robot_state: RobotState) -> RrtNode:
        distances, nearest_node_idx = self.kd_tree.query(np.array([robot_state]))
        return self.nodes[nearest_node_idx[0]]

    def check_if_robot_state_already_in_tree(self, robot_state: RobotState) -> bool:
        return robot_state in self.robot_states_to_idx_map

    def get_idx_from_robot_state(self, robot_state: RobotState) -> Optional[int]:
        if robot_state not in self.robot_states_to_idx_map:
            return -1
        return self.robot_states_to_idx_map[robot_state]

    def get_num_nodes(self):
        return len(self.nodes)

    def get_path_from_tree(self, goal_state: RobotState) -> Optional[List[RobotState]]:
        if goal_state not in self.robot_states_to_idx_map:
            return None
        path = [goal_state]
        goal_idx = self.robot_states_to_idx_map[goal_state]
        node_on_path_idx = self.nodes[goal_idx].get_parent_idx()
        while node_on_path_idx is not None:
            path.append(self.nodes[node_on_path_idx].get_robot_state())
            node_on_path_idx = self.nodes[node_on_path_idx].get_parent_idx()

        return path[::-1]

    def get_path_idx_from_tree(self, goal_state: RobotState) -> Optional[List[int]]:
        if goal_state not in self.robot_states_to_idx_map:
            return None
        path = [goal_idx]
        goal_idx = self.robot_states_to_idx_map[goal_state]
        node_on_path_idx = self.nodes[goal_idx].get_parent_idx()
        while node_on_path_idx is not None:
            path.append(node_on_path_idx)
            node_on_path_idx = self.nodes[node_on_path_idx].get_parent_idx()

        return path[::-1]


class RrtStarTree(RrtTree):
    def __init__(self, near_node_dist_trheshold: float):
        self.near_node_dist_trheshold = near_node_dist_trheshold

        self.nodes: List[RrtStarNode] = []
        self.robot_states_to_idx_map = {}

        self.kd_tree: KDTree = None

    def reset_tree(self):
        self.nodes: List[RrtStarNode] = []
        self.robot_states_to_idx_map = {}

        self.kd_tree: KDTree = None

    def add_node(
        self,
        robot_state: RobotState,
        parent_idx: int,
        cost: float,
        cost_from_parent: float,
    ) -> int:
        """
        Returns
        -------
        The index of the added node.
        """
        idx = len(self.nodes)
        node = RrtStarNode(
            idx=idx,
            robot_state=robot_state,
            parent_idx=parent_idx,
            cost=cost,
            cost_from_parent=cost_from_parent,
        )
        self.nodes.append(node)
        if parent_idx is not None:
            self.nodes[parent_idx].add_child(idx)
        self.robot_states_to_idx_map[robot_state] = idx
        self.kd_tree = KDTree([node.get_robot_state() for node in self.nodes])
        return idx

    def add_root(self, root_pos) -> int:
        assert len(self.nodes) == 0
        return self.add_node(root_pos, None, 0, 0)

    def propagate_cost_to_children(self, node_idx: int):
        for child_idx in self.nodes[node_idx].child_indexes:
            updated_cost = (
                self.nodes[node_idx].get_cost()
                + self.nodes[child_idx].get_cost_from_parent()
            )

            self.nodes[child_idx].update_cost(updated_cost)
            self.propagate_cost_to_children(child_idx)

    def find_near_nodes(self, new_robot_state: RobotState) -> List[int]:
        near_nodes_idx = self.kd_tree.query_ball_point(
            np.array([new_robot_state]), self.near_node_dist_trheshold
        )
        return near_nodes_idx[0]

    def get_cost_of_robot_state(self, robot_state: RobotState) -> float:
        if robot_state not in self.robot_states_to_idx_map:
            return float("inf")
        return self.nodes[self.robot_states_to_idx_map[robot_state]].get_cost()

    def get_cost_of_node_idx(self, idx: int) -> float:
        return self.nodes[idx].get_cost()

    def update_parent_and_cost(
        self,
        node_idx: int,
        new_parent_idx: int,
        new_cost: float,
        new_cost_from_parent: float,
    ):
        original_parent_idx = self.nodes[node_idx].get_parent_idx()
        self.nodes[original_parent_idx].remove_child(node_idx)
        self.nodes[node_idx].update_parent_and_cost(
            new_parent_idx=new_parent_idx,
            new_cost=new_cost,
            new_cost_from_parent=new_cost_from_parent,
        )
        self.nodes[new_parent_idx].add_child(node_idx)

    def find_optimal_parent_and_add_node_to_tree(
        self,
        new_robot_state: RobotState,
        collision_free_near_nodes_idx: List[int],
        cost_to_collision_free_near_nodes: List[float],
    ) -> Tuple[int, float]:
        optimal_parent = None
        min_cost = float("inf")
        cost_to_parent = None
        for iter_idx, near_node_idx in enumerate(collision_free_near_nodes_idx):
            cost_via_near_node = (
                self.nodes[near_node_idx].get_cost()
                + cost_to_collision_free_near_nodes[iter_idx]
            )
            if cost_via_near_node < min_cost:
                min_cost = cost_via_near_node
                cost_to_parent = cost_to_collision_free_near_nodes[iter_idx]
                optimal_parent = near_node_idx

        if optimal_parent is None:
            raise ValueError("No optimal parent found")

        return self.add_node(new_robot_state, optimal_parent, min_cost, cost_to_parent)
