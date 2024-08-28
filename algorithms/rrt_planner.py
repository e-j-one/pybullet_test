from typing import Tuple, Optional, List

import numpy as np
import matplotlib.pyplot as plt

from algorithms.path_planner import PathPlanner
from utils.types import RobotState
from utils.rrt_trees import RrtTree

# from planners.utils.line_algorithm import line_algorithm
# from planners.trees.rrt_trees import RrtTree


class RrtPlanner(PathPlanner):
    def __init__(self, config):
        super().__init__(config)

    def set_env(
        self,
        start_state: RobotState,
        goal_state: RobotState,
        obstacle_positions: List[Tuple[float, float, float]],
        obstacle_dimensions: List[Tuple[float, float, float]],
    ):
        super().set_env(
            start_state, goal_state, obstacle_positions, obstacle_dimensions
        )

    def plan_path(self) -> List[RobotState]:
        """
        Plan a path from start to goal using RRT algorithm

        1. Initialize tree
        2. Sample random point
        3. Find nearest node in the tree
        4. Drive from nearest node to random point
        5. Check collision
            If collision, go to 2
        6. Add new node to the tree
        7. Repeat 2-6 until goal is reached or max_iter is reached
        """

        self._initialize_tree()

        for sample_iter in range(self.max_iter):
            pass

        path = None

        return path, sample_iter

        # Initialize the tree with the start node
        self.tree.reset_tree()
        self.tree.add_root(start_pos)

        path_found = False

        for sample_iter in range(self.max_iter):
            if sample_iter % 1000 == 0 and print_iter:
                print("sample_iter: ", sample_iter)

            random_point = self._sample_point(goal_pos)

            nearest_node = self.tree.find_nearest_node(random_point)
            nearest_node_pos = nearest_node.get_pos()

            new_node_pos = self._drive(nearest_node_pos, random_point)

            if self.tree.check_if_pos_in_tree(new_node_pos):
                continue

            if self._check_collision_between_pos(nearest_node_pos, new_node_pos):
                continue

            new_node_idx = self.tree.add_node(new_node_pos, nearest_node.get_idx())

            # Check if the goal is reached
            if self._check_goal_reached(new_node_pos, goal_pos):
                path_found = True
                if not self.tree.check_if_pos_in_tree(goal_pos):
                    self.tree.add_node(goal_pos, new_node_idx)
                break

        if path_found:
            # print(
            #     "Path found with number of nodes: {}, num_iter {}".format(
            #         self.tree.get_num_nodes(), sample_iter
            #     )
            # )
            self.path = self.tree.get_path_from_tree(goal_pos)
            return self.path, sample_iter
        else:
            print("Path not found")
            return None, sample_iter

    def _initialize_tree(self):
        self.tree = RrtTree()
