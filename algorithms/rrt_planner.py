from typing import Tuple, Optional, List

import numpy as np
import matplotlib.pyplot as plt

from algorithms.path_planner import PathPlanner

# from planners.utils.line_algorithm import line_algorithm
# from planners.trees.rrt_trees import RrtTree


class RrtPlanner(PathPlanner):
    def __init__(self, config):
        super().__init__(config)

    def plan_path(self) -> List[List[float]]:
        """
        Plan a path from start to goal using RRT algorithm
        """

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
