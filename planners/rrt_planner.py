from typing import Tuple, Optional, List

import numpy as np
import matplotlib.pyplot as plt

from planners.path_planner import PathPlanner
from planners.utils.rrt_trees import RrtTree
from utils.types import RobotState

# from planners.utils.line_algorithm import line_algorithm
# from planners.trees.rrt_trees import RrtTree


class RrtPlanner(PathPlanner):
    def __init__(
        self,
        max_iter: int,
        robot_state_ranges: List[Tuple[float, float]],
        drive_dist: float,
        collision_check_step_size: float,
    ):
        """
        Parameters
        ----------
        max_iter: int
            Maximum number of iterations to run the algorithm
        robot_state_ranges: List[Tuple[float, float]]
            List of tuples of (min, max) values for each dimension of the robot state
        drive_dist: float
            Distance to drive from the nearest node to the sampled node
        collision_check_step_size: float
            Step size to check for collision between states
        """
        super().__init__(max_iter=max_iter, robot_state_ranges=robot_state_ranges)
        self.drive_dist = drive_dist
        self.collision_check_step_size = collision_check_step_size

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

    def _initialize_tree(self):
        """
        Reset the tree to start a new planning
        Reset tree and add the start state to the tree
        """
        self.tree = RrtTree()
        self.tree.add_root(self.start_state)

    def _sample_robot_state(self) -> RobotState:
        """
        Sample a random robot state within the self.robot_state_ranges
        """

        sampled_robot_state = []
        for state_range in self.robot_state_ranges:
            sampled_robot_state.append(
                np.random.uniform(state_range[0], state_range[1])
            )
        return tuple(sampled_robot_state)

    def _find_nearest_node(self, robot_state: RobotState) -> Tuple[int, RobotState]:
        nearest_node = self.tree.find_nearest_node(robot_state)
        return nearest_node.get_idx(), nearest_node.get_robot_state()

    def _drive(
        self, nearest_node_robot_state: RobotState, sampled_robot_state: RobotState
    ) -> RobotState:
        """
        Drive from nearest node to sampled node by a distance of self.drive_dist
        """

        nearest_node_robot_state_np = np.array(nearest_node_robot_state)
        sampled_robot_state_np = np.array(sampled_robot_state)
        diff = sampled_robot_state_np - nearest_node_robot_state_np

        dist_to_sampled_state = np.linalg.norm(diff)

        if dist_to_sampled_state < self.drive_dist:
            return sampled_robot_state
        if dist_to_sampled_state == 0:
            return nearest_node_robot_state

        drive_direction = diff / dist_to_sampled_state

        new_robot_state = (
            nearest_node_robot_state_np + self.drive_dist * drive_direction
        )

        # print("nearest node", nearest_node_robot_state)
        # print("sampled node", sampled_robot_state)
        # print("drived  node", tuple(new_robot_state))
        return tuple(new_robot_state)

    def _is_state_already_in_tree(self, robot_state: RobotState) -> bool:
        return self.tree.check_if_robot_state_already_in_tree(robot_state)

    def _is_collision_between_states(
        self, robot_state_i: RobotState, robot_state_j: RobotState
    ) -> bool:
        """
        Check if there is a collision between the states robot_state_i and robot_state_j
        Progresively sample points between the two states and check for collision
        """

        raise NotImplementedError

    def _is_goal_reached(new_robot_state):
        raise NotImplementedError

    def _add_goal_state_if_not_in_tree(self, parent_node_idx):
        if not self.tree.check_if_robot_state_already_in_tree(self.goal_state):
            self.tree.add_node(self.goal_state, parent_node_idx)

    def _get_path_to_goal(self) -> List[RobotState]:
        path = self.tree.get_path_from_tree(self.goal_state)
        if path is None:
            raise ValueError("Cannot find path to goal")
        return path

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

        path_found = False

        for sample_iter in range(self.max_iter):
            random_robot_state = self._sample_robot_state()

            nearest_node_idx, nearest_node_robot_state = self._find_nearest_node(
                random_robot_state
            )

            new_robot_state = self._drive(nearest_node_robot_state, random_robot_state)

            if self._is_state_already_in_tree(new_robot_state):
                continue

            if self._is_collision_between_states(
                nearest_node_robot_state, new_robot_state
            ):
                continue

            new_node_idx = self.tree.add_node(new_robot_state, nearest_node_idx)

            if self._is_goal_reached(new_robot_state):
                self._add_goal_state_if_not_in_tree(parent_node_idx=new_node_idx)
                path_found = True
                break

        if not path_found:
            print("Path not found")
            return None, sample_iter

        return self._get_path_to_goal(), sample_iter
