from typing import Tuple, List, Callable

import numpy as np

from planners.rrt_planner import RrtPlanner
from planners.utils.rrt_trees import RrtStarTree
import planners.utils.geometry_utils as GeometryUtils

from utils.types import RobotState
import utils.plot_utils as PlotUtils


class RrtStarPlanner(RrtPlanner):
    def __init__(
        self,
        max_iter: int,
        joint_ids: List[int],
        robot_state_ranges: List[Tuple[float, float]],
        collision_check_step_size: float,
        goal_reached_threshold: float,
        drive_dist: float,
        goal_sample_rate: float,
        near_radius: float,
    ):
        """
        Parameters
        ----------
        max_iter: int
            Maximum number of iterations to run the algorithm
        joint_ids: List[int]
            List of joint ids to plan for
        robot_state_ranges: List[Tuple[float, float]]
            List of tuples of (min, max) values for each dimension of the robot state
        drive_dist: float
            Distance to drive from the nearest node to the sampled node
        collision_check_step_size: float
            Step size to check for collision between states
        near_radius: float
            Radius to find near nodes in the tree
        """
        super().__init__(
            max_iter=max_iter,
            joint_ids=joint_ids,
            robot_state_ranges=robot_state_ranges,
            collision_check_step_size=collision_check_step_size,
            goal_reached_threshold=goal_reached_threshold,
            drive_dist=drive_dist,
            goal_sample_rate=goal_sample_rate,
        )
        self.near_radius = near_radius

    def set_env(
        self,
        robot_uid: int,
        start_state: RobotState,
        goal_state: RobotState,
        obstacle_positions: List[Tuple[float, float, float]],
        obstacle_dimensions: List[Tuple[float, float, float]],
        check_collision_fn: Callable[[RobotState], bool],
    ):
        super().set_env(
            robot_uid,
            start_state,
            goal_state,
            obstacle_positions,
            obstacle_dimensions,
            check_collision_fn,
        )

    def _initialize_tree(self):
        """
        Reset the tree to start a new planning
        Reset tree and add the start state to the tree
        """
        self.tree = RrtStarTree()
        self.tree.add_root(self.start_state)

    def _sample_robot_state(self) -> RobotState:
        return super()._sample_robot_state()

    def _find_nearest_node(self, robot_state: RobotState) -> Tuple[int, RobotState]:
        return super()._find_nearest_node(robot_state)

    def _steer(
        self, nearest_node_robot_state: RobotState, sampled_robot_state: RobotState
    ) -> RobotState:
        return super()._steer(nearest_node_robot_state, sampled_robot_state)

    def _is_state_already_in_tree(self, robot_state: RobotState) -> bool:
        return super()._is_state_already_in_tree(robot_state)

    def _is_collision_between_states(
        self, robot_state_i: RobotState, robot_state_j: RobotState
    ) -> bool:
        return super()._is_collision_between_states(robot_state_i, robot_state_j)

    def _add_node_to_tree(
        self,
        robot_state: RobotState,
        parent_idx: int,
        cost: float,
        cost_from_parent: float,
    ) -> int:
        return self.tree.add_node(
            robot_state=robot_state,
            parent_idx=parent_idx,
            cost=cost,
            cost_from_parent=cost_from_parent,
        )

    def _is_goal_reached(self, robot_state: RobotState) -> bool:
        return super()._is_goal_reached(robot_state)

    def _add_goal_state_if_not_in_tree(self, parent_node_idx):
        return super()._add_goal_state_if_not_in_tree(parent_node_idx)

    def _get_path_to_goal(self) -> List[RobotState]:
        return super()._get_path_to_goal()

    def plan_path(self) -> List[RobotState]:
        """
        Plan a path from start to goal using RRT algorithm

        1. Initialize tree
        2. Sample random point
        3. Find nearest node in the tree
        4. Drive from nearest node to random point
        5. Check collision
            If collision, go to 2
        6-1. Find near nodes
        6-2. Find the parent node with the minimum cost and collision-free path
        6-3. Rewire the tree
        7. Repeat 2-6 until goal is reached or max_iter is reached
        """

        self._initialize_tree()

        path_found = False

        for sample_iter in range(self.max_iter):
            if sample_iter % 1000 == 0:
                print(f"iter: {sample_iter}")

            random_robot_state = self._sample_robot_state()

            nearest_node_idx, nearest_node_robot_state = self._find_nearest_node(
                random_robot_state
            )

            new_robot_state = self._steer(nearest_node_robot_state, random_robot_state)

            if self._is_state_already_in_tree(new_robot_state):
                continue

            if self._is_collision_between_states(
                nearest_node_robot_state, new_robot_state
            ):
                continue

            # collision_free_near_nodes_idx, cost_to_collision_free_near_nodes = (
            #     self._find_collision_free_near_nodes(new_robot_state)
            # )

            # near_nodes_idx = self.tree.find_near_nodes(new_node_pos)

            # collision_free_near_nodes_idx, cost_to_collision_free_near_nodes = (
            #     self._get_collision_free_near_nodes(new_node_pos, near_nodes_idx)
            # )

            # new_node_idx = self.tree.find_optimal_parent_and_add_node_to_tree(
            #     new_node_pos,
            #     collision_free_near_nodes_idx,
            #     cost_to_collision_free_near_nodes,
            # )

            # # Rewire tree
            # self._rewire_tree(new_node_idx, near_nodes_idx)

            # # check if goal is reached
            # if self._check_goal_reached(new_node_pos, goal_pos):
            #     path_found = True
            #     if not self.tree.check_if_pos_in_tree(goal_pos):
            #         cost_to_goal = self._get_cost_between_pos(new_node_pos, goal_pos)
            #         self.tree.add_node(
            #             goal_pos,
            #             new_node_idx,
            #             self.tree.get_cost_of_node_idx(new_node_idx) + cost_to_goal,
            #             cost_to_goal,
            #         )
            #     break

            # PlotUtils.plot_end_effector_line_between_robot_states(
            #     robot_uid=self.robot_uid,
            #     joint_ids=self.joint_ids,
            #     robot_state_i=nearest_node_robot_state,
            #     robot_state_j=new_robot_state,
            # )

            # print("iter   : ", sample_iter)
            # print("random : ", [f"{x:.2f}" for x in random_robot_state])
            # print("nearest: ", [f"{x:.2f}" for x in nearest_node_robot_state])
            # print("new    : ", [f"{x:.2f}" for x in new_robot_state])

            # pdb.set_trace()
            # input("Press Enter to continue...")

            new_node_idx = self._add_node_to_tree(
                robot_state=new_robot_state, parent_idx=nearest_node_idx
            )

            if self._is_goal_reached(robot_state=new_robot_state):
                print("==================== Goal reached ====================")
                self._add_goal_state_if_not_in_tree(parent_node_idx=new_node_idx)
                path_found = True
                break

        if not path_found:
            print("-------------------- Path not found --------------------")
            return None, sample_iter

        return self._get_path_to_goal(), sample_iter
