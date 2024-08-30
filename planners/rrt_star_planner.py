from typing import Tuple, List, Callable, Optional

from planners.rrt_planner import RrtPlanner
from planners.utils.rrt_trees import RrtStarTree

import utils.geometry_utils as GeometryUtils
import utils.plot_utils as PlotUtils
from utils.types import RobotState


class RrtStarPlanner(RrtPlanner):
    def __init__(
        self,
        max_iter: int,
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
            collision_check_step_size=collision_check_step_size,
            goal_reached_threshold=goal_reached_threshold,
            drive_dist=drive_dist,
            goal_sample_rate=goal_sample_rate,
        )
        self.near_radius = near_radius

    def set_env(
        self,
        robot_uid: int,
        joint_ids: List[int],
        robot_state_ranges: List[Tuple[float, float]],
        start_state: RobotState,
        goal_state: RobotState,
        obstacle_positions: List[Tuple[float, float, float]],
        obstacle_dimensions: List[Tuple[float, float, float]],
        check_collision_fn: Callable[[RobotState], bool],
        use_goal_reached_fn: bool = False,
        check_goal_reached_fn=None,
    ):
        super().set_env(
            robot_uid,
            joint_ids,
            robot_state_ranges,
            start_state,
            goal_state,
            obstacle_positions,
            obstacle_dimensions,
            check_collision_fn,
            use_goal_reached_fn=use_goal_reached_fn,
            check_goal_reached_fn=check_goal_reached_fn,
        )

    def _initialize_tree(self):
        """
        Reset the tree to start a new planning
        Reset tree and add the start state to the tree
        """
        self.tree = RrtStarTree(
            near_node_dist_trheshold=self.near_radius,
        )
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

    def _get_collision_free_near_nodes(
        self, new_node_pos: Tuple[float, float], near_nodes_idx: List[int]
    ):
        collision_free_near_nodes_idx = []
        cost_to_collision_free_near_nodes = []
        for near_node_idx in near_nodes_idx:
            if not self._check_collision_between_pos(
                self.tree.get_pos_of_node(near_node_idx), new_node_pos
            ):
                collision_free_near_nodes_idx.append(near_node_idx)
                cost_to_collision_free_near_nodes.append(
                    self._get_cost_between_pos(
                        self.tree.get_pos_of_node(near_node_idx), new_node_pos
                    )
                )
        return collision_free_near_nodes_idx, cost_to_collision_free_near_nodes

    def _find_collision_free_near_nodes(
        self, new_robot_state: RobotState
    ) -> Tuple[List[int], List[float]]:
        """
        Returns
        -------
        collision_free_near_nodes_idx: List[int]
            List of indices of near nodes that have a collision-free path to the new node
        cost_to_collision_free_near_nodes: List[float]
            List of costs to the near nodes that have a collision-free path to the new node
        """
        near_node_idxes = self.tree.find_near_nodes(new_robot_state)

        collision_free_near_node_idxes = []
        cost_to_collision_free_near_nodes = []

        for near_node_idx in near_node_idxes:
            near_node_robot_state = self.tree.get_robot_state_of_node(near_node_idx)
            if self._is_collision_between_states(
                near_node_robot_state, new_robot_state
            ):
                continue

            collision_free_near_node_idxes.append(near_node_idx)
            cost_to_collision_free_near_nodes.append(
                GeometryUtils.get_cost_between_states(
                    near_node_robot_state, new_robot_state
                )
            )

        return collision_free_near_node_idxes, cost_to_collision_free_near_nodes

    def _find_optimal_parent_and_add_node_to_tree(
        self,
        new_robot_state: RobotState,
        collision_free_near_node_idxes: List[int],
        cost_to_collision_free_near_nodes: List[float],
    ) -> int:
        """
        Find the parent node with the minimum cost and collision-free path
        Add the new node to the tree

        Returns
        -------
        new_node_idx: int
        """
        return self.tree.find_optimal_parent_and_add_node_to_tree(
            new_robot_state=new_robot_state,
            collision_free_near_node_idxes=collision_free_near_node_idxes,
            cost_to_collision_free_near_nodes=cost_to_collision_free_near_nodes,
        )

    def _rewire_tree(
        self,
        new_node_idx: int,
        collision_free_near_node_idxes: List[int],
        cost_to_collision_free_near_nodes: List[float],
    ):
        """
        Rewire the tree to update the parent of near nodes if a collision-free path is found with the new node
        Assuming cost(a,b) = cost(b,a) for all a, b
        """
        cost_of_new_node = self.tree.get_cost_of_node_idx(new_node_idx)

        for near_node_idx, cost_to_near_node in zip(
            collision_free_near_node_idxes, cost_to_collision_free_near_nodes
        ):
            updated_cost = cost_of_new_node + cost_to_near_node

            if updated_cost < self.tree.get_cost_of_node_idx(near_node_idx):
                self.tree.update_parent_and_cost(
                    node_idx=near_node_idx,
                    new_parent_idx=new_node_idx,
                    new_cost=updated_cost,
                    new_cost_from_parent=cost_to_near_node,
                )

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
        if not self.tree.check_if_robot_state_already_in_tree(self.goal_state):
            parent_node_cost = self.tree.get_cost_of_node_idx(parent_node_idx)
            cost_from_parent_to_goal = GeometryUtils.get_cost_between_states(
                self.tree.get_robot_state_of_node(parent_node_idx), self.goal_state
            )
            self.tree.add_node(
                self.goal_state,
                parent_node_idx,
                parent_node_cost + cost_from_parent_to_goal,
                cost_from_parent_to_goal,
            )

    def _get_path_to_goal(self) -> List[RobotState]:
        return super()._get_path_to_goal()

    def _plot_debugline_between_robot_states(
        self, robot_state_i: RobotState, robot_state_j: RobotState
    ):
        PlotUtils.plot_end_effector_line_between_robot_states(
            robot_uid=self.robot_uid,
            joint_ids=self.joint_ids,
            robot_state_i=robot_state_i,
            robot_state_j=robot_state_j,
        )

    def plan_path(self) -> Optional[List[RobotState]]:
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

        Returns
        -------
        None if path is not found
        List of robot states in the path if path is found
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

            collision_free_near_node_idxes, cost_to_collision_free_near_nodes = (
                self._find_collision_free_near_nodes(new_robot_state)
            )

            new_node_idx = self._find_optimal_parent_and_add_node_to_tree(
                new_robot_state,
                collision_free_near_node_idxes,
                cost_to_collision_free_near_nodes,
            )

            # self._plot_debugline_between_robot_states(
            #     nearest_node_robot_state, new_robot_state
            # )

            self._rewire_tree(
                new_node_idx,
                collision_free_near_node_idxes,
                cost_to_collision_free_near_nodes,
            )

            # print("iter   : ", sample_iter)
            # print("random : ", [f"{x:.2f}" for x in random_robot_state])
            # print("nearest: ", [f"{x:.2f}" for x in nearest_node_robot_state])
            # print("new    : ", [f"{x:.2f}" for x in new_robot_state])

            # input("Press Enter to continue...")

            if self._is_goal_reached(robot_state=new_robot_state):
                print("==================== Goal reached ====================")
                if self.use_goal_reached_fn:
                    self.goal_reached_node_idx = new_node_idx
                else:
                    self._add_goal_state_if_not_in_tree(parent_node_idx=new_node_idx)
                path_found = True
                break

        if not path_found:
            print("-------------------- Path not found --------------------")
            return None, sample_iter

        return self._get_path_to_goal(), sample_iter
