from typing import List, Tuple, Callable

import numpy as np

import utils.geometry_utils as GeometryUtils
from utils.types import RobotState


class PathPlanner:
    def __init__(
        self,
        max_iter: int,
        collision_check_step_size: float,
        goal_reached_threshold: float,
    ):
        self.max_iter = max_iter
        self.collision_check_step_size = collision_check_step_size
        self.goal_reached_threshold = goal_reached_threshold

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
    ):
        """
        Parameters
        -------
        start_state: Tuple (2d_x, joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        goal_state: Tuple (2d_x, joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        obstacle_positions: List of Tuple (3d_x, 3d_y, 3d_z)
        obstacle_dimensions: W x L x H
        """
        self.robot_uid = robot_uid
        self.joint_ids = joint_ids
        self.robot_state_ranges = robot_state_ranges
        self.start_state = start_state
        self.goal_state = goal_state
        self.obstacle_positions = obstacle_positions
        self.obstacle_dimensions = obstacle_dimensions
        self.check_collision_fn = check_collision_fn

    def _is_state_in_collision(self, robot_state: RobotState) -> bool:
        return self.check_collision_fn(robot_state)

    def _is_collision_between_states(
        self, robot_state_i: RobotState, robot_state_j: RobotState
    ) -> bool:
        """
        Check if there is a collision between the states robot_state_i and robot_state_j
        Progresively sample points between the two states and check for collision
        """
        dist_between_states = np.linalg.norm(
            np.array(robot_state_i) - np.array(robot_state_j)
        )

        dist_travelled = 0
        while dist_travelled < dist_between_states:
            state_to_check = GeometryUtils.move_state_towards_target(
                start_state=robot_state_i,
                target_state=robot_state_j,
                step_size=dist_travelled,
            )
            if self._is_state_in_collision(state_to_check):
                return True
            dist_travelled += self.collision_check_step_size

        if self._is_state_in_collision(robot_state_j):
            return True

        return False

    def plan_path(self) -> List[List[float]]:
        raise NotImplementedError
