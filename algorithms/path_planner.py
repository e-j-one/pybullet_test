from enum import Enum
from typing import List, Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt

from utils.types import RobotState


class PathPlanner:
    def __init__(self, config):
        pass

    def set_env(
        self,
        start_state: RobotState,
        goal_state: RobotState,
        obstacle_positions: List[Tuple[float, float, float]],
        obstacle_dimensions: List[Tuple[float, float, float]],
    ):
        """
        Parameters
        -------
        start_state: Tuple (2d_x, joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        goal_state: Tuple (2d_x, joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        obstacle_positions: List of Tuple (3d_x, 3d_y, 3d_z)
        obstacle_dimensions: W x L x H
        """
        self.start_state = start_state
        self.goal_state = goal_state
        self.obstacle_positions = obstacle_positions
        self.obstacle_dimensions = obstacle_dimensions

    def plan_path(self) -> List[List[float]]:
        raise NotImplementedError
