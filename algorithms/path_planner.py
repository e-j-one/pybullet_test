from enum import Enum
from typing import List, Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt


class PathPlanner:
    def __init__(self, config):
        pass

    def set_env(
        self,
        start_state: List[float],
        goal_ee_pos: Tuple[float, float, float],
        obstacle_positions: List[Tuple[float, float, float]],
        obstacle_dimensions: List[Tuple[float, float, float]],
    ):
        self.start_state = start_state
        self.goal_ee_pos = goal_ee_pos
        self.obstacle_positions = obstacle_positions
        self.obstacle_dimensions = obstacle_dimensions

    def plan_path(self) -> List[List[float]]:
        raise NotImplementedError
