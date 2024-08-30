from itertools import product
from typing import List, Tuple

import numpy as np
import pybullet as p

import utils.bullet_obj_utils as bullet_obj_utils
from utils.types import RobotState


def get_check_goal_reached_fn(
    robot_uid: int, joint_ids: List[int], goal_state: RobotState
):
    goal_end_effector_pos = bullet_obj_utils.get_end_effector_position_from_robot_state(
        robot_uid, joint_ids, goal_state
    )

    def is_collision_fn(
        robot_state: RobotState, end_effector_pos_distance_threshold=0.04
    ):
        """
        Check if the robot reached the goal by end effector position.
        """
        current_end_effector_pos = (
            bullet_obj_utils.get_end_effector_position_from_robot_state(
                robot_uid, joint_ids, robot_state
            )
        )

        end_effector_pos_distance = np.linalg.norm(
            np.array(goal_end_effector_pos) - np.array(current_end_effector_pos)
        )

        if end_effector_pos_distance <= end_effector_pos_distance_threshold:
            return True
        return False

    return is_collision_fn
