import numpy as np
from typing import List, Tuple
import time

import pybullet as p
import pybullet_data

from utils.collision_utils import get_collision_fn

import utils.bullet_obj_utils as bullet_obj_utils
import utils.plot_utils as plot_utils


def get_env_config_demo() -> (
    Tuple[np.ndarray, float, List[float], List[List[float]], List[List[float]]]
):
    """
    Return a env config for testing

    Returns
    -------
    start_state, rail_length, goal_ee_pos, obstacle_positions, obstacle_dimensions
    """
    start_state = np.array([3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0])
    rail_length = 3.0
    goal_ee_pos = [0.162, -0.192, 0.906]

    obstacle_positions = [
        # [0.5, 0.0, 0.0],
        [0.0, 0.5, 0.0],
        [0.0, 0.0, 0.5],
    ]
    obstacle_dimensions = [
        # [0.2, 0.1, 0.1],
        [0.1, 0.3, 0.1],
        [0.1, 0.1, 0.4],
    ]
    return (
        start_state,
        rail_length,
        goal_ee_pos,
        obstacle_positions,
        obstacle_dimensions,
    )


def test_rrt_star():
    # =============================== init sim ===============================
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    urdf_path = "urdf/ur5.urdf"

    # =============================== get env ===============================
    # env config
    start_state, rail_length, goal_ee_pos, obstacle_positions, obstacle_dimensions = (
        get_env_config_demo()
    )

    start_ee_pos = [-0.343, -0.191, 0.847]  # for plotting

    # =============================== spawn objects ===============================
    # spawn plane
    plane_uid = p.loadURDF("plane.urdf")

    # spawn robot
    robot_uid = bullet_obj_utils.spawn_robot(urdf_path=urdf_path)
    non_fixed_joint_uids = bullet_obj_utils.get_non_fixed_joints(robot_uid)

    # spawn obstacles
    spawned_obstacle_uids = bullet_obj_utils.spawn_obstacles(
        obstacle_positions, obstacle_dimensions
    )
    obstacle_uids = [plane_uid] + spawned_obstacle_uids

    # =============================== set and plot env ===============================
    bullet_obj_utils.set_joint_positions(
        robot_uid=robot_uid, joint_ids=non_fixed_joint_uids, joint_states=start_state
    )
    plot_utils.plot_start_and_goal_pos(start_ee_pos, goal_ee_pos)
    plot_utils.plot_rail(rail_length=rail_length)

    p.resetDebugVisualizerCamera(
        cameraDistance=rail_length,
        cameraYaw=0,
        cameraPitch=-135,
        # cameraPitch=-45,
        cameraTargetPosition=[rail_length * 0.5, 0.0, 0.0],
    )
    # =============================== get collision checker ===============================

    is_collision = get_collision_fn(robot_uid, non_fixed_joint_uids, obstacle_uids)

    # joint_states = [
    #     # [3.14, 0.0, 0.0, 0.0, 0.0, 0.0],
    #     # [3.14, -np.pi * 0.125, 0.0, 0.0, 0.0, 0.0],
    #     [3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0],
    #     [3.14, -np.pi * 0.5, 0.0, 0.0, 0.0, 0.0],
    #     [3.14, -np.pi * 0.6, 0.0, 0.0, 0.0, 0.0],
    # ]

    # for joint_state in joint_states:
    #     print(is_collision(joint_state))
    #     print("ee pose: ", bullet_obj_utils.get_end_effector_position(robot_uid))

    # while p.isConnected():
    #     pass
    #     # p.stepSimulation()

    # =============================== plan path ===============================

    rrt_path = [
        np.array([3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0]),
        np.array([3.14, -np.pi * 0.5, 0.0, 0.0, 0.0, 0.0]),
        np.array([3.14, -np.pi * 0.6, 0.0, 0.0, 0.0, 0.0]),
    ]

    # =============================== plot path ===============================

    # plot_utils.plot_path_forever(
    #     rrt_path,
    #     non_fixed_joint_uids,
    #     robot_uid,
    #     plot_end_effector_pos=True,
    # )

    while p.isConnected():
        pass
        # p.stepSimulation()


if __name__ == "__main__":
    test_rrt_star()
