import numpy as np
from typing import List, Tuple
import time

import pybullet as p
import pybullet_data

from planners.rrt_planner import RrtPlanner
from planners.rrt_star_planner import RrtStarPlanner

import utils.plot_utils as plot_utils
import utils.bullet_obj_utils as bullet_obj_utils
from utils.collision_utils import get_check_collision_fn
from utils.types import RobotState

SEED = 0
np.random.seed(SEED)


def get_env_config_demo() -> Tuple[
    RobotState,
    RobotState,
    float,
    List[Tuple[float, float, float]],
    List[Tuple[float, float, float]],
]:
    """
    Return a env config for testing.
    In rail env, 2d_x in [0, rail_length], 2d_y = 0, 2d_yaw = 0 for robot state.

    Returns
    -------
    start_state, goal_state, rail_length, obstacle_positions, obstacle_dimensions

    state: np.ndarray (2d_x, 2d_y, 2d_yaw, joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    """
    start_state = (0.0, 3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0)
    goal_state = (3.0, 3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0)
    rail_length = 4.0
    # goal_ee_pos = [0.162, -0.192, 0.906]

    obstacle_positions = [
        # [0.5, 0.0, 0.0],
        [0.0, 0.5, 0.0],
        [0.0, 0.0, 0.5],
        # [1.5, 0.0, 0.5],
        [1.5, -1.5, 0.8],
    ]
    obstacle_dimensions = [
        # [0.2, 0.1, 0.1],
        [0.1, 0.3, 0.1],
        [0.1, 0.1, 0.4],
        # [0.1, 0.1, 0.4],
        [0.1, 3.0, 0.4],
    ]
    return (
        start_state,
        goal_state,
        rail_length,
        obstacle_positions,
        obstacle_dimensions,
    )


def test_rrt_star():
    planner_config = {
        "max_iter": 80000,
        "collision_check_step_size": 0.04,
        "goal_reached_threshold": 0.04,
        # rrt
        "drive_dist": 0.314,
        "goal_sample_rate": 0.05,
        # rrt star
        "near_radius": 0.648,
    }
    test_algo = "rrt_star"
    # =============================== init sim ===============================
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    urdf_path = "urdf/ur5.urdf"

    # =============================== get env ===============================
    # env config
    start_state, goal_state, rail_length, obstacle_positions, obstacle_dimensions = (
        get_env_config_demo()
    )

    # =============================== spawn objects ===============================
    # spawn plane
    plane_uid = p.loadURDF("plane.urdf")

    # spawn robot
    robot_uid = bullet_obj_utils.spawn_robot(urdf_path=urdf_path)
    non_fixed_joint_ids = bullet_obj_utils.get_non_fixed_joints(robot_uid)

    # spawn obstacles
    spawned_obstacle_uids = bullet_obj_utils.spawn_obstacles(
        obstacle_positions, obstacle_dimensions
    )
    obstacle_uids = [plane_uid] + spawned_obstacle_uids
    plot_utils.plot_rail(rail_length=rail_length)

    # =============================== set and plot env ===============================

    bullet_obj_utils.set_base_and_joint_positions(
        robot_uid=robot_uid, joint_ids=non_fixed_joint_ids, robot_state=goal_state
    )
    goal_ee_pos = bullet_obj_utils.get_end_effector_position(robot_uid)

    bullet_obj_utils.set_base_and_joint_positions(
        robot_uid=robot_uid, joint_ids=non_fixed_joint_ids, robot_state=start_state
    )
    start_ee_pos = bullet_obj_utils.get_end_effector_position(robot_uid)

    plot_utils.plot_start_and_goal_pos(start_ee_pos, goal_ee_pos)

    p.resetDebugVisualizerCamera(
        cameraDistance=rail_length,
        cameraYaw=0,
        cameraPitch=-150,
        # cameraPitch=-45,
        cameraTargetPosition=[rail_length * 0.5, 0.0, 0.0],
    )
    # =============================== get collision checker ===============================

    check_collision = get_check_collision_fn(
        robot_uid=robot_uid,
        joint_ids=non_fixed_joint_ids,
        obstacles=obstacle_uids,
        rail_length=rail_length,
    )

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

    robot_state_ranges = bullet_obj_utils.get_robot_state_ranges(
        robot_uid=robot_uid, joint_ids=non_fixed_joint_ids, rail_length=rail_length
    )
    # print("robot_state_ranges: ", robot_state_ranges)

    num_samples = -1
    if test_algo == "rrt":
        motion_planner = RrtPlanner(
            max_iter=planner_config["max_iter"],
            collision_check_step_size=planner_config["collision_check_step_size"],
            goal_reached_threshold=planner_config["goal_reached_threshold"],
            drive_dist=planner_config["drive_dist"],
            goal_sample_rate=planner_config["goal_sample_rate"],
        )
    elif test_algo == "rrt_star":
        motion_planner = RrtStarPlanner(
            max_iter=planner_config["max_iter"],
            collision_check_step_size=planner_config["collision_check_step_size"],
            goal_reached_threshold=planner_config["goal_reached_threshold"],
            drive_dist=planner_config["drive_dist"],
            goal_sample_rate=planner_config["goal_sample_rate"],
            near_radius=planner_config["near_radius"],
        )

    motion_planner.set_env(
        robot_uid=robot_uid,
        joint_ids=non_fixed_joint_ids,
        robot_state_ranges=robot_state_ranges,
        start_state=start_state,
        goal_state=goal_state,
        obstacle_positions=obstacle_positions,
        obstacle_dimensions=obstacle_dimensions,
        check_collision_fn=check_collision,
    )
    sbmp_path, num_samples = motion_planner.plan_path()

    print("n sample: ", num_samples)

    # sbmp_path = [
    #     start_state,
    #     goal_state,
    # ]

    # =============================== plot path ===============================

    if sbmp_path is not None:
        plot_utils.plot_path_forever(
            path=sbmp_path,
            joint_ids=non_fixed_joint_ids,
            robot_uid=robot_uid,
            plot_end_effector_poses=True,
        )

    while p.isConnected():
        pass
        # p.stepSimulation()


if __name__ == "__main__":
    test_rrt_star()
