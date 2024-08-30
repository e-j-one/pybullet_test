import time

import pybullet as p
import numpy as np

from dataset_generator.arm_on_rail_data_generator import ArmOnRailDataGenerator

from planners.rrt_planner import RrtPlanner
from planners.rrt_star_planner import RrtStarPlanner
from env.arm_on_rail_env import ArmOnRailEnv
import utils.plot_utils as plot_utils

SEED = 0
np.random.seed(SEED)


def generate_dataset():
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

    # =============================== get env ===============================
    data_generator = ArmOnRailDataGenerator()
    data_generator.set_config(
        rail_length=3.0,
        num_obstacles=4,
    )
    start_state, goal_state, rail_length, obstacle_positions, obstacle_dimensions = (
        data_generator.get_env_config_demo()
    )

    # =============================== get env ===============================
    arm_on_rail_env = ArmOnRailEnv()

    # =============================== get collision checker ===============================
    arm_on_rail_env.reset_env(
        robot_urdf_path="urdf/ur5.urdf",
        rail_length=rail_length,
        start_state=start_state,
        goal_state=goal_state,
        obstacle_positions=obstacle_positions,
        obstacle_dimensions=obstacle_dimensions,
    )

    # =============================== plan path ===============================

    robot_uid = arm_on_rail_env.get_robot_uid()
    non_fixed_joint_ids = arm_on_rail_env.get_non_fixed_joint_ids()
    robot_state_ranges = arm_on_rail_env.get_robot_state_ranges()
    check_collision = arm_on_rail_env.get_check_collision_fn()

    num_samples = -1
    if test_algo == "rrt":
        motion_planner = RrtPlanner(
            max_iter=planner_config["max_iter"],
            joint_ids=non_fixed_joint_ids,
            robot_state_ranges=robot_state_ranges,
            collision_check_step_size=planner_config["collision_check_step_size"],
            goal_reached_threshold=planner_config["goal_reached_threshold"],
            drive_dist=planner_config["drive_dist"],
            goal_sample_rate=planner_config["goal_sample_rate"],
        )
    elif test_algo == "rrt_star":
        motion_planner = RrtStarPlanner(
            max_iter=planner_config["max_iter"],
            joint_ids=non_fixed_joint_ids,
            robot_state_ranges=robot_state_ranges,
            collision_check_step_size=planner_config["collision_check_step_size"],
            goal_reached_threshold=planner_config["goal_reached_threshold"],
            drive_dist=planner_config["drive_dist"],
            goal_sample_rate=planner_config["goal_sample_rate"],
            near_radius=planner_config["near_radius"],
        )

    motion_planner.set_env(
        robot_uid=robot_uid,
        start_state=start_state,
        goal_state=goal_state,
        obstacle_positions=obstacle_positions,
        obstacle_dimensions=obstacle_dimensions,
        check_collision_fn=check_collision,
    )

    start_time = time.time()
    sbmp_path, num_samples = motion_planner.plan_path()
    print("time: ", time.time() - start_time)

    print("n sample: ", num_samples)

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
    generate_dataset()
