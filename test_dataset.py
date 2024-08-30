import os
import time
import json

import pybullet as p
import numpy as np

from dataset_generator.arm_on_rail_data_generator import ArmOnRailDataGenerator

from planners.rrt_star_planner import RrtStarPlanner
from arm_on_rail_env.arm_on_rail_env import ArmOnRailEnv
import utils.plot_utils as plot_utils

SEED = 5
np.random.seed(SEED)


def test_dataset():
    planner_config = {
        "max_iter": 100000,
        "collision_check_step_size": 0.04,
        "goal_reached_threshold": 0.04,
        # rrt
        "drive_dist": 0.628,
        "goal_sample_rate": 0.1,
        # rrt star
        "near_radius": 1.256,
    }
    robot_urdf_path = "urdf/ur5.urdf"

    dataset_path = "datasets/train_240831/1.0/2.json"

    # if file does not exist, exit
    if not os.path.exists(dataset_path):
        print("Dataset does not exist")
        return

    arm_on_rail_env = ArmOnRailEnv()
    arm_on_rail_env.set_robot_urdf_path(robot_urdf_path=robot_urdf_path)
    arm_on_rail_env.load_env_from_json(
        json_file_path=dataset_path,
    )
    arm_on_rail_env.plot_path_label()

    path_planner = RrtStarPlanner(
        max_iter=planner_config["max_iter"],
        collision_check_step_size=planner_config["collision_check_step_size"],
        goal_reached_threshold=planner_config["goal_reached_threshold"],
        drive_dist=planner_config["drive_dist"],
        goal_sample_rate=planner_config["goal_sample_rate"],
        near_radius=planner_config["near_radius"],
    )
    arm_on_rail_env.set_path_planner(path_planner)
    arm_on_rail_env.set_path_planner_env()

    if arm_on_rail_env.is_initial_state_in_collision():
        print("==================== !!! ERROR !!! ====================")
        print("============ Initial state is in collision ============")
        print("============ robot uid: ", arm_on_rail_env.get_robot_uid())
        print("============ obstc uid: ", arm_on_rail_env.get_obstacle_uids())
        arm_on_rail_env.close_sim_env()
        return

    # =============================== plan path ===============================
    start_time = time.time()
    sbmp_path, num_samples = arm_on_rail_env.plan_path()
    print("time: ", time.time() - start_time)
    print("n sample: ", num_samples)
    arm_on_rail_env.plot_path(10)
    arm_on_rail_env.close_sim_env()


if __name__ == "__main__":
    test_dataset()
