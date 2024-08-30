import os
import time
import json

import pybullet as p
import numpy as np

from dataset_generator.arm_on_rail_data_generator import ArmOnRailDataGenerator

from planners.rrt_star_planner import RrtStarPlanner
from arm_on_rail_env.arm_on_rail_env import ArmOnRailEnv
import utils.plot_utils as plot_utils

SEED = 1
np.random.seed(SEED)


def generate_dataset(
    start_seed=0,
    num_samples=2000,
    rail_length=3.0,
    num_obstacles=10,
    max_iter=1000000,
    dataset_dir="datasets/train/",
):
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

    dataset_dir = dataset_dir + str(rail_length) + str(num_obstacles) + "/"

    # Create the dataset directory if it does not exist
    if not os.path.exists(dataset_dir):
        os.makedirs(dataset_dir)

    arm_on_rail_env = ArmOnRailEnv()
    arm_on_rail_env.set_robot_urdf_path(robot_urdf_path=robot_urdf_path)

    path_planner = RrtStarPlanner(
        max_iter=planner_config["max_iter"],
        collision_check_step_size=planner_config["collision_check_step_size"],
        goal_reached_threshold=planner_config["goal_reached_threshold"],
        drive_dist=planner_config["drive_dist"],
        goal_sample_rate=planner_config["goal_sample_rate"],
        near_radius=planner_config["near_radius"],
    )
    arm_on_rail_env.set_path_planner(path_planner)

    data_generator = ArmOnRailDataGenerator()
    data_generator.set_config(
        rail_length=rail_length,
        num_obstacles=num_obstacles,
    )

    map_seed = start_seed
    num_samples_collected = 0

    while num_samples_collected < num_samples:
        map_seed += 1
        np.random.seed(map_seed)
        (
            start_state,
            goal_state,
            rail_length,
            obstacle_positions,
            obstacle_dimensions,
        ) = data_generator.sample_new_map_data()

        arm_on_rail_env.reset_env(
            rail_length=rail_length,
            start_state=start_state,
            goal_state=goal_state,
            obstacle_positions=obstacle_positions,
            obstacle_dimensions=obstacle_dimensions,
        )
        arm_on_rail_env.set_path_planner_env()

        if arm_on_rail_env.is_initial_state_in_collision():
            print("============ Initial state is in collision ============")
            print("============ robot uid: ", arm_on_rail_env.get_robot_uid())
            print("============ obstc uid: ", arm_on_rail_env.get_obstacle_uids())
            arm_on_rail_env.close_sim_env()
            continue

        start_time = time.time()
        sbmp_path, num_samples = arm_on_rail_env.plan_path()
        print(
            "map_seed: ",
            map_seed,
            " n_sample: ",
            num_samples,
            " time: ",
            time.time() - start_time,
        )

        if sbmp_path is None:
            arm_on_rail_env.close_sim_env()
            print("============ Path not found ============")
            continue

        arm_on_rail_env.plot_path()
        arm_on_rail_env.close_sim_env()

        map_data = {
            "start_state": start_state,
            "goal_state": goal_state,
            "rail_length": rail_length,
            "obstacle_positions": obstacle_positions,
            "obstacle_dimensions": obstacle_dimensions,
            "planner_config": planner_config,
            "rrt_star_path": sbmp_path,
        }

        file_path = dataset_dir + str(map_seed) + ".json"
        with open(file_path, "w") as json_file:
            json.dump(map_data, json_file, indent=4)

        num_samples_collected += 1
        # while p.isConnected():
        #     pass
        # p.stepSimulation()


if __name__ == "__main__":
    dataset_dir = "datasets/train_240831/"
    dataset_configs = [
        {
            "num_samples": 2000,
            "rail_length": 1.0,
            "num_obstacles": 4,
        },
        {
            "num_samples": 2000,
            "rail_length": 2.0,
            "num_obstacles": 8,
        },
        {
            "num_samples": 2000,
            "rail_length": 3.0,
            "num_obstacles": 12,
        },
        {
            "num_samples": 2000,
            "rail_length": 4.0,
            "num_obstacles": 16,
        },
    ]

    for config in dataset_configs:
        generate_dataset(
            start_seed=SEED,
            num_samples=config["num_samples"],
            rail_length=config["rail_length"],
            num_obstacles=config["num_obstacles"],
            max_iter=1000000,
            dataset_dir=dataset_dir,
        )
