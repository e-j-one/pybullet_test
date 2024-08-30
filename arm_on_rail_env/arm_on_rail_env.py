from typing import Tuple, List
import json

import pybullet as p
import pybullet_data

import utils.plot_utils as plot_utils
import utils.bullet_obj_utils as bullet_obj_utils
from utils.collision_utils import get_check_collision_fn
from utils.types import RobotState


class ArmOnRailEnv:
    def __init__(self):
        pass

    def set_robot_urdf_path(self, robot_urdf_path):
        self.robot_urdf_path = robot_urdf_path

    def load_env_from_json(self, json_file_path: str):
        with open(json_file_path, "r") as f:
            env_data = json.load(f)
        self.reset_env(**env_data)

        self.path_label = env_data["path_label"]

    def reset_env(
        self,
        rail_length: float,
        start_state: RobotState,
        goal_state: RobotState,
        obstacle_positions: List[Tuple[float, float, float]],
        obstacle_dimensions: List[Tuple[float, float, float]],
    ):
        """
        Connect to the pybullet GUI and reset the environment with the given config.
        Spawn the robot, obstacles and set the start and goal states.
        """
        # init sim env
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.start_state = start_state
        self.goal_state = goal_state
        self.obstacle_positions = obstacle_positions
        self.obstacle_dimensions = obstacle_dimensions

        # spawn plane, robot and obstacles
        self.plane_uid = p.loadURDF("plane.urdf")
        self.robot_uid = bullet_obj_utils.spawn_robot(urdf_path=self.robot_urdf_path)
        spawned_obstacle_uids = bullet_obj_utils.spawn_obstacles(
            obstacle_positions, obstacle_dimensions
        )
        self.obstacle_uids = [self.plane_uid] + spawned_obstacle_uids
        plot_utils.plot_rail(rail_length=rail_length)

        self.non_fixed_joint_ids = bullet_obj_utils.get_non_fixed_joints(self.robot_uid)
        self.robot_state_ranges = bullet_obj_utils.get_robot_state_ranges(
            robot_uid=self.robot_uid,
            joint_ids=self.non_fixed_joint_ids,
            rail_length=rail_length,
        )

        p.resetDebugVisualizerCamera(
            cameraDistance=rail_length * 1.5,
            cameraYaw=0,
            cameraPitch=-150,
            # cameraPitch=-45,
            cameraTargetPosition=[rail_length * 0.5, 0.0, 0.0],
        )

        self.check_collision = get_check_collision_fn(
            robot_uid=self.robot_uid,
            joint_ids=self.non_fixed_joint_ids,
            obstacles=self.obstacle_uids,
            rail_length=rail_length,
        )

        self.mark_start_and_goal_ee_pos()

    def mark_start_and_goal_ee_pos(self):
        bullet_obj_utils.set_base_and_joint_positions(
            robot_uid=self.robot_uid,
            joint_ids=self.non_fixed_joint_ids,
            robot_state=self.goal_state,
        )
        goal_ee_pos = bullet_obj_utils.get_end_effector_position(self.robot_uid)
        bullet_obj_utils.set_base_and_joint_positions(
            robot_uid=self.robot_uid,
            joint_ids=self.non_fixed_joint_ids,
            robot_state=self.start_state,
        )
        start_ee_pos = bullet_obj_utils.get_end_effector_position(self.robot_uid)
        plot_utils.plot_start_and_goal_pos(start_ee_pos, goal_ee_pos)

    def is_initial_state_in_collision(self):
        """
        Check if the start and goal states are valid
        """
        if self.check_collision(self.start_state):
            return True
        if self.check_collision(self.goal_state):
            return True
        return False

    def get_non_fixed_joint_ids(self):
        return self.non_fixed_joint_ids

    def get_robot_state_ranges(self):
        return self.robot_state_ranges

    def get_obstacle_uids(self):
        return self.obstacle_uids

    def get_check_collision_fn(self):
        return self.check_collision

    def get_robot_uid(self):
        return self.robot_uid

    def close_sim_env(self):
        p.disconnect()

    def set_path_planner(self, path_planner):
        self.path_planner = path_planner
