from typing import Tuple, List
import json

import pybullet as p
import pybullet_data

import utils.plot_utils as plot_utils
import utils.bullet_obj_utils as bullet_obj_utils
from utils.collision_utils import get_check_collision_fn
from utils.goal_reached_utils import get_check_goal_reached_fn
from utils.types import RobotState


class ArmOnRailEnv:
    """

    Example usage
    -------------
    arm_on_rail_env = ArmOnRailEnv()
    arm_on_rail_env.set_robot_urdf_path(robot_urdf_path=robot_urdf_path)
    if read from json:
        arm_on_rail_env.load_env_from_json(
            json_file_path=dataset_path,
        )
        arm_on_rail_env.plot_path_label()
    else:
        arm_on_rail_env.reset_env(
            rail_length=rail_length,
            start_state=start_state,
            goal_state=goal_state,
            obstacle_positions=obstacle_positions,
            obstacle_dimensions=obstacle_dimensions,
        )

    # set path planner
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

    sbmp_path, num_samples = arm_on_rail_env.plan_path()
    arm_on_rail_env.plot_path()
    arm_on_rail_env.close_sim_env()

    """

    def __init__(self):
        pass

    def set_robot_urdf_path(self, robot_urdf_path):
        self.robot_urdf_path = robot_urdf_path

    def load_env_from_json(self, json_file_path: str):
        with open(json_file_path, "r") as f:
            env_data = json.load(f)
        self.reset_env(
            rail_length=env_data["rail_length"],
            start_state=tuple(env_data["start_state"]),
            goal_state=tuple(env_data["goal_state"]),
            obstacle_positions=env_data["obstacle_positions"],
            obstacle_dimensions=env_data["obstacle_dimensions"],
        )

        self.path_label = env_data["rrt_star_path"]

    def reset_env(
        self,
        rail_length: float,
        start_state: RobotState,
        goal_state: RobotState,
        obstacle_positions: List[Tuple[float, float, float]],
        obstacle_dimensions: List[Tuple[float, float, float]],
        use_goal_reached_fn: bool = False,
    ):
        """
        Connect to the pybullet GUI and reset the environment with the given config.
        Spawn the robot, obstacles and set the start and goal states.
        Get collision checker function.

        Parameters
        ----------
        use_goal_reached_fn: bool
            If True, check if the goal state is reached using the check_goal_reached
            function. It checks end effector position to check if the goal is reached.

        """
        # init sim env
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.start_state = start_state
        self.goal_state = goal_state
        self.obstacle_positions = obstacle_positions
        self.obstacle_dimensions = obstacle_dimensions
        self.use_goal_reached_fn = use_goal_reached_fn

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

        if self.use_goal_reached_fn:
            self.check_goal_reached = get_check_goal_reached_fn(
                robot_uid=self.robot_uid,
                joint_ids=self.non_fixed_joint_ids,
                goal_state=self.goal_state,
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

    def set_path_planner_env(self):
        if self.use_goal_reached_fn:
            self.path_planner.set_env(
                robot_uid=self.robot_uid,
                joint_ids=self.non_fixed_joint_ids,
                robot_state_ranges=self.robot_state_ranges,
                start_state=self.start_state,
                goal_state=self.goal_state,
                obstacle_positions=self.obstacle_positions,
                obstacle_dimensions=self.obstacle_dimensions,
                check_collision_fn=self.check_collision,
                use_goal_reached_fn=True,
                check_goal_reached_fn=self.check_goal_reached,
            )
        else:
            self.path_planner.set_env(
                robot_uid=self.robot_uid,
                joint_ids=self.non_fixed_joint_ids,
                robot_state_ranges=self.robot_state_ranges,
                start_state=self.start_state,
                goal_state=self.goal_state,
                obstacle_positions=self.obstacle_positions,
                obstacle_dimensions=self.obstacle_dimensions,
                check_collision_fn=self.check_collision,
            )

    def plan_path(self):
        self.path, self.num_samples = self.path_planner.plan_path()
        return self.path, self.num_samples

    def plot_path(self, num_repeat=1):
        if self.path is None:
            print("Can't plot path: Path is None")
            return
        plot_utils.plot_path(
            path=self.path,
            joint_ids=self.non_fixed_joint_ids,
            robot_uid=self.robot_uid,
            plot_end_effector_poses=True,
            num_repeat=num_repeat,
        )

    def plot_path_label(self):
        if self.path_label is None:
            print("Can't plot path: Path is None")
            return
        plot_utils.plot_path(
            path=self.path_label,
            joint_ids=self.non_fixed_joint_ids,
            robot_uid=self.robot_uid,
            plot_end_effector_poses=True,
            num_repeat=2,
            marker_color=(1, 0, 0, 0.7),
        )
