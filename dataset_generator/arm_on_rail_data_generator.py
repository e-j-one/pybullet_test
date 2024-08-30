from typing import List, Tuple
import numpy as np

import pybullet as p
import pybullet_data

import utils.bullet_obj_utils as bullet_obj_utils
import utils.geometry_utils as geometry_utils
from utils.collision_utils import get_check_collision_fn
from utils.types import RobotState


class ArmOnRailDataGenerator:
    """
    Create a map with obstacles, start and goal positions
    - map_size: height and width of the map [float, float]
    - obstacle_map: map with obstacles info only
    - full_map: map with obstacles, start and goal info
    Map representation
    - 0: free space
    - 1: obstacle
    - 2: start position
    - 3: goal position
    """

    # map_size: height and width of the map [float, float]
    def __init__(self):
        pass

    def set_config(
        self,
        rail_length: float,
        num_obstacles: int,
        min_state_cost_threshold: float = 0.5,
        min_r_obstacle: float = 0.4,
        max_r_obstacle: float = 1.0,
        min_phi_obstacle: float = 0.0,
        max_phi_obstacle: float = np.pi,
        min_length_obstacle: float = 0.1,
        max_length_obstacle: float = 0.4,
        robot_urdf_path="urdf/ur5.urdf",
    ):
        self.rail_length = rail_length
        self.num_obstacles = num_obstacles
        self.min_state_cost_threshold = min_state_cost_threshold

        self.min_r_obstacle = min_r_obstacle
        self.max_r_obstacle = max_r_obstacle
        self.min_phi_obstacle = min_phi_obstacle
        self.max_phi_obstacle = max_phi_obstacle
        self.min_length_obstacle = min_length_obstacle
        self.max_length_obstacle = max_length_obstacle
        self.robot_urdf_path = robot_urdf_path

        self.num_joints = 6
        self.num_max_state_tries = int(1e5)

    def _sample_robot_state(self) -> RobotState:
        """
        Sample a random robot state
        """
        # 2d_x in [0, rail_length], 2d_y = 0, 2d_yaw = 0
        state = []
        for i in range(self.num_joints + 1):
            if i == 0:
                state.append(np.random.uniform(0.0, self.rail_length))
                continue
            if i == 2:
                state.append(np.random.uniform(-np.pi, 0.0))
                continue
            state.append(np.random.uniform(-np.pi, np.pi))
        return tuple(state)

    def sample_new_map_data(self) -> Tuple[
        RobotState,
        RobotState,
        Tuple[float, float, float],
        float,
        List[Tuple[float, float, float]],
        List[Tuple[float, float, float]],
    ]:
        """
        1. Sample start position until it is collision free.
        2. Sample goal position until it is collision free and cost > min_state_cost_threshold.
        3. Sample obstacles until collision free.

        Returns
        -------
        start_state, goal_state, goal_end_effector_pos, rail_length, obstacle_positions, obstacle_dimensions
        """

        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_uid = p.loadURDF("plane.urdf")
        self.robot_uid = bullet_obj_utils.spawn_robot(urdf_path=self.robot_urdf_path)
        obstacle_uids = [self.plane_uid]
        non_fixed_joint_ids = bullet_obj_utils.get_non_fixed_joints(self.robot_uid)

        check_collision = get_check_collision_fn(
            robot_uid=self.robot_uid,
            joint_ids=non_fixed_joint_ids,
            obstacles=obstacle_uids,
            rail_length=self.rail_length,
        )

        num_tries = 0
        while num_tries < self.num_max_state_tries:
            start_state = self._sample_robot_state()
            if not check_collision(start_state):
                break
            num_tries += 1

        print("start_state found after ", num_tries, " tries ", start_state)

        while num_tries < self.num_max_state_tries:
            goal_state = self._sample_robot_state()
            if (
                not check_collision(goal_state)
                and geometry_utils.get_cost_between_states(start_state, goal_state)
                > self.min_state_cost_threshold
            ):
                break
            num_tries += 1

        print("goal_state found after ", num_tries, " tries ", goal_state)
        goal_end_effector_pos = (
            bullet_obj_utils.get_end_effector_position_from_robot_state(
                self.robot_uid, non_fixed_joint_ids, goal_state
            )
        )

        obstacle_positions = []
        obstacle_dimensions = []

        while num_tries < self.num_max_state_tries:
            obstacle_positions = []
            obstacle_dimensions = []
            for _ in range(self.num_obstacles):
                r_obstacle = np.random.uniform(self.min_r_obstacle, self.max_r_obstacle)
                phi_obstacle = np.random.uniform(
                    self.min_phi_obstacle, self.max_phi_obstacle
                )
                x_obstacle = np.random.uniform(0, self.rail_length)
                y_obstacle = r_obstacle * np.cos(phi_obstacle)
                z_obstacle = r_obstacle * np.sin(phi_obstacle)
                width_obstacle, length_obstacle, height_obstacle = (
                    np.random.uniform(
                        self.min_length_obstacle, self.max_length_obstacle
                    ),
                    np.random.uniform(
                        self.min_length_obstacle, self.max_length_obstacle
                    ),
                    np.random.uniform(
                        self.min_length_obstacle, self.max_length_obstacle
                    ),
                )

                obstacle_positions.append([x_obstacle, y_obstacle, z_obstacle])
                obstacle_dimensions.append(
                    [width_obstacle, length_obstacle, height_obstacle]
                )

            spawned_obstacle_uids = bullet_obj_utils.spawn_obstacles(
                obstacle_positions, obstacle_dimensions
            )

            check_collision = get_check_collision_fn(
                robot_uid=self.robot_uid,
                joint_ids=non_fixed_joint_ids,
                obstacles=spawned_obstacle_uids,
                rail_length=self.rail_length,
            )

            if not check_collision(start_state) and not check_collision(goal_state):
                break
            for obstacle_uid in spawned_obstacle_uids:
                p.removeBody(obstacle_uid)
            num_tries += 1
        print(
            "obstacle_positions found after ", num_tries, " tries ", obstacle_positions
        )

        p.disconnect()

        return (
            start_state,
            goal_state,
            goal_end_effector_pos,
            self.rail_length,
            obstacle_positions,
            obstacle_dimensions,
        )

    def get_env_config_demo(self) -> Tuple[
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
