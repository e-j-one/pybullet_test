import numpy as np
from typing import List

import pybullet as p
import pybullet_data

from utils.collision_utils import get_collision_fn
import utils.bullet_obj_utils as bullet_obj_utils


def test_collision_check():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-130,
        # cameraPitch=-45,
        cameraTargetPosition=[0, 0, 0],
    )

    urdf_path = "urdf/ur5.urdf"

    joint_states = [
        [3.14, 0.0, 0.0, 0.0, 0.0, 0.0],
        [3.14, -np.pi * 0.125, 0.0, 0.0, 0.0, 0.0],
        [3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0],
        [3.14, -np.pi * 0.5, 0.0, 0.0, 0.0, 0.0],
        [3.14, -np.pi * 0.6, 0.0, 0.0, 0.0, 0.0],
    ]

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

    plane_uid = p.loadURDF("plane.urdf")
    robot_uid = bullet_obj_utils.spawn_robot(urdf_path=urdf_path)
    spawned_obstacle_uids = bullet_obj_utils.spawn_obstacles(
        obstacle_positions, obstacle_dimensions
    )

    obstacle_uids = [plane_uid] + spawned_obstacle_uids

    non_fixed_joint_ids = bullet_obj_utils.get_non_fixed_joints(robot_uid)

    is_collision = get_collision_fn(robot_uid, non_fixed_joint_ids, obstacle_uids)

    for joint_state in joint_states:
        print(is_collision(joint_state))

    while p.isConnected():
        pass
        # p.stepSimulation()


if __name__ == "__main__":
    test_collision_check()
