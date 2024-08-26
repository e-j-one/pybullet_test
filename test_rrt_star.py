import numpy as np
from typing import List
import time

import pybullet as p
import pybullet_data

from utils.collision_utils import get_collision_fn

import utils.bullet_obj_utils as bullet_obj_utils
import utils.plot_utils as plot_utils


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

    start_state = np.array([3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0])

    start_ee_pos = [-0.343, -0.191, 0.847]
    goal_ee_pos = [0.162, -0.192, 0.906]

    plot_utils.plot_start_and_goal_pos(start_ee_pos, goal_ee_pos)

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

    # joint_states = [
    #     # [3.14, 0.0, 0.0, 0.0, 0.0, 0.0],
    #     # [3.14, -np.pi * 0.125, 0.0, 0.0, 0.0, 0.0],
    #     [3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0],
    #     [3.14, -np.pi * 0.5, 0.0, 0.0, 0.0, 0.0],
    #     [3.14, -np.pi * 0.6, 0.0, 0.0, 0.0, 0.0],
    # ]

    is_collision = get_collision_fn(robot_uid, non_fixed_joint_uids, obstacle_uids)

    # for joint_state in joint_states:
    #     print(is_collision(joint_state))
    #     print("ee pose: ", bullet_obj_utils.get_end_effector_position(robot_uid))

    # while p.isConnected():
    #     pass
    #     # p.stepSimulation()

    rrt_path = [
        np.array([3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0]),
        np.array([3.14, -np.pi * 0.5, 0.0, 0.0, 0.0, 0.0]),
        np.array([3.14, -np.pi * 0.6, 0.0, 0.0, 0.0, 0.0]),
    ]

    draw_path = False

    while True:
        if not draw_path:
            for state_on_path in rrt_path:
                bullet_obj_utils.set_joint_positions(
                    robot_uid, non_fixed_joint_uids, state_on_path
                )

                curr_end_effectgor_pos = bullet_obj_utils.get_end_effector_position(
                    robot_uid
                )
                # cur_world_pos = p.getLinkState(robot_uid, 3)[0]
                plot_utils.draw_sphere_marker(
                    curr_end_effectgor_pos, 0.02, [1, 0, 0, 1]
                )
            draw_path = True

        for state_on_path in rrt_path:
            bullet_obj_utils.set_joint_positions(
                robot_uid, non_fixed_joint_uids, state_on_path
            )
            time.sleep(0.3)


if __name__ == "__main__":
    test_collision_check()
