from typing import List, Tuple
import time

import pybullet as p

import utils.bullet_obj_utils as bullet_obj_utils


def plot_start_and_goal_pos(
    start_ee_pos: List[float],
    goal_ee_pos: List[float],
    start_pos_color=[0, 1, 0, 1],
    goal_pos_color=[1, 0, 0, 1],
    text_size=2.5,
    marker_radius=0.04,
):
    start_dot_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        rgbaColor=start_pos_color,
        radius=marker_radius,
    )
    goal_dot_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        rgbaColor=goal_pos_color,
        radius=marker_radius,
    )

    start_dot_body = p.createMultiBody(
        baseVisualShapeIndex=start_dot_visual_shape, basePosition=start_ee_pos
    )
    goal_dot_body = p.createMultiBody(
        baseVisualShapeIndex=goal_dot_visual_shape, basePosition=goal_ee_pos
    )

    p.addUserDebugText(
        text="  start",
        textPosition=start_ee_pos,
        textColorRGB=[1, 1, 1],
        textSize=text_size,
    )

    p.addUserDebugText(
        text="  goal",
        textPosition=goal_ee_pos,
        textColorRGB=[1, 1, 1],
        textSize=text_size,
    )


def plot_rail(
    rail_length: float = 3.0, rail_width: float = 0.2, rail_color=[0.8, 0.8, 0.8, 1.0]
) -> int:
    rail_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[rail_length * 0.5, rail_width * 0.5, 0.001],
        rgbaColor=rail_color,
    )
    rail_uid = p.createMultiBody(
        baseVisualShapeIndex=rail_visual_shape,
        basePosition=[rail_length * 0.5, 0.0, 0.0],
    )
    return rail_uid


def plot_path_forever(
    path: List[List[float]],
    joint_uids: List[int],
    robot_uid: int,
    hold_time: float = 0.2,
    plot_end_effector_pos: bool = True,
    marker_radius=0.01,
    marker_color=[0, 0, 1, 0.5],
):
    while True:
        if plot_end_effector_pos:
            for state_on_path in path:
                bullet_obj_utils.set_joint_positions(
                    robot_uid, joint_uids, state_on_path
                )
                curr_end_effectgor_pos = bullet_obj_utils.get_end_effector_position(
                    robot_uid
                )
                # cur_world_pos = p.getLinkState(robot_uid, 3)[0]
                vs_id = p.createVisualShape(
                    p.GEOM_SPHERE, radius=marker_radius, rgbaColor=marker_color
                )
                p.createMultiBody(
                    basePosition=curr_end_effectgor_pos,
                    baseCollisionShapeIndex=-1,
                    baseVisualShapeIndex=vs_id,
                )
            plot_end_effector_pos = False

        for state_on_path in path:
            bullet_obj_utils.set_joint_positions(robot_uid, joint_uids, state_on_path)
            time.sleep(hold_time)
