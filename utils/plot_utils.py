from typing import List, Tuple

import pybullet as p


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
