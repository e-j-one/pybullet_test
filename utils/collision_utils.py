from itertools import product
from typing import List, Tuple

import pybullet as p

import utils.bullet_obj_utils as bullet_obj_utils
from utils.types import RobotState


def is_joint_state_in_limits(robot_uid: int, joint_states: List[float]) -> bool:
    for i in range(len(joint_states)):
        joint_info = p.getJointInfo(robot_uid, i)
        joint_position = joint_states[i]
        joint_lower_limit = joint_info[8]
        joint_upper_limit = joint_info[9]
        if joint_position < joint_lower_limit or joint_position > joint_upper_limit:
            return False
    return True


def is_link_in_collision(robot_uid: int, link_a: int, link_b: int, max_distance=0.0):
    return (
        len(
            p.getClosestPoints(
                bodyA=robot_uid,
                bodyB=robot_uid,
                distance=max_distance,
                linkIndexA=link_a,
                linkIndexB=link_b,
            )
        )
        != 0
    )


def is_body_in_collision(body_a: int, body_b: int, max_distance: float = 0.0) -> bool:
    return (
        len(p.getClosestPoints(bodyA=body_a, bodyB=body_b, distance=max_distance)) != 0
    )


def get_non_adjacent_pairs(robot_uid: int) -> List[Tuple[int, int]]:
    """
    Joint is assumed to be adjacent if the
    """
    num_joints = p.getNumJoints(robot_uid)
    non_adjacent_pairs = [
        (i, j) for i in range(num_joints) for j in range(i + 2, num_joints)
    ]
    return non_adjacent_pairs


def get_check_collision_fn(
    robot_uid: int, joint_ids: List[int], obstacles: List[int], rail_length: float
):
    non_adjacent_pairs = get_non_adjacent_pairs(robot_uid)

    # print("non_adjacent_pairs:", non_adjacent_pairs)

    robot_body_uids = [robot_uid]
    check_body_uid_pairs = list(product(robot_body_uids, obstacles))

    # print("============================================")
    # print("========== non_adjacent_pairs:", non_adjacent_pairs)
    # print("========== check_body_uid_pairs:", check_body_uid_pairs)
    # print("============================================")

    def is_collision_fn(robot_state: RobotState):
        base_position, base_orientation, joint_states = (
            bullet_obj_utils.get_base_pose_and_joint_state_from_robot_state(robot_state)
        )

        if base_position[0] < 0 or base_position[0] > rail_length:
            # print("Robot is out of rail")
            return True

        if not is_joint_state_in_limits(robot_uid=robot_uid, joint_states=joint_states):
            print("Joint state is out of limits")
            return True

        bullet_obj_utils.set_base_and_joint_positions(
            robot_uid=robot_uid, joint_ids=joint_ids, robot_state=robot_state
        )

        for link1, link2 in non_adjacent_pairs:
            if is_link_in_collision(robot_uid, link1, link2):
                print(f"Link {link1} and Link {link2} are in collision")
                return True

        for pair in check_body_uid_pairs:
            if is_body_in_collision(pair[0], pair[1]):
                print(f"Body {pair[0]} and Body {pair[1]} are in collision")
                return True

    return is_collision_fn
