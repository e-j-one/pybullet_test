from itertools import product
from typing import List

import pybullet as p

from utils.bullet_obj_utils import set_joint_positions


def is_joint_state_in_limits(robot_uid: int, joint_states: List[float]) -> bool:
    for i in range(len(joint_states)):
        joint_info = p.getJointInfo(robot_uid, i)
        joint_position = joint_states[i]
        joint_lower_limit = joint_info[8]
        joint_upper_limit = joint_info[9]
        if joint_position < joint_lower_limit or joint_position > joint_upper_limit:
            return False
    return True


def is_link_in_collision(robot_uid, link1, link2, max_distance=0.0):
    return (
        len(
            p.getClosestPoints(
                bodyA=robot_uid,
                bodyB=robot_uid,
                distance=max_distance,
                linkIndexA=link1,
                linkIndexB=link2,
            )
        )
        != 0
    )


def is_body_in_collision(body_a, body_b, max_distance=0.0):
    return (
        len(p.getClosestPoints(bodyA=body_a, bodyB=body_b, distance=max_distance)) != 0
    )


def get_non_adjacent_pairs(robot_uid):
    """
    Joint is assumed to be adjacent if the
    """
    num_joints = p.getNumJoints(robot_uid)
    non_adjacent_pairs = [
        (i, j) for i in range(num_joints) for j in range(i + 2, num_joints)
    ]
    return non_adjacent_pairs


def get_collision_fn(
    robot_uid,
    joint_ids,
    obstacles,
):
    non_adjacent_pairs = get_non_adjacent_pairs(robot_uid)

    print("non_adjacent_pairs:", non_adjacent_pairs)

    robot_body_uids = [robot_uid]
    check_body_uid_pairs = list(product(robot_body_uids, obstacles))

    print("============================================")
    print("========== non_adjacent_pairs:", non_adjacent_pairs)
    print("========== check_body_uid_pairs:", check_body_uid_pairs)

    def is_collision_fn(q):
        if not is_joint_state_in_limits(robot_uid, q):
            print("Joint state is out of limits")
            return True

        set_joint_positions(robot_uid, joint_ids, q)

        for link1, link2 in non_adjacent_pairs:
            if is_link_in_collision(robot_uid, link1, link2):
                print(f"Link {link1} and Link {link2} are in collision")
                return True

        for pair in check_body_uid_pairs:
            if is_body_in_collision(pair[0], pair[1]):
                print(f"Body {pair[0]} and Body {pair[1]} are in collision")
                return True

    return is_collision_fn
