from typing import List, Tuple

import pybullet as p

from utils.types import RobotState

ROBOT_BASE_Z_COORD = 1e-2


def get_base_pose_and_joint_state_from_robot_state(
    robot_state: RobotState,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float], List[float]]:
    """
    Separate the base position, base orientation and joint states from the robot state of ur5 on rail.

    Returns
    -------
    base_pose, base_orientation, joint_states
    """
    # robot base moves on 1d rail.
    base_position = (robot_state[0], 0.0, ROBOT_BASE_Z_COORD)
    base_orientation = (0, 0, 0, 1)
    joint_states = robot_state[1:]
    return base_position, base_orientation, joint_states


def get_non_fixed_joints(robot_uid: int) -> List[int]:
    num_joints = p.getNumJoints(robot_uid)
    non_fixed_joints = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_uid, i)
        joint_type = joint_info[2]
        if joint_type != p.JOINT_FIXED:
            non_fixed_joints.append(i)
    return non_fixed_joints


def get_end_effector_position(robot_uid: int) -> Tuple[float, float, float]:
    # Get the state of the end-effector link (ee_link)
    end_effector_state = p.getLinkState(
        bodyUniqueId=robot_uid,
        linkIndex=6,  # The index of the end-effector link (usually the last link)
        computeForwardKinematics=True,
    )

    # The position of the end-effector is the first element of the link state
    end_effector_position = end_effector_state[4]

    return end_effector_position


def get_joint_limits(robot_uid: int, joint_ids: List[int]) -> List[Tuple[float, float]]:
    joint_limits = []
    for joint_id in joint_ids:
        joint_info = p.getJointInfo(robot_uid, joint_id)
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]
        # print(f"Joint {joint_id}: Lower limit: {lower_limit}, Upper limit: {upper_limit}")
        joint_limits.append((lower_limit, upper_limit))
    return joint_limits


def get_robot_state_ranges(
    robot_uid: int, joint_ids: List[int], rail_length: int, limit_shoulder_joint=True
) -> List[Tuple[float, float]]:
    robot_state_ranges = [(0, rail_length)]
    joint_limits = get_joint_limits(robot_uid, joint_ids)
    if limit_shoulder_joint:
        joint_limits[1] = (-3.141592, 0.0)
    robot_state_ranges += joint_limits
    return robot_state_ranges


def set_joint_positions(
    robot_uid: int, joint_ids: List[int], joint_states: List[float]
):
    assert len(joint_ids) == len(
        joint_states
    ), "joint_ids and joint_states must have the same length"

    for joint_id, joint_values in zip(joint_ids, joint_states):
        p.resetJointState(robot_uid, joint_id, joint_values)


def set_base_and_joint_positions(
    robot_uid: int, joint_ids: List[int], robot_state: RobotState
):
    base_position, base_orientation, joint_states = (
        get_base_pose_and_joint_state_from_robot_state(robot_state)
    )
    # base_orientation = p.getQuaternionFromEuler([0, 0, base_pose[2]])
    p.resetBasePositionAndOrientation(
        bodyUniqueId=robot_uid, posObj=base_position, ornObj=base_orientation
    )
    set_joint_positions(
        robot_uid=robot_uid, joint_ids=joint_ids, joint_states=joint_states
    )


def spawn_robot(urdf_path) -> int:
    robot_uid = p.loadURDF(
        urdf_path,
        basePosition=[0.0, 0.0, ROBOT_BASE_Z_COORD],  # Slightly above the ground
        useFixedBase=False,
    )
    return robot_uid


def spawn_obstacles(obstacle_positions, obstacle_dimensions) -> List[int]:
    obstacle_ids = []
    # Spawn cuboid obstacles with given positions and dimensions
    for position, dimensions in zip(obstacle_positions, obstacle_dimensions):
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX, halfExtents=[dim / 2 for dim in dimensions]
        )
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_BOX, halfExtents=[dim / 2 for dim in dimensions]
        )

        created_obstacle_id = p.createMultiBody(
            baseMass=0,  # Static object, mass = 0
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position,
        )
        obstacle_ids.append(created_obstacle_id)
    return obstacle_ids


def print_joint_positions(robot_uid):
    num_joints = p.getNumJoints(robot_uid)

    print("Joint positions for robot UID:", robot_uid)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_uid, i)
        # print("joint_info[8]:", joint_info[8])
        # print("joint_info[9]:", joint_info[9])
        joint_name = joint_info[1].decode("utf-8")
        joint_position = p.getJointState(robot_uid, i)[0]
        print(f"Joint {i}: {joint_name}, \tPosition: {joint_position:.3f}")
