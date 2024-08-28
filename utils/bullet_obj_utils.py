from typing import List

import pybullet as p


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


def set_joint_positions(
    robot_uid: int, joint_ids: List[int], joint_states: List[float]
):
    assert len(joint_ids) == len(
        joint_states
    ), "joint_ids and joint_states must have the same length"

    for joint_id, joint_values in zip(joint_ids, joint_states):
        p.resetJointState(robot_uid, joint_id, joint_values)


def set_base_and_joint_positions(
    robot_uid: int, joint_ids: List[int], robot_state: List[float]
):
    base_pose = robot_state[:3]
    joint_states = robot_state[3:]
    # base_orientation = p.getQuaternionFromEuler([0, 0, base_pose[2]])
    p.resetBasePositionAndOrientation(
        bodyUniqueId=robot_uid,
        posObj=base_pose,
        ornObj=[0, 0, 0, 1],  # base_orientation
    )
    set_joint_positions(
        robot_uid=robot_uid, joint_ids=joint_ids, joint_states=joint_states
    )


def get_non_fixed_joints(robot_uid):
    num_joints = p.getNumJoints(robot_uid)
    non_fixed_joints = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_uid, i)
        joint_type = joint_info[2]
        if joint_type != p.JOINT_FIXED:
            non_fixed_joints.append(i)
    return non_fixed_joints


def spawn_robot(urdf_path):
    robot_uid = p.loadURDF(
        urdf_path,
        basePosition=[0.0, 0.0, 1e-2],  # Slightly above the ground
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


def get_end_effector_position(robot_uid):
    # Get the state of the end-effector link (ee_link)
    end_effector_state = p.getLinkState(
        bodyUniqueId=robot_uid,
        linkIndex=6,  # The index of the end-effector link (usually the last link)
        computeForwardKinematics=True,
    )

    # The position of the end-effector is the first element of the link state
    end_effector_position = end_effector_state[4]

    return end_effector_position
