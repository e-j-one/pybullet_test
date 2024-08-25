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


def set_joint_positions(robot_uid, joint_ids, joint_states):
    assert len(joint_ids) == len(joint_states)

    for joint_id, joint_values in zip(joint_ids, joint_states):
        p.resetJointState(robot_uid, joint_id, joint_values)


def get_non_fixed_joints(robot_uid):
    num_joints = p.getNumJoints(robot_uid)
    non_fixed_joints = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_uid, i)
        joint_type = joint_info[2]
        if joint_type != p.JOINT_FIXED:
            non_fixed_joints.append(i)
    return non_fixed_joints
