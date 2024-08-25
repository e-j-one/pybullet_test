import pybullet as p
import pybullet_data
import numpy as np

from collision_utils import get_collision_fn


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


def get_non_fixed_joints(robot_uid):
    num_joints = p.getNumJoints(robot_uid)
    non_fixed_joints = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_uid, i)
        joint_type = joint_info[2]
        if joint_type != p.JOINT_FIXED:
            non_fixed_joints.append(i)
    return non_fixed_joints


# Function to move the robot joints to a specified configuration
def move_robot_joints(joint_positions, robot_uid):
    for i, joint_position in enumerate(joint_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_uid,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_position,
        )
    # Step simulation to apply the new joint positions
    p.stepSimulation()


# Function to get the current position of the end-effector
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


def spawn_robot(urdf_path):
    robot_uid = p.loadURDF(
        urdf_path,
        basePosition=[0.0, 0.0, 1e-2],  # Slightly above the ground
        useFixedBase=True,
    )
    return robot_uid


def spawn_obstacles(obstacle_positions, obstacle_dimensions):
    # Spawn cuboid obstacles with given positions and dimensions
    for position, dimensions in zip(obstacle_positions, obstacle_dimensions):
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX, halfExtents=[dim / 2 for dim in dimensions]
        )
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_BOX, halfExtents=[dim / 2 for dim in dimensions]
        )

        p.createMultiBody(
            baseMass=0,  # Static object, mass = 0
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position,
        )


def interpolate_positions(start_pos, end_pos, steps):
    return [np.linspace(start, end, steps) for start, end in zip(start_pos, end_pos)]


def move_robot_joints(joint_positions, robot_uid):
    for i, joint_position in enumerate(joint_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_uid,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_position,
        )
    p.stepSimulation()


def check_collision(robot_uid):
    num_joints = p.getNumJoints(robot_uid)
    for i in range(num_joints):
        contacts = p.getContactPoints(robot_uid)
        if contacts:
            return True
    return False


def can_move_without_collision(
    start_pos, end_pos, robot_uid, steps=100, threshold=1e-3
):
    interpolated_positions = interpolate_positions(start_pos, end_pos, steps)

    for step in range(steps):
        joint_positions = [pos[step] for pos in interpolated_positions]
        move_robot_joints(joint_positions, robot_uid)

        if check_collision(robot_uid):
            print(f"Collision detected at step {step}!")
            return False

        # Optional: Ensure convergence after each step
        converged = False
        for _ in range(100):
            p.stepSimulation()
            current_positions = [
                p.getJointState(robot_uid, i)[0] for i in range(len(joint_positions))
            ]
            errors = [
                abs(current_positions[i] - joint_positions[i])
                for i in range(len(joint_positions))
            ]
            if all(error < threshold for error in errors):
                converged = True
                break
        if not converged:
            print(f"Failed to converge at step {step}.")
            return False

    print("Movement is possible without collision.")
    return True


def main():
    """
    1. spawn floor and obstacles
    2.
    """
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,  # Distance from the target position
        cameraYaw=45,  # Yaw angle in degrees
        cameraPitch=-130,  # Pitch angle in degrees
        cameraTargetPosition=[0, 0, 0],  # Target position (x, y, z)
    )

    urdf_path = "urdf/ur5.urdf"

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
    robot_uid = spawn_robot(urdf_path=urdf_path)

    # Load the cuboid URDF
    cuboid_id = p.loadURDF(
        "cube_small.urdf",  # URDF file for the cuboid
        basePosition=[0.0, 0.0, 0.5],
        # baseOrientation=orientation,
    )

    print("robot uid:", robot_uid)
    obstacles_uid = [plane_uid, cuboid_id]

    # spawn_obstacles(
    #     obstacle_positions=obstacle_positions, obstacle_dimensions=obstacle_dimensions
    # )

    # joint_positions = [1.57, 0.0, 0.0, 0.0, 0.0, 0.0]

    # move_robot_joints(joint_positions=joint_positions, robot_uid=robot_uid)
    # print_joint_positions(robot_uid=robot_uid)

    # ee_pose = get_end_effector_position(robot_uid=robot_uid)
    # print("End-effector position:", ee_pose)

    start_pos = [0.0, -np.pi * 0.25, 0.0, 0.0, 0.0, 0.0]
    end_pos = [1.57, -np.pi * 0.25, 0.0, 0.0, 0.0, 0.0]
    pos_3 = [3.14, -np.pi * 0.5, 0.0, 0.0, 0.0, 0.0]
    pos_4 = [3.14, -np.pi * 0.25, 0.0, 0.0, 0.0, 0.0]

    # if can_move_without_collision(start_pos, end_pos, robot_uid, steps=100):
    #     print("The robot can move from start to end without collision.")
    # else:
    #     print("The robot cannot move from start to end without collision.")

    # for i in range(1000):
    #     p.stepSimulation()

    print_joint_positions(robot_uid=robot_uid)

    non_fixed_joint_uids = get_non_fixed_joints(robot_uid)

    is_collision = get_collision_fn(robot_uid, non_fixed_joint_uids, obstacles_uid)

    print(is_collision(start_pos))
    print(is_collision(end_pos))
    print(is_collision(pos_3))
    print(is_collision(pos_4))
    print(is_collision(pos_3))
    print(is_collision(pos_4))

    while p.isConnected():
        p.stepSimulation()


if __name__ == "__main__":
    main()
