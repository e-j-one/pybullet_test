import pybullet as p
import pybullet_data
import numpy as np
from typing import List

from utils.collision_utils import get_collision_fn
import utils.bullet_obj_utils as bullet_obj_utils


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


# def interpolate_positions(start_pos, end_pos, steps):
#     return [np.linspace(start, end, steps) for start, end in zip(start_pos, end_pos)]


def move_robot_joints(robot_uid, joint_positions):
    for i, joint_position in enumerate(joint_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_uid,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_position,
        )
    # p.stepSimulation()


def check_collision(robot_uid):
    # check if robot body is in contact with multiple bodies
    contact_points = p.getContactPoints(bodyA=robot_uid)
    return len(contact_points) > 0


def test_collision_obj():
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
    robot_uid = bullet_obj_utils.spawn_robot(urdf_path=urdf_path)
    spawned_obstacle_ids = spawn_obstacles(obstacle_positions, obstacle_dimensions)

    obstacle_ids = [plane_uid] + spawned_obstacle_ids

    non_fixed_joint_uids = bullet_obj_utils.get_non_fixed_joints(robot_uid)

    pos_4 = [0.0, -np.pi * 0.5, 0.0, 0.0, 0.0, 0.0]
    # pos_4 = [3.14, -np.pi * 0.5, 0.0, 0.0, 0.0, 0.0]

    # move_robot_joints(robot_uid=robot_uid, joint_positions=pos_4)
    bullet_obj_utils.set_joint_positions(
        robot_uid=robot_uid, joint_ids=non_fixed_joint_uids, joint_states=pos_4
    )
    # print("collision: ", check_collision(robot_uid=robot_uid))
    from utils.collision_utils import is_body_in_collision

    for obstacle_id in obstacle_ids:
        print(
            "collision with obstacle ",
            obstacle_id,
            ": ",
            is_body_in_collision(robot_uid, obstacle_id),
        )

    while p.isConnected():
        pass
        p.stepSimulation()


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

    plane_uid = p.loadURDF("plane.urdf")
    robot_uid = bullet_obj_utils.spawn_robot(urdf_path=urdf_path)

    # Load the cuboid URDF
    cuboid_id = p.loadURDF(
        "cube_small.urdf",  # URDF file for the cuboid
        basePosition=[0.0, 0.0, 0.5],
        # baseOrientation=orientation,
    )

    print("robot uid:", robot_uid)
    obstacles_uid = [plane_uid, cuboid_id]

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

    bullet_obj_utils.print_joint_positions(robot_uid=robot_uid)

    non_fixed_joint_uids = bullet_obj_utils.get_non_fixed_joints(robot_uid)

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
    test_collision_obj()
    # main()
