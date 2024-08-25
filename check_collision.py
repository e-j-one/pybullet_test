import pybullet as p
import pybullet_data


# Function to set joint angles and get end-effector position
def get_end_effector_position(joint_positions, robotUid):
    for i, joint_position in enumerate(joint_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robotUid,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_position,
        )

    # Step simulation to update the position
    p.stepSimulation()

    # Get the state of the end-effector link (ee_link)
    end_effector_state = p.getLinkState(
        bodyUniqueId=robotUid,
        linkIndex=7,  # The index of the end-effector link (usually the last link)
        computeForwardKinematics=True,
    )

    # The position of the end-effector is the first element of the link state
    end_effector_position = end_effector_state[4]

    return end_effector_position


def spawn_robot(urdf_path):
    robotUid = p.loadURDF(
        urdf_path,
        useFixedBase=True,
    )
    return robotUid


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


if __name__ == "__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

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

    spawn_robot(urdf_path=urdf_path)
    spawn_obstacles(
        obstacle_positions=obstacle_positions, obstacle_dimensions=obstacle_dimensions
    )

    # Keep the simulation running without gravity
    while p.isConnected():
        p.stepSimulation()
    pass
