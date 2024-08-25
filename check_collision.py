import pybullet as p
import pybullet_data


def print_joint_positions(robotUid):
    num_joints = p.getNumJoints(robotUid)

    print("Joint positions for robot UID:", robotUid)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robotUid, i)
        joint_name = joint_info[1].decode("utf-8")
        joint_position = p.getJointState(robotUid, i)[0]
        print(f"Joint {i}: {joint_name}, \tPosition: {joint_position:.3f}")


# Function to move the robot joints to a specified configuration
def move_robot_joints(joint_positions, robotUid):
    for i, joint_position in enumerate(joint_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robotUid,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_position,
        )
    # Step simulation to apply the new joint positions
    p.stepSimulation()


# Function to get the current position of the end-effector
def get_end_effector_position(robotUid):
    # Get the state of the end-effector link (ee_link)
    end_effector_state = p.getLinkState(
        bodyUniqueId=robotUid,
        linkIndex=6,  # The index of the end-effector link (usually the last link)
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


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,  # Distance from the target position
        cameraYaw=30,  # Yaw angle in degrees
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

    robotUid = spawn_robot(urdf_path=urdf_path)
    spawn_obstacles(
        obstacle_positions=obstacle_positions, obstacle_dimensions=obstacle_dimensions
    )

    joint_positions = [-1.57, 0.0, 0.0, 0.0, 0.0, 0.0]

    move_robot_joints(joint_positions=joint_positions, robotUid=robotUid)
    print_joint_positions(robotUid=robotUid)

    ee_pose = get_end_effector_position(robotUid=robotUid)
    print("End-effector position:", ee_pose)

    while p.isConnected():
        p.stepSimulation()


if __name__ == "__main__":
    main()
