import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

pandaUid = p.loadURDF(
    "urdf/ur5.urdf",
    useFixedBase=True,
)

obstacle_positions = [
    [0.5, 0.0, 0.0],
    [0.0, 0.5, 0.0],
    [0.0, 0.0, 0.5],
]

obstacle_dimensions = [
    [0.2, 0.1, 0.1],
    [0.1, 0.3, 0.1],
    [0.1, 0.1, 0.4],
]

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

# Keep the simulation running without gravity
while p.isConnected():
    p.stepSimulation()
