import os
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

pandaUid = p.loadURDF(
    "urdf/ur5.urdf",
    useFixedBase=True,
)

while True:
    p.stepSimulation()
