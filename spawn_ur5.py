import os
import time

import numpy as np
import pybullet as p
import pybullet_data


import utils.bullet_obj_utils as bullet_obj_utils

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-45,
    cameraTargetPosition=[0, 0, 0],
)

ur5_uid = p.loadURDF(
    "urdf/ur5.urdf",
    useFixedBase=True,
)
init_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

non_fixed_joint_ids = bullet_obj_utils.get_non_fixed_joints(ur5_uid)
bullet_obj_utils.set_joint_positions(ur5_uid, non_fixed_joint_ids, init_joint_states)

joint_angles_candidates_positive = np.linspace(0, np.pi, 10)
joint_angles_candidates_negative = np.linspace(0, -np.pi, 10)


for i_joint in range(len(init_joint_states)):
    for angle in joint_angles_candidates_positive:
        joint_states = init_joint_states.copy()
        joint_states[i_joint] = angle
        print("angle:", angle)
        print(joint_states)
        bullet_obj_utils.set_joint_positions(ur5_uid, non_fixed_joint_ids, joint_states)
        time.sleep(0.2)

    for angle in joint_angles_candidates_negative:
        joint_states = init_joint_states.copy()
        joint_states[i_joint] = angle
        print("angle:", angle)
        print(joint_states)
        bullet_obj_utils.set_joint_positions(ur5_uid, non_fixed_joint_ids, joint_states)
        time.sleep(0.2)


while True:
    p.stepSimulation()
