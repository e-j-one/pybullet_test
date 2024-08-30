from typing import Tuple


RobotState = Tuple[float, float, float, float, float, float, float]
"""
RobotState represents the state of the robot in a tuple format.

- The tuple consists of 7 float elements:
  - 1D position (x) in the plane
  - Joint states for the robot's six joints (joint_state_0 to joint_state_5)
"""
