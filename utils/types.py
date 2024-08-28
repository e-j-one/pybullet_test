from typing import Tuple

# RobotState represents the state of the robot in a tuple format.
# The tuple consists of the following 8 float elements:
#   - 2D position (x, y) in the plane
#   - 2D orientation (yaw) in radians
#   - Joint states for the robot's six joints (joint_state_0 to joint_state_5)
RobotState = Tuple[float, float, float, float, float, float, float, float]
