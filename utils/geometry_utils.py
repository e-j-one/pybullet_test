import numpy as np

from utils.types import RobotState


def move_state_towards_target(
    start_state: RobotState,
    target_state: RobotState,
    step_size: float,
) -> RobotState:
    """
    Move the start state towards the target state by step_size
    If the distance between the start and target state is less than step_size, return the target state
    """
    delta = np.array(target_state) - np.array(start_state)
    delta_norm = np.linalg.norm(delta)
    if delta_norm < step_size:
        return target_state
    progressed_state = start_state + step_size * delta / delta_norm
    return tuple(progressed_state)


def get_distance_between_states(
    state_i: RobotState,
    state_j: RobotState,
) -> float:
    return np.linalg.norm(np.array(state_i) - np.array(state_j))


def get_cost_between_states(
    state_i: RobotState,
    state_j: RobotState,
) -> float:
    return get_distance_between_states(state_i, state_j)
