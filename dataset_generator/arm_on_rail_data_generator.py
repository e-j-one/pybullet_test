from typing import List, Tuple
import numpy as np


from utils.types import RobotState


class ArmOnRailDataGenerator:
    """
    Create a map with obstacles, start and goal positions
    - map_size: height and width of the map [float, float]
    - obstacle_map: map with obstacles info only
    - full_map: map with obstacles, start and goal info
    Map representation
    - 0: free space
    - 1: obstacle
    - 2: start position
    - 3: goal position
    """

    # map_size: height and width of the map [float, float]
    def __init__(self):
        pass

    def set_config(self, rail_length: float, num_obstacles: int):
        self.rail_length = rail_length
        self.num_obstacles = num_obstacles

    def get_env_config_demo(self) -> Tuple[
        RobotState,
        RobotState,
        float,
        List[Tuple[float, float, float]],
        List[Tuple[float, float, float]],
    ]:
        """
        Return a env config for testing.
        In rail env, 2d_x in [0, rail_length], 2d_y = 0, 2d_yaw = 0 for robot state.

        Returns
        -------
        start_state, goal_state, rail_length, obstacle_positions, obstacle_dimensions

        state: np.ndarray (2d_x, 2d_y, 2d_yaw, joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        """
        start_state = (0.0, 3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0)
        goal_state = (3.0, 3.14, -np.pi * 0.4, 0.0, 0.0, 0.0, 0.0)
        rail_length = 4.0
        # goal_ee_pos = [0.162, -0.192, 0.906]

        obstacle_positions = [
            # [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [0.0, 0.0, 0.5],
            # [1.5, 0.0, 0.5],
            [1.5, -1.5, 0.8],
        ]
        obstacle_dimensions = [
            # [0.2, 0.1, 0.1],
            [0.1, 0.3, 0.1],
            [0.1, 0.1, 0.4],
            # [0.1, 0.1, 0.4],
            [0.1, 3.0, 0.4],
        ]
        return (
            start_state,
            goal_state,
            rail_length,
            obstacle_positions,
            obstacle_dimensions,
        )

    # def reset_map(self):
    #     """
    #     Reset maps
    #     - obstacle_map: map with obstacles info only
    #     - full_map: map with obstacles, start and goal info
    #     """
    #     self.obstacle_map = np.zeros((self.width, self.height), dtype=np.int8)
    #     self.full_map = np.zeros((self.width, self.height), dtype=np.int8)
    #     # map name with datetime
    #     self.map_name = "map_" + datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    #     # print("Map reset successfully. size: ", self.map.shape)

    # def generate_obstacles(self):
    #     pass

    # def generate_vertical_walls(
    #     self, num_walls: int = 0, max_length: int = 20, max_width: int = 10
    # ):
    #     if num_walls == 0:
    #         num_walls = np.random.randint(1, 10)
    #     # print("Generating {} walls".format(num_walls))

    #     for i in range(num_walls):
    #         x = np.random.randint(0, self.width)
    #         y = np.random.randint(0, self.height)

    #         wall_length = np.random.randint(1, max_length)
    #         wall_width = np.random.randint(1, max_width)
    #         direction = np.random.randint(0, 2)

    #         if direction == 0:
    #             # horizontal wall
    #             if x + wall_length > self.width:
    #                 x = self.width - wall_length
    #             self.obstacle_map[x : x + wall_length, y : y + wall_width] = 1
    #         else:
    #             # vertical wall
    #             if y + wall_length > self.height:
    #                 y = self.height - wall_length
    #             self.obstacle_map[x : x + wall_width, y : y + wall_length] = 1

    # def _sample_start_goal(self, max_tries: int = 1000):
    #     """
    #     Sample start and goal position in obstacle free space
    #     """
    #     self.full_map = self.obstacle_map.copy()
    #     start_idx = (
    #         np.random.randint(0, self.width),
    #         np.random.randint(0, self.height),
    #     )
    #     goal_idx = (np.random.randint(0, self.width), np.random.randint(0, self.height))

    #     num_tries = 0

    #     while self.full_map[start_idx] == 1 and num_tries < max_tries:
    #         start_idx = (
    #             np.random.randint(0, self.width),
    #             np.random.randint(0, self.height),
    #         )
    #         num_tries += 1

    #     while self.full_map[goal_idx] == 1 and num_tries < max_tries:
    #         goal_idx = (
    #             np.random.randint(0, self.width),
    #             np.random.randint(0, self.height),
    #         )
    #         num_tries += 1

    #     if self.full_map[start_idx] == 1 or self.full_map[goal_idx] == 1:
    #         print("Failed to find start and goal")
    #         return None

    #     self.full_map[start_idx] = 2
    #     self.full_map[goal_idx] = 3

    #     # print(
    #     #     "Start and goal position sampled start: {}, goal: {}".format(
    #     #         start_idx, goal_idx
    #     #     )
    #     # )
    #     self.start_idx = start_idx
    #     self.goal_idx = goal_idx
    #     return start_idx, goal_idx

    # def sample_start_goal_position(self, max_tries: int = 1000):
    #     start_idx, goal_idx = self._sample_start_goal(max_tries)
    #     start_position = (start_idx[0] + 0.5, start_idx[1] + 0.5)
    #     goal_position = (goal_idx[0] + 0.5, goal_idx[1] + 0.5)
    #     return start_position, goal_position

    # def get_start_goal_pos(self):
    #     if len(self.start_idx) > 0 and len(self.goal_idx) > 0:
    #         return self.start_idx, self.goal_idx
    #     elif self.full_map is not None:
    #         for i in range(self.full_map.shape[0]):
    #             for j in range(self.full_map.shape[1]):
    #                 if self.full_map[i, j] == 2:
    #                     start_idx = (i, j)
    #                 elif self.full_map[i, j] == 3:
    #                     goal_idx = (i, j)
    #         start_position = (start_idx[0] + 0.5, start_idx[1] + 0.5)
    #         goal_position = (goal_idx[0] + 0.5, goal_idx[1] + 0.5)
    #         return start_position, goal_position
    #     else:
    #         print("Map is not generated yet")
    #         return None

    # def get_obstacle_map(self):
    #     return self.obstacle_map

    # def get_full_map(self):
    #     return self.full_map

    # def save_full_map(self):
    #     self._save_map(self.full_map)

    # def save_obstacle_map(self):
    #     self._save_map(self.obstacle_map)

    # def _save_map(self, map_to_save, file_name: Optional[str] = None):
    #     if file_name:
    #         np.save(file_name, map_to_save)
    #     else:
    #         np.save("saved_maps/" + self.map_name, map_to_save)

    # def load_obstacle_map(self, file_name: str):
    #     self.obstacle_map = np.load(file_name)

    # def load_full_map(self, file_name: str):
    #     self.full_map = np.load(file_name)

    #     self.width = self.full_map.shape[0]
    #     self.height = self.full_map.shape[1]

    #     self.start_idx = []
    #     self.goal_idx = []

    # def set_full_map(self, full_map: np.ndarray, set_obstacle_map: bool = True):
    #     self.full_map = full_map

    #     self.width = self.full_map.shape[0]
    #     self.height = self.full_map.shape[1]

    #     self.start_idx = []
    #     self.goal_idx = []

    #     if set_obstacle_map:
    #         self._set_obstacle_map_from_full_map()

    # def _set_obstacle_map_from_full_map(self):
    #     self.obstacle_map = self.full_map.copy()
    #     self.obstacle_map[self.obstacle_map == 2] = 0
    #     self.obstacle_map[self.obstacle_map == 3] = 0

    # def plot_obstacle_map(self):
    #     plt.imshow(self.obstacle_map, cmap="gray")
    #     plt.show()

    # def plot_full_map(self):
    #     plt.imshow(self.full_map, cmap="gray")
    #     plt.show()
