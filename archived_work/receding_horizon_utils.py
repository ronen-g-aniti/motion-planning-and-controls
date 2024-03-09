from environment import Environment
import numpy as np


def construct_voxmap(local_position: np.ndarray, environment: Environment):
	
"""
# Build a voxel array around the current position
voxel_map = self.build_voxel_map()
# Establish the start position as the current position
current_pos_in_voxel = (20, 20, 10)
# Mark the grid cell that is the closest cell to the active waypoint
goal_voxel_index = self._determine_goal_voxel_index()
# Use A* grid search to find a path of grid cells to that location
path_of_voxel_indices = self._determine_path_through_voxel()
# Set the target location to be the local position of the grid cell that's furthest
# from the current location but that's also not in collision with a straight line path
# from the current position. 
target_index = self._determine_target_voxel_index()
# Update the receding horizon target position to be the local position of that grid cell
target_position = (-100, 100, 29, 0)
class VoxelMap:
	def __init__(self, environment, local_position, goal_position):
		self._north_size = 40
		self._east_size = 40
		self._down_size = 20
		self._voxmap = self._build_voxmap_around_current_position()


def build_voxel_map(environment: Environment, local_position: np.ndarray) -> np.ndarray:
	pass
def determine_goal_voxel_index(voxel_map, target_location)

"""
