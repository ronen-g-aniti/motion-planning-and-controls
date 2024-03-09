from enum import Enum
from environment import Environment
from state import State
from local_position import LocalPosition
import matplotlib.pyplot as plt
from planner import Planner
from typing import List, Tuple
import pdb
import numpy as np
import copy
from queue import PriorityQueue
import scipy.ndimage
from scipy.spatial import KDTree
from mpl_toolkits.mplot3d import Axes3D
import time
import itertools


class HeightMap:
	def __init__(self, environment: Environment, start_state: State, goal_state: State):
		self._environment = environment
		self._start_state = start_state
		self._goal_state = goal_state
		self._north_size = int(self._environment.north_bounds.maximum - self._environment.north_bounds.minimum)
		self._east_size = int(self._environment.east_bounds.maximum - self._environment.east_bounds.minimum)
		self._north_offset = self._environment.obstacles.north_offset
		self._east_offset = self._environment.obstacles.east_offset


		self._start_grid_north = int(self._start_state.local_position.north - self._north_offset)
		self._start_grid_east = int(self._start_state.local_position.east - self._east_offset)
		self._start_grid_down = int(self._start_state.local_position.down)
		self._goal_grid_north = int(self._goal_state.local_position.north - self._north_offset)
		self._goal_grid_east = int(self._goal_state.local_position.east - self._east_offset)
		self._goal_grid_down = int(self._goal_state.local_position.down)
		self._start_grid = (self._start_grid_north, self._start_grid_east, self._start_grid_down)
		self._goal_grid = (self._goal_grid_north, self._goal_grid_east, self._goal_grid_down)


		self._height_map = np.zeros((self._north_size, self._east_size))
		for obstacle in self._environment.obstacles.list:
			north_start = int(obstacle.north_min - self._north_offset)
			north_end = int(obstacle.north_max - self._north_offset)
			east_start = int(obstacle.east_min - self._east_offset)
			east_end = int(obstacle.east_max - self._east_offset)
			self._height_map[north_start: north_end, east_start: east_end] = obstacle.height

		plt.imshow(np.transpose(self._height_map), origin="lower")
		plt.colorbar()
		plt.scatter(self._start_grid_north, self._start_grid_east)
		plt.scatter(self._goal_grid_north, self._goal_grid_east)
		plt.show()

		self._build_voxmap_around_current(self._start_grid)

	def _build_voxmap_around_current(self, current_pos: Tuple[int]):
		start_time = time.time()
		current_pos = (200, 200, 20)
		# Define the shape of the grid
		#voxmap_shape = (40, 40, 20)
		voxmap_shape = (900, 900, 200)
		north_start_index = max(int(current_pos[0] - voxmap_shape[0] / 2), 0)
		north_end_index = int(current_pos[0] + voxmap_shape[0] / 2)

		east_start_index = max(int(current_pos[1] - voxmap_shape[1] / 2), 0)
		east_end_index = int(current_pos[1] + voxmap_shape[1] / 2)

		down_start_index = max(int(current_pos[2] - voxmap_shape[2] / 2), 0)
		down_end_index = int(current_pos[2] + voxmap_shape[2] / 2)
		
		# Define the range of indices for each dimension
		north_indices = np.arange(north_start_index, north_end_index)
		east_indices = np.arange(east_start_index, east_end_index)
		down_indices = np.arange(down_start_index, down_end_index)

		# Create a 3D grid of indices
		north_grid, east_grid, down_grid = np.meshgrid(north_indices, east_indices, down_indices, indexing='ij')

		# Compute the voxmap using vectorized operations
		voxmap = self._height_map[north_grid, east_grid] > down_grid

		end_time = time.time()

		# Determine the grid cell location of the subgoal position
		current_voxel_position = (int(voxmap_shape[0] / 2), int(voxmap_shape[1] / 2), int(voxmap_shape[2] / 2))
		
		#start_state.position_in_3d		
		#next_waypoint = _____



		# Check whether or not that location is occupied

		# If it's occupied, find the nearest neighbor free cell

		# Search the voxmap from start position to the subgoal position

		# Determine the sequence of indices from start to subgoal

		# Filter out the indices that don't change direction

		


		print(end_time - start_time)
		# Plot the voxmap as a 3D scatter plot
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')

		# Get the indices of the occupied cells
		occupied_indices = np.nonzero(voxmap)

		# Plot the occupied cells
		ax.scatter(
			occupied_indices[0],
			occupied_indices[1],
			occupied_indices[2],
			c='red',
			marker='o',
			label='Occupied',
			alpha=0.1
		)

		# Get the indices of the free cells
		free_indices = np.where(voxmap == False)

		# Plot the free cells
		ax.scatter(
			free_indices[0],
			free_indices[1],
			free_indices[2],
			c='green',
			marker='o',
			label='Free',
			alpha=0.0
		)

		ax.scatter(
			voxmap_shape[0]/2,
			voxmap_shape[1]/2,
			voxmap_shape[2]/2,
			c="black",
			marker="o",
			label="Start"
			)
		# Set axis labels
		ax.set_xlabel('X')
		ax.set_ylabel('Y')
		ax.set_zlabel('Z')

		# Add a legend
		ax.legend()

		# Show the plot
		plt.show()

		values = [0, 1, -1]
		permuted_tuples = list(itertools.product(values, repeat=3))
		# Filter out the tuple (0, 0, 0)
		permuted_tuples = [t for t in permuted_tuples if t != (0, 0, 0)]
		print(len(permuted_tuples))

class Grid(Planner):
	pass
	"""
	def __init__(self, environment: Environment, start_state: State, goal_state: State):
		super().__init__(environment, start_state, goal_state)
		self._grid, self._north_offset, self._east_offset = self._convert_obstacles_to_grid()
		self._start_grid_north = int(self._start_state.local_position.north - self._north_offset)
		self._start_grid_east = int(self._start_state.local_position.east - self._east_offset)
		self._goal_grid_north = int(self._goal_state.local_position.north - self._north_offset)
		self._goal_grid_east = int(self._goal_state.local_position.east - self._east_offset)
		self._start_grid = (self._start_grid_north, self._start_grid_east)
		self._goal_grid = (self._goal_grid_north, self._goal_grid_east)
		self._build_obstacle_potential_field()

	def _build_obstacle_potential_field(self):
		altitude = self._goal_state.local_position.down
		self._north_size = int(self._environment.north_bounds.maximum - self._environment.north_bounds.minimum)
		self._east_size = int(self._environment.east_bounds.maximum - self._environment.east_bounds.minimum)
		north_offset = self._environment.obstacles.north_offset
		east_offset = self._environment.obstacles.east_offset
		grid = np.zeros((self._north_size, self._east_size))
		for obstacle in self._environment.obstacles.list:
			north_start = int(obstacle.north_min - north_offset)
			north_end = int(obstacle.north_max - north_offset)
			east_start = int(obstacle.east_min - east_offset)
			east_end = int(obstacle.east_max - east_offset)

			grid[north_start: north_end, east_start:east_end] = 1.0 # occupied

		obstacle_distance_transform = 100 / scipy.ndimage.distance_transform_edt(1-grid)


		plt.imshow(obstacle_distance_transform, origin="lower", cmap="jet")
		plt.colorbar()
		plt.show()
		grid2 = np.zeros((self._north_size, self._east_size))
		grid2[self._goal_grid_north,self._goal_grid_east] = 1
		distance_transform_goal = scipy.ndimage.distance_transform_edt(1-grid2)

		plt.imshow(distance_transform_goal, origin="lower", cmap="jet")
		plt.colorbar()
		plt.show()

		# Compute the potential field.
		potential = obstacle_distance_transform + distance_transform_goal

		# Compute the gradient of the potential field.
		gradient_y, gradient_x = np.gradient(-potential)  # We negate the potential field here because we want to descend the potential field, not ascend it

		# Compute the magnitude of the gradient.
		gradient_magnitude = np.sqrt(gradient_y**2 + gradient_x**2)

		# Normalize the gradient.
		gradient_y /= gradient_magnitude
		gradient_x /= gradient_magnitude

		# Create a grid of coordinates.
		x, y = np.meshgrid(np.arange(self._east_size), np.arange(self._north_size))

		# Every nth arrow for cleaner plotting
		nth = 10
		fig, ax = plt.subplots(figsize=(10, 10))

		# Plot the potential field.
		ax.imshow(potential, origin="lower", cmap="jet")

		# Plot the vector field.
		ax.quiver(x[::nth, ::nth], y[::nth, ::nth], gradient_x[::nth, ::nth], gradient_y[::nth, ::nth], scale=50)
		plt.scatter(self._goal_grid_east, self._goal_grid_north)

		# Show the plot.
		plt.show()

	def generate_trajectory(self):
		step = 0.01
		north_current = 0
		east_current = 0 
		gradient_north = 0
		gradient_east = 0
		while True:
			north_current = north_current + gradient_north * time_step


		
	def _convert_obstacles_to_grid(self):
		altitude = self._goal_state.local_position.down
		self._north_size = int(self._environment.north_bounds.maximum - self._environment.north_bounds.minimum)
		self._east_size = int(self._environment.east_bounds.maximum - self._environment.east_bounds.minimum)
		north_offset = self._environment.obstacles.north_offset
		east_offset = self._environment.obstacles.east_offset
		grid = np.zeros((self._north_size, self._east_size))
		for obstacle in self._environment.obstacles.list:
			north_start = int(obstacle.north_min - north_offset)
			north_end = int(obstacle.north_max - north_offset)
			east_start = int(obstacle.east_min - east_offset)
			east_end = int(obstacle.east_max - east_offset)

			grid[north_start: north_end, east_start:east_end] = 1.0 # occupied
		
		distance_transform = scipy.ndimage.distance_transform_edt(1-grid)

		return distance_transform, north_offset, east_offset

	def _local_to_grid(self, state: State):
		north_grid = int(state.local_position.north - self._north_offset)
		east_grid = int(state.local_position.east - self._east_offset)
		return north_grid, east_grid


	def search(self):
		start_pos = self._start_grid
		goal_pos = self._goal_grid
		visited = set()
		goal_is_found = False
		branch = {}
		queue = PriorityQueue()

		queue.put((0.0, start_pos))
		while not queue.empty():

			current_pos = queue.get()[1]
			print(current_pos)

			if current_pos == goal_pos:
				goal_is_found = True
				print("The goal has been found.")
				break

			for neighbor_and_cost in self._get_neighbors(current_pos):
				neighbor = neighbor_and_cost[0]
				if neighbor not in visited:
					g_score = neighbor_and_cost[1]
					h_score = np.linalg.norm(np.array([neighbor[0], neighbor[1]])- np.array(goal_pos))
					f_score = g_score + h_score
					queue.put((f_score, neighbor))
					branch[neighbor] = (current_pos, g_score)
					visited.add(neighbor)
		if goal_is_found:
			cost = 0.0
			current_pos = goal_pos
			path = [current_pos]
			while current_pos != start_pos:
				cost += branch[current_pos][1]
				path.append(branch[current_pos][0])
				current_pos = branch[current_pos][0]
			path.append(start_pos)
			path.reverse()
			self._path = path
			print(("The cost is", cost))




	def _get_neighbors(self, current_pos):
		valid_positions_and_costs = []
		for action in Action:
			new_pos = (current_pos[0] + action.delta[0], current_pos[1] + action.delta[1])
			if new_pos[0] < 0 or new_pos[0] >= self._north_size or new_pos[1] < 0 or new_pos[1] >= self._east_size:
				continue 
			if self._grid[new_pos[0], new_pos[1]] > 5.0:
				valid_positions_and_costs.append((new_pos, action.cost))
		return valid_positions_and_costs

	def visualize(self):
		
		plt.imshow(np.transpose(self._grid), origin="lower", cmap="jet")
		plt.colorbar()
		plt.scatter(self._start_grid[0], self._start_grid[1], label="Start", color="lime")
		plt.scatter(self._goal_grid[0], self._goal_grid[1], label="Goal",color="lime")
		plt.plot([pos[0] for pos in self._path], [pos[1] for pos in self._path], color="orange", label="Path")
		plt.title('Distance transform of the environment ')

		plt.legend()
		plt.show()

class Action(Enum):
	NORTH = (1, 0, 1)
	EAST = (0, 1, 1)
	SOUTH = (-1, 0, 1)
	WEST = (0, -1, 1)
	NORTHEAST = (1, 1, np.sqrt(2))
	NORTHWEST = (1, -1, np.sqrt(2))
	SOUTHEAST = (-1, 1, np.sqrt(2))
	SOUTHWEST = (-1, -1, np.sqrt(2))

	@property
	def delta(self):
		return self.value[0:2]
	@property
	def cost(self):
		return self.value[2]
	
"""