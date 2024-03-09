from geodetic_position import GeodeticPosition
from halfsize import HalfSize 
from local_position import LocalPosition
import numpy as np
from obstacle import Obstacle
from sklearn.neighbors import KDTree
from typing import List

class ObstacleCollection:
	def __init__(self, obstacle_array: np.ndarray, geodetic_home: GeodeticPosition):
		self._obstacle_array = obstacle_array
		self._list = self._build_list(obstacle_array)
		self._tree = self._build_tree()
		self._safety = self._determine_safety()
		self._north_offset = self._determine_north_offset()
		self._east_offset = self._determine_east_offset()
		self._height_map = self._build_height_map()
		self._detected_obstacle_list = []

	def _build_height_map(self) -> np.ndarray:
		north_max = np.ceil(np.max(self._obstacle_array[:, 0] + self._obstacle_array[:, 3]))
		east_max = np.ceil(np.max(self._obstacle_array[:, 1] + self._obstacle_array[:, 4]))
		north_min = np.floor(np.min(self._obstacle_array[:, 0] - self._obstacle_array[:, 3]))
		east_min = np.floor(np.min(self._obstacle_array[:, 1] - self._obstacle_array[:, 4]))
		north_size = int(north_max - north_min)
		east_size = int(east_max - east_min)
		height_map = np.zeros((north_size, east_size))
		for obstacle in self._list:
			north_start = int(obstacle.north_min - self._north_offset)
			north_end = int(obstacle.north_max - self._north_offset)
			east_start = int(obstacle.east_min - self._east_offset)
			east_end = int(obstacle.east_max - self._east_offset)
			height_map[north_start: north_end, east_start: east_end] = obstacle.height
		return height_map
		

	def _determine_east_offset(self) -> float:
		"""Determine the minimum obstacle center for east"""
		east_minimum = np.inf
		for obstacle in self._list:
			if obstacle.east_min < east_minimum:
				east_minimum = obstacle.east_min
		return east_minimum

	def _determine_north_offset(self) -> float:
		"""Determine the minimum obstacle center for north"""
		north_minimum = np.inf
		for obstacle in self._list:
			if obstacle.north_min < north_minimum:
				north_minimum = obstacle.north_min
		return north_minimum

	def _determine_safety(self) -> float:
		"""Determine the largest hypotenuse of obstacles in the list of obstacles"""
		safety = 0
		for obstacle in self._list:
			hypotenuse = np.hypot(obstacle.halfsize.north, obstacle.halfsize.east)
			if hypotenuse > safety:
				safety = hypotenuse

		# For added safety, return a safety distance equal to twice or three times the longest obstacle
		# hypotenuse, rather than return a safety distance that is merely the longest
		# obstacle hypotenuse. 
		
		return 3*safety

	def _build_list(self, obstacle_array: np.ndarray) -> List[Obstacle]:
		"""Converts the obstacle information from an array into a list of Obstacle objects"""
		obstacle_list = []
		for row_index in range(obstacle_array.shape[0]):
			north, east, down, north_halfsize, east_halfsize, down_halfsize = obstacle_array[row_index, :]
			local_position = LocalPosition(north, east, down)
			halfsize = HalfSize(north_halfsize, east_halfsize, down_halfsize)
			obstacle_list.append(Obstacle(local_position, halfsize))

		return obstacle_list
	
	def _build_tree(self) -> KDTree:
		"""Returns a KDTree of obstacle ground-center positions"""
		ground_center_coords = []
		for obstacle in self._list:
			ground_center_coord = np.array([obstacle.local_position.north, obstacle.local_position.east])
			ground_center_coords.append(ground_center_coord)
		ground_center_coords = np.array(ground_center_coords)
		
		tree = KDTree(ground_center_coords)

		return tree

	def add_detected_obstacle(self, new_obstacle: Obstacle) -> None:
		self._detected_obstacle_list.append(new_obstacle)

	def insert_obstacle_into_collection(self, new_obstacle: Obstacle) -> None:
		"""Inserts an obstacle into the collection of obstacles"""
		self._list.append(new_obstacle)
		self._tree = self._build_tree()

		# Update heightmap
		north_start = int(new_obstacle.north_min - self._north_offset)
		north_end = int(new_obstacle.north_max - self._north_offset)
		east_start = int(new_obstacle.east_min - self._east_offset)
		east_end = int(new_obstacle.east_max - self._east_offset)
		self.height_map[north_start: north_end, east_start: east_end] = new_obstacle.height

	@property
	def detected_obstacle_list(self):
		return self._detected_obstacle_list

	@property
	def list(self):
		return self._list

	@property
	def tree(self):
		return self._tree

	@property
	def safety(self):
		return self._safety

	@property
	def north_offset(self):
		return self._north_offset

	@property
	def east_offset(self):
		return self._east_offset

	@property
	def height_map(self):
		return self._height_map
