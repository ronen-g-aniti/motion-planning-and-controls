from bounds import Bounds
from geodetic_position import GeodeticPosition
from local_position import LocalPosition
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.patches import Circle
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from obstacle import Obstacle
from obstacle_collection import ObstacleCollection
from state import State
import utm

class Environment:
	def __init__(self, geodetic_home: GeodeticPosition, obstacle_collection: ObstacleCollection):
		self._geodetic_home = geodetic_home
		self._obstacles = obstacle_collection
		self._north_bounds = self._determine_north_bounds() # Build and return Bounds Object from obstacle_collection
		self._east_bounds = self._determine_east_bounds()
		self._down_bounds = self._determine_down_bounds()


	def _state_is_out_of_bounds(self, state: State) -> bool:
		if state.local_position.north < self._north_bounds.minimum:
			return True
		if state.local_position.north > self._north_bounds.maximum:
			return True
		if state.local_position.east < self._east_bounds.minimum:
			return True
		if state.local_position.east > self._east_bounds.maximum:
			return True

	def state_collides_with_obstacle(self, state: State) -> bool:
		state_is_out_of_bounds = self._state_is_out_of_bounds(state)
		if state_is_out_of_bounds:
			return True
		else:
			indices_of_nearby_obstacles = self._obstacles.tree.query_radius([state.ground_position], self.obstacles.safety)[0]
			obstacles_are_nearby = len(indices_of_nearby_obstacles) > 0
			collision_is_detected = False # At first, assume no collision
			if obstacles_are_nearby:
				for index in indices_of_nearby_obstacles:
					if self._obstacles.list[index].height >= state.local_position.down:
						return True
			return False

	def _determine_north_bounds(self) -> Bounds:
		north_minimum_environment = np.inf
		for obstacle in self._obstacles.list:
			north_minimum_obstacle = obstacle.local_position.north - obstacle.halfsize.north
			if north_minimum_obstacle < north_minimum_environment:
				north_minimum_environment = north_minimum_obstacle

		north_maximum_environment = -np.inf
		for obstacle in self._obstacles.list:
			north_maximum_obstacle = obstacle.local_position.north + obstacle.halfsize.north
			if north_maximum_obstacle > north_maximum_environment:
				north_maximum_environment = north_maximum_obstacle

		north_bounds = Bounds(north_minimum_environment, north_maximum_environment)

		return north_bounds

	def _determine_east_bounds(self) -> Bounds:
		east_minimum_environment = np.inf
		for obstacle in self._obstacles.list:
			east_minimum_obstacle = obstacle.local_position.east - obstacle.halfsize.east
			if east_minimum_obstacle < east_minimum_environment:
				east_minimum_environment = east_minimum_obstacle

		east_maximum_environment = -np.inf
		for obstacle in self._obstacles.list:
			east_maximum_obstacle = obstacle.local_position.east + obstacle.halfsize.east
			if east_maximum_obstacle > east_maximum_environment:
				east_maximum_environment = east_maximum_obstacle

		east_bounds = Bounds(east_minimum_environment, east_maximum_environment)

		return east_bounds

	def _determine_down_bounds(self) -> Bounds:
		down_minimum_environment = np.inf
		for obstacle in self._obstacles.list:
			down_minimum_obstacle = obstacle.local_position.down - obstacle.halfsize.down
			if down_minimum_obstacle < down_minimum_environment:
				down_minimum_environment = down_minimum_obstacle

		down_maximum_environment = -np.inf
		for obstacle in self._obstacles.list:
			down_maximum_obstacle = obstacle.local_position.down + obstacle.halfsize.down
			if down_maximum_obstacle > down_maximum_environment:
				down_maximum_environment = down_maximum_obstacle

		down_bounds = Bounds(down_minimum_environment, down_maximum_environment)

		return down_bounds

	def _update_bounds(self) -> None:
		#Should find the bounds based on the obstacle tree or list
		pass

	def visualize(self) -> None:
		# Generate a graph of the environment, labeled with axes, obstacle heights, and a title. 
		fig, ax = plt.subplots()
		ax.set_xlabel("North")
		ax.set_ylabel("East")
		color_map_of_plot = plt.get_cmap('gist_yarg')
		list_of_obstacle_heights = [obstacle.height for obstacle in self.obstacles.list]
		normalization_of_obstacle_heights = Normalize(vmin=min(list_of_obstacle_heights), vmax=max(list_of_obstacle_heights))
		scalar_mappable = ScalarMappable(norm=normalization_of_obstacle_heights, cmap=color_map_of_plot)
		scalar_mappable.set_array([])
		for obstacle in self.obstacles.list:
			obstacle_color = color_map_of_plot(normalization_of_obstacle_heights(obstacle.height))
			obstacle_as_a_circle_patch = patches.Circle((obstacle.local_position.north, obstacle.local_position.east), obstacle.safety, color=obstacle_color, alpha=0.3)
			ax.add_patch(obstacle_as_a_circle_patch)
			#ax.text(obstacle.local_position.north, obstacle.local_position.east, f"{float(obstacle.height):.1f}", fontsize=8)
		#ax.scatter(self.goal_state.local_position.north, self.goal_state.local_position.east, c='red', s=30, marker='x', label='Goal')
		#ax.legend()
		if len(self.obstacles.detected_obstacle_list) > 0:
			for obstacle in self.obstacles.detected_obstacle_list:
				obstacle_as_a_circle_patch = patches.Circle((obstacle.local_position.north, obstacle.local_position.east), obstacle.safety, color="red")
				ax.add_patch(obstacle_as_a_circle_patch)

		color_bar = plt.colorbar(scalar_mappable, ax=ax)
		color_bar.set_label("Altitude (m)")
		ax.set_xlim(self.north_bounds.minimum, self.north_bounds.maximum)
		ax.set_ylim(self.east_bounds.minimum, self.east_bounds.maximum)
		plt.title("The Environment")
		return fig, ax

	@property
	def north_bounds(self):
		return self._north_bounds

	@property
	def east_bounds(self):
		return self._east_bounds

	@property
	def down_bounds(self):
		return self._down_bounds

	@property
	def obstacles(self):
		return self._obstacles

	@property
	def geodetic_home(self):
		return self._geodetic_home
