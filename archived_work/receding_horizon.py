import environment as Environment
import numpy as np


class PotentialField2:
	def __init__(self, environment: Environment, current_waypoint: np.ndarray, next_waypoint: np.ndarray):
		# Given the environment and the waypoint or local position, this should be able to compute the direction to travel in. The receding horizon planner should then plan 40m ahead of that location.  
		self._environment = environment
		self._current_waypoint = current_waypoint
		self._next_waypoint = next_waypoint
		self._force_from_current_to_next = self._compute_force_current_to_next()
		self._force_from_cur

		pass



# Determine the position of the nearest obstacle
next_waypoint_ground_position = np.array([self.next_waypoint[0], self.next_waypoint[1]])
ground_position = np.ndarray([local_position[0], local_position[1]])




nearest_obstacle_index = self._environment.obstacles.tree.query([current_position])[1][0][0]
nearest_obstacle = self._environment.obstacles.list[nearest_obstacle_index]
obstacle_position = np.array([nearest_obstacle.local_position.north, nearest_obstacle.local_position.east])

ko = 1
if nearest_obstacle.height < local_position[2]:
	ko = 0
force_obstacle_on_current = ko * (ground_position - obstacle_position) / np.linalg.norm(ground_position - obstacle_position)

kw = 4
force_next_waypoint_on_current = kw * (next_waypoint_ground_position - ground_position) / np.linalg.norm(next_waypoint_ground_position - ground_position)

force_resultant = force_obstacle_on_current + force_next_waypoint_on_current


# Get all obstacles from self._environment.obstacles.tree that are within a 40m ground position radius by querying the KD tree and using those return indices 
# to look up the actual objects in the list. Once I have the list of obstacles that apply, I take the closest one, if there is one, and compute the vector from
# it to the current position in 3d. I will pretend that it's height is the same as the height of the current position unless it's a special object, by which case 
# its height will be set with a special parameter to aid in obstacle avoidance. 