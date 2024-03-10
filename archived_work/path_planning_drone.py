"""
. Start the drone connection. 
. Establish the current geodetic location as a geodetic object. 
. Establish a goal geodetic location
. Use one of the global planning classes to return a sequence of waypoints for the drone to follow to 
	travel from the start geodetic location to the goal geodetic location 
. Once the global path is determined, the drone will use a receding horizon planner for navigation 
	between global waypoints. 
. . The receding horizon planner will work by building a 3d occupancy grid around the current location
	and searching from the current location inside of that grid to the closest available node inside
	that grid to the next global waypoint. The receding horizon planner will then return the next local
	waypoint to the drone as a position command. The drone should gradually make progress towards the first
	global waypoint. When it's within the deadband radius of the next coarse plan waypoint, the 
	program will establish the next global waypoint as the next global goal.   


The receding horizon planner probably must perform its computations quickly so that it doesn't 
interfere with the simulation loop. 

The receding horizon planner should be nested inside the position update callback function.
It will command the drone to fly to the next waypoint on the horizon. 
"""
import msgpack
import numpy as np

import time

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

from enum import Enum, auto
import pdb

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Dict, Tuple

class States(Enum):
	MANUAL = auto()
	ARMING = auto()
	WAYPOINT = auto()
	LANDING = auto()
	DISARMING = auto()
	FOLLOW = auto()


class PathPlanning(Drone):
	def __init__(self, connection: MavlinkConnection, obstacles: np.ndarray, geodetic_home: np.ndarray, leader_data: np.ndarray, filtered_points: np.ndarray):
		#self, connection: MavlinkConnection, obstacles: np.ndarray, waypoints: np.ndarray, geodetic_home: np.ndarray, leader_data: np.ndarray
		super().__init__(connection)
		"""
		self.orig_waypoints = waypoints

		self.target_position = np.array([0.0, 0.0, 0.0, 0.0])
		self.waypoints = list(waypoints)
		self.goal_altitude = waypoints[-1][2]
		self.obstacles = obstacles
		self.geodetic_home = geodetic_home

		self.in_mission = True
		self.check_state = {}
	
		#self.states = None
		#self.connections = None
		
		self.flight_state = States.MANUAL

		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.STATE, self.state_callback)
		
		# For evaluating test data
		self.start_time = time.time()
		self.timer = []
		self.pos = []
		"""

		self.obstacles = obstacles
		self.geodetic_home = geodetic_home
		self.leader_data = leader_data
		self.in_mission = True
		self.check_state = {}
		self.flight_state = States.MANUAL
		self.leader_follow_point = self.leader_data[0][5:8] # Initial tracking point for path planning drone
		self.filtered_points = filtered_points
		print("LFP", self.leader_follow_point)
		# For evaluating test data
		self.start_time = time.time()
		self.timer = []
		self.pos = []
		self.kd_tree_obstacles = KDTree(obstacles[:,:2])
		# A Star to find path from start to initial follow point. 
		self.filtered_points_kd_tree = KDTree(self.filtered_points)
		self.path = list(a_star(np.array([0,0,0]), self.leader_follow_point, self.filtered_points, self.filtered_points_kd_tree))
		print(self.path)
		self.target_waypoint = self.path[1]
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.STATE, self.state_callback)

	def local_position_callback(self):

		# For evaluating test data:
		self.timer.append(time.time() - self.start_time)
		print(time.time() - self.start_time)
		self.pos.append([self.local_position[0], self.local_position[1], -1.0 * self.local_position[2]])
		
		# Check how far away drone is from the nearest coarse state to the follow position
		dx = abs(np.interp(self.timer[-1], self.leader_data[:, 0], self.leader_data[:, 1]) - self.pos[-1][0])
		dy = abs(np.interp(self.timer[-1], self.leader_data[:, 0], self.leader_data[:, 2]) - self.pos[-1][1])
		dz = abs(np.interp(self.timer[-1], self.leader_data[:, 0], self.leader_data[:, 3]) - self.pos[-1][2])
		dist_to_follow_pos = np.sqrt(dx**2 + dy**2 + dz**2)
		print("Dist to follow pos", dist_to_follow_pos)

		# If it's within 100 m of the true follow position, activate potential field attraction to the follow position
		if dist_to_follow_pos <= 100.0:
			a=1
		
		# IF it's further than 100m, consider whether its position has changed since the last position callback
		else:
			# Lookup the mesh point closest to the follow location
			lookup_idx = np.argmax(self.leader_data[: , 0] > self.timer[-1])
			print("Vector to follow point",self.leader_data[lookup_idx, 5:8] - self.leader_follow_point)
			if np.linalg.norm(self.leader_data[lookup_idx, 5:8] - self.leader_follow_point) < 1.0:
				# Nearest mesh point Position has not changed (beyond a small tolerance)
				self.target_waypoint = np.array(self.path.pop(0))
				print("Target Waypoint",self.target_waypoint)
				vector_to_leader = self.target_waypoint[:3] - np.array(self.pos[-1])
				direction_to_leader = vector_to_leader / np.linalg.norm(vector_to_leader)
				attractive_gain = 50.0 
				nearest_obs = determine_nearest_obs(np.array(self.pos[-1]), self.obstacles, self.kd_tree_obstacles)
				vector_from_nearest_obs = np.array(self.pos[-1]) - nearest_obs[:3] 
				dist_to_obs = np.linalg.norm(vector_from_nearest_obs)
				detection_dist = 50
				collision_dist = 10
				repulsive_gain = 50.0 * (1 - ((dist_to_obs - collision_dist) / (detection_dist - collision_dist)))
				direction_from_obs = vector_from_nearest_obs / np.linalg.norm(vector_from_nearest_obs)
				command_vector = attractive_gain * direction_to_leader + repulsive_gain * direction_from_obs
				command_direction = command_vector / np.linalg.norm(command_vector)
			else:
				# Nearest mesh point Position has changed
				self.path = a_star(np.array(self.pos[-1]), self.leader_follow_point, self.filtered_points, self.filtered_points_kd_tree)
				self.path = list(self.path)
				self.target_waypoint = self.path[0]
				vector_to_leader = self.target_waypoint[:3] - np.array(self.pos[-1])
				direction_to_leader = vector_to_leader / np.linalg.norm(vector_to_leader)
				attractive_gain = 50.0
				nearest_obs = determine_nearest_obs(np.array(self.pos[-1]), self.obstacles, self.kd_tree_obstacles)
				vector_from_nearest_obs = np.array(self.pos[-1]) - nearest_obs[:3]
				dist_to_obs = np.linalg.norm(vector_from_nearest_obs)
				detection_dist = 50
				collision_dist = 10
				repulsive_gain = 50.0 * (1 - ((dist_to_obs - collision_dist) / (detection_dist - collision_dist)))
				direction_from_obs = vector_from_nearest_obs / np.linalg.norm(vector_from_nearest_obs)
				command_vector = attractive_gain * direction_to_leader + repulsive_gain * direction_from_obs
				command_direction = command_vector / np.linalg.norm(command_vector)
			command_position = np.array(self.pos[-1]) + command_direction*20
			
			print("Command: ",command_direction)
			print("Command POS: ", command_position)
			self.cmd_position(*command_position, 0)


			# IF its position has changed, then use a* from the current position to the follow position, and return the first waypoint

				# Compute the attractive vector to the target

				# Compute the repulsive vector from the nearest obstacle

				# Compute the weighted sum of the attractive vector and repulsive vector

				

			# ELIF its position hasn't changed

				# Compute the attractive vector to the target 

				# Compute the repulsive vector to the target

				# Compute the weighted sum of the attractive vector and repulsive vector


			# Command a new position that's equal to the sum of the current position plus the weighted sum

		# If it's within 20m of the true follower position, then:

			# Compute the attractive vector to the true follow position

			# Compute the repulsive vector from the nearest obstacle

			# Compute the weighted sum of the attractive vector and repulsive vector

			# Command a new position that's equal 


		deadband_radius = 5.0
		if np.linalg.norm(np.append(self.local_position[0:2], -1.0*self.local_position[2]) - self.target_waypoint[0:3]) < deadband_radius:
				self.waypoint_transition()
		"""
		# Have the receding horizon planner command an available position on the edge of 
		# the horizon that's in the direction of the next waypoint.
		deadband_radius = 5 # meters
		if self.flight_state == States.WAYPOINT:
			self.local_planner()
			if np.linalg.norm(np.append(self.local_position[0:2], -1.0*self.local_position[2]) - self.next_waypoint[0:3]) < deadband_radius:
				self.waypoint_transition()
		"""
	def velocity_callback(self):
		pass

	def state_callback(self):
		print("State callback")
		if self.in_mission:
			if self.flight_state == States.MANUAL:
				self.arming_transition()
			elif self.flight_state == States.ARMING:
				if self.armed:
					self.set_home_position(*self.geodetic_home)
					self.takeoff_transition()

	def local_planner(self):
		# Avoid obstacles using a potential field algorithm
		# The next waypoint will have the attractive potential
		# The previous waypoint will have a repulsive potential
		# Obstacles will have a repulsive potential 
		# The activation distance for obstacle avoidance shall be a parameter OBSTACLE_SAFETY
		
		pass

	def tracking_planner(self, goal_pos):
		# Update the global plan to match a given local position
		# Use RRT algorithm with a bias towards the first waypoint of the previous plan and the goal position
		# To test:
		# Repeatedly plan to the same location on the map [800, 800, 20, 0], using the current location [cx, cy, cz, cpsi] as the starting state.
		# Check to see if the drone can actually reach the destination without problems. 

		current_pos = np.array([self.local_position[0], self.local_position[1], self.local_position[2] * -1.0, 0])
		shortened_path, self.states, self.connections = rrt(current_pos, goal_pos, self.states, self.connections)
		command_pos = shortened_path[0]
		self.cmd_position(command_pos)
		
	def manual_transition(self):
		pass

	def arming_transition(self):
		self.flight_state = States.ARMING
		print("Arming transition")
		self.arm()
		time.sleep(2)
		self.take_control()


	def waypoint_transition(self):
		print("Popping waypoint")
		self.target_waypoint = self.path.pop(0) 
		"""
		self.flight_state = States.WAYPOINT
		if len(self.waypoints) > 0:
			self.next_waypoint = self.waypoints.pop(0)
			self.cmd_position(*self.next_waypoint)
		else:
			print("Done")
			time_history = np.array(self.timer)
			position_history = np.array(self.pos)
			obstacles = self.obstacles
			orig_waypoints = self.orig_waypoints
			np.savez('arrays.npz', time_history=time_history, position_history=position_history, obstacles=obstacles, orig_waypoints=orig_waypoints)
			print("Saved")
			self.in_mission = False
			self.stop()
			self.release_control()
		"""



	def takeoff_transition(self):
		#self.flight_state = States.TAKEOFF
		#print("Transitioning to takeoff state")
		#self.takeoff(self.goal_altitude)
		pass
	def landing_transition(self):
		pass

	def disarming_transition(self):
		pass

	def send_waypoints(self):
		print("Sending waypoint to simulator...")
		data = msgpack.dumps(self.waypoints)
		self.connection._master.write(data)

	def start(self):
		self.start_log("Logs", "NavLog.txt")
		print("Starting connection")
		self.connection.start()
		


# Get geodetic home from csv file

# Establish tracking of a given target which is type leader

class Leader:
	"""
	This leader drone will advance its position forward according to its speed and its direction vector 
	to its next waypoint. Its waypoints will be determined by an rrt search over the obstacle space.
	"""
	def __init__(self, start_pos: np.ndarray, goal_pos: np.ndarray, speed: float, coarse_space: np.ndarray, obstacles: np.ndarray, kd_tree_obs, coarse_space_kd_tree):
		self.start_pos = start_pos
		self.goal_pos = goal_pos
		self.coarse_space = coarse_space
		self.speed = speed
		self.obstacles = obstacles
		self.kd_tree_obs = kd_tree_obs
		self.kd_tree_coarse_space = coarse_space_kd_tree
		self.waypoints = list(a_star(start_pos[:3], goal_pos[:3], coarse_space, KDTree(coarse_space)))
		self.start_time = time.time()
		#self.timer = [time.time() - self.start_time] # Seconds
		self.timer = [0.0]
		self.current_pos = [start_pos]
		self.follow_pos = [start_pos]

		# Follow point is the list of coarse points closest to the follow position
		nearest_coarse_point_idx = self.kd_tree_coarse_space.query([[self.follow_pos[-1][:3]]], k=1)[1][0][0]
		pdb.set_trace()
		self.follow_point = [self.coarse_space[nearest_coarse_point_idx]]

		self.next_pos = self.waypoints.pop(0)
		self.deadband = 5.0
		self.safety_dist = 5.0
		self.detection_dist = 20.0
		self.repulsive_gain_max = 50.0
		self.attractive_gain = 50.0
		self.follow_dist = 7.0

	def advance_pos(self):
		# If current position is within the deadband radius of the next waypoint, set the waypoint after as `next_waypoint`
		if np.linalg.norm(self.next_pos[:3] - self.current_pos[-1][:3]) < self.deadband:
			if len(self.waypoints) == 0:
				return None
			self.next_pos = self.waypoints.pop(0)

		# Determine the direction vector to the next waypoint from the current location
		direction_to_next_waypoint = (self.next_pos[:3] - self.current_pos[-1][:3]) / np.linalg.norm(self.next_pos[:3] - self.current_pos[-1][:3])
		
		# Determine the nearest obstacle to the current location that's higher than the current altitude of the drone
		nearest_obs = determine_nearest_obs(self.current_pos[-1], self.obstacles, self.kd_tree_obs)
		
		if nearest_obs is not None:
			vector_obs_leader = np.array(self.current_pos[-1][:3]) - np.append(nearest_obs[:2], self.current_pos[-1][2])
			dist_from_obs = np.linalg.norm(vector_obs_leader)
			direction_from_obs = vector_obs_leader / dist_from_obs
			repulsive_gain = 0.0
			if dist_from_obs < self.detection_dist:
				print("Applying potential field")
				print("Percent strength", (1- ((dist_from_obs - self.safety_dist) / (self.detection_dist - self.safety_dist)) ))
				print((nearest_obs[2]+nearest_obs[5], self.current_pos[-1][2]))
				repulsive_gain = self.repulsive_gain_max * (1- ((dist_from_obs - self.safety_dist) / (self.detection_dist - self.safety_dist)) )
			repulsive_vector = repulsive_gain * direction_from_obs
		else:
			repulsive_vector = np.zeros((1, 3))

		# Determine attraction vector
		vector_leader_target = self.next_pos[:3] - self.current_pos[-1][:3]
		dist_to_target = np.linalg.norm(vector_leader_target)
		direction_to_target = vector_leader_target / dist_to_target
		attractive_vector = self.attractive_gain * direction_to_target

		# Resultant vector and direction
		resultant_vector = repulsive_vector + attractive_vector
		resultant_direction = resultant_vector / np.linalg.norm(resultant_vector)

		# Update `self.current_pos`
		elapsed_time = time.time() - self.start_time - self.timer[-1]
		#self.timer.append(time.time() - self.start_time)
		self.timer.append(self.timer[-1] + 1/24) # 12 m/s
		self.current_pos.append(np.append(self.current_pos[-1][:3] + 1 * resultant_direction, 0.0)) #Debug: self.speed * elapsed_time * resultant_direction

		# Update `self.follow_pos`
		self.follow_pos.append(np.append(self.current_pos[-1][:3] - resultant_direction * self.follow_dist, 0.0))
		nearest_coarse_point_idx = self.kd_tree_coarse_space.query([[self.follow_pos[-1][:3]]], k=1)[1][0][0]
		self.follow_point.append(self.coarse_space[nearest_coarse_point_idx])

def determine_nearest_obs(current_pos: np.ndarray, obstacles: np.ndarray, kd_tree_obs):
	print(current_pos)
	_, neighbors = kd_tree_obs.query([current_pos[:2]], k=10)
	neighbors = neighbors[0]
	for neighbor_idx in neighbors:
		if obstacles[neighbor_idx][2] + obstacles[neighbor_idx][5] > current_pos[2]:
			return obstacles[neighbor_idx]
	return None


def read_obstacle_data(filename: str) -> np.ndarray:
	obstacles = np.genfromtxt(filename, delimiter=',', skip_header=2)
	return obstacles
def determine_offsets(obstacles: np.ndarray) -> np.ndarray:
	x_center_min = obstacles[np.argmin(obstacles[:, 0])][0]
	y_center_min = obstacles[np.argmin(obstacles[:, 1])][1]
	z_center_min = obstacles[np.argmin(obstacles[:, 2])][2]
	center_min = np.array([x_center_min, y_center_min, z_center_min])
	return center_min
def determine_safety(obstacles: np.ndarray) -> float:
	dx_max = obstacles[np.argmax(obstacles[:, 3])][3]
	dy_max = obstacles[np.argmax(obstacles[:, 4])][4]
	safety = np.hypot(dx_max, dy_max) 
	return safety
def determine_bounds(obstacles: np.ndarray) -> np.ndarray:
	obstacle_centers = obstacles[:,:3]
	obstacle_halfsizes = obstacles[:,3:] 
	x_min = np.min(obstacle_centers[:,0] - obstacle_halfsizes[:,0])
	x_max = np.max(obstacle_centers[:,0] + obstacle_halfsizes[:,0])

	y_min = np.min(obstacle_centers[:,1] - obstacle_halfsizes[:,1])
	y_max = np.max(obstacle_centers[:,1] + obstacle_halfsizes[:,1])

	z_min = np.min(obstacle_centers[:,2] - obstacle_halfsizes[:,2])
	z_max = np.max(obstacle_centers[:,2] + obstacle_halfsizes[:,2])

	# The box that bounds the obstacles
	bounds = np.array([x_min, x_max, y_min, y_max, z_min, z_max])
	return bounds


# Helper functions for the RRT algorithm
def sample_random_state(bounds):
	return np.append(np.random.uniform(bounds[::2], bounds[1::2], 3), 0)

def check_collision(start_point, end_point, obstacles):
	"""Returns `True` if a collision is detected""" 
	direction_vector = end_point - start_point

	# Calculate the minimum and maximum values for each dimension
	min_values = np.minimum(start_point, end_point)
	max_values = np.maximum(start_point, end_point)

	# Check collision for each obstacle
	obstacle_centers = obstacles[:, :3]
	obstacle_halfsizes = obstacles[:, 3:]
	collision_mask = np.all((obstacle_centers - obstacle_halfsizes) <= max_values, axis=1) & np.all((obstacle_centers + obstacle_halfsizes) >= min_values, axis=1)

	return np.any(collision_mask)

def even(int_num):
	if int_num % 2 == 0:
		return True
	return False

def rrt(obstacles: np.ndarray, start_waypoint: np.ndarray, goal_waypoint: np.ndarray, states=None, connections=None):

	s_start = start_waypoint[:3]
	s_goal = goal_waypoint[:3]

	if states is None:
		states = np.empty((0, 4))
		connections = {}
			
	# RRT code
	step_dist = 3
	states = np.vstack((states, s_start))
	completion_dist = 5
	max_iter = 10000
	s_near = np.array([np.inf, np.inf, np.inf, 0])
	iteration = 0
	goal_reached = False

	while np.linalg.norm(s_goal[:3] - s_near[:3]) > completion_dist and iteration < max_iter:
		# 1.
		iteration += 1

		if even(iteration):
			s_new = s_goal
		else:
			s_new = sample_random_state(bounds)
		
		# 2.
		s_neighbor = states[np.argmin(np.linalg.norm(s_new[:3] - states[:,:3], axis=1))]

		# 3.
		u_hat = (s_new[:3] - s_neighbor[:3]) / np.linalg.norm(s_new[:3] - s_neighbor[:3])

		# 4.
		s_near = np.append(s_neighbor[:3] + u_hat * step_dist, 0)

		#5. 
		if not check_collision(s_neighbor[:3], s_near[:3], obstacles):
			connections[tuple(s_near)] = tuple(s_neighbor)
			states = np.vstack((states, s_near))

		if np.linalg.norm(s_goal[:3] - s_near[:3]) <= completion_dist:
			goal_reached = True
			print("The RRT algorithm has found the goal state")


	if goal_reached:

		# Add the goal state to the connections dictionary
		connections[tuple(s_near)] = tuple(s_neighbor)

		# Generate a path of waypoints
		path = []
		current_state = tuple(s_near)
		while current_state != tuple(s_start):
			previous_state = connections[current_state]
			path.append(previous_state)
			current_state = previous_state
		path = path[::-1]
		path = np.array(path)

		# Initialize an empty list for the shortened path
		shortened_path = []

		# Set the current state equal to the start state
		current_state = path[0]

		# Check for collision between the start state and all other states in the path
		collision_results = np.array([check_collision(path[0][:3], state[:3], obstacles) for state in path])


		# Get the maximum index that is False (indicating no collision)
		last_false_index = np.where(collision_results == False)[0][-1]

		# Append the path state corresponding to the max_false_index to the shortened_path list
		shortened_path.append(path[last_false_index])

		# Repeat steps 3-5 until reaching the end state
		while not np.array_equal(current_state, path[-1]):
			print("Count")
			# Update the current state to be the last state added to the shortened path
			current_state = shortened_path[-1]

			# Check for collision between the current state and all other states in the path
			collision_results = np.array([check_collision(current_state[:3], state[:3], obstacles) for state in path])

			# Get the maximum index that is False (indicating no collision)
			last_false_index = np.where(collision_results == False)[0][-1]

			# Append the path state corresponding to the max_false_index to the shortened_path list
			shortened_path.append(path[last_false_index])

		shortened_path = np.array(shortened_path)
		print("The path has been shortened")

	else:
		print("Failed to find a path")

	return shortened_path, states, connections





def is_inside_obstacle(points, obstacle):
		center, halfsizes = obstacle[:3], obstacle[3:]
		return np.all(np.abs(points - center) <= halfsizes, axis=1)

def filter_points(obstacles):
		xmin, ymin, zmin = np.min(obstacles[:,:3] - obstacles[:,3:], axis=0)
		xmax, ymax, zmax = np.max(obstacles[:,:3] + obstacles[:,3:], axis=0)

		res = 100
		x = np.arange(xmin, xmax, res)
		y = np.arange(ymin, ymax, res)
		z = np.arange(10.0, zmax, res)

		grid = np.array(np.meshgrid(x, y, z)).T.reshape(-1,3)
		
		mask = np.zeros(grid.shape[0], dtype=bool)
		for obstacle in obstacles:
				mask |= is_inside_obstacle(grid, obstacle)

		return grid[~mask]
from scipy.spatial import KDTree
import heapq

def a_star(start, goal, points, kd_tree):
		#kd_tree = KDTree(points)
		start_idx = kd_tree.query([start], k=1)[1][0]
		goal_idx = kd_tree.query([goal], k=1)[1][0]

		frontier = [(0, start_idx)]
		came_from = {start_idx: None}
		cost_so_far = {start_idx: 0}
		goal_reached = False

		while frontier:
				current_priority, current_idx = heapq.heappop(frontier)
				if current_idx == goal_idx:
						goal_reached = True
						print("Goal reached")
						break

				current_point = points[current_idx]
				_, neighbors = kd_tree.query([current_point], k=7)  # adjust k based on how many neighbors you want
				neighbors = neighbors[0]

				for neighbor_idx in neighbors:
						if neighbor_idx != current_idx:  # Avoid self as neighbor due to floating point inaccuracies
								if np.linalg.norm(points[current_idx] - points[neighbor_idx]) < 100.0 + 1.0: # Avoid navigating to further away neighbors that have potential for collision.
									new_cost = cost_so_far[current_idx] + np.linalg.norm(points[current_idx] - points[neighbor_idx])
									if neighbor_idx not in cost_so_far or new_cost < cost_so_far[neighbor_idx]:
											cost_so_far[neighbor_idx] = new_cost
											priority = new_cost + np.linalg.norm(points[goal_idx] - points[neighbor_idx])
											heapq.heappush(frontier, (priority, neighbor_idx))
											came_from[neighbor_idx] = current_idx

		if not goal_reached:
				return []

		# Reconstruct path
		current_idx = goal_idx
		path = []
		while current_idx is not None:
				path.append(points[current_idx])
				current_idx = came_from[current_idx]
		path.reverse()

		# Add headings to path
		path = np.array(path)
		zeros = np.zeros((path.shape[0], 1))  # create an array of zeros with same number of rows as 'array'
		path = np.hstack((path, zeros))  # horizontally stack the zeros onto 'array'

		return path

# Here's how you can use this function:


if __name__ == "__main__":
	filename = "colliders.csv"
	obstacles = read_obstacle_data(filename)
	offsets = determine_offsets(obstacles)
	safety = determine_safety(obstacles)
	bounds = determine_bounds(obstacles)
	pdb.set_trace()
	filtered_points = filter_points(obstacles)
	leader_start = np.array([300, -300, 10, 0])  # replace with actual start point
	leader_goal = np.array([-300, 300, 55, 0])  # replace with actual goal point
	leader_speed = 3.5 # m/s
	kd_tree_obs = KDTree(obstacles[:, :2]) # Obstacle ground position
	kd_tree_coarse_space = KDTree(filtered_points)
	leader_drone = Leader(leader_start, leader_goal, leader_speed, filtered_points, obstacles, kd_tree_obs, kd_tree_coarse_space)
	for _ in range(1000):
		leader_drone.advance_pos()
	leader_path = np.array(leader_drone.current_pos)
	leader_timer = np.array(leader_drone.timer)
	leader_follow_points = np.array(leader_drone.follow_point)
	leader_follow_pos = np.array(leader_drone.follow_pos)
	leader_data= np.column_stack((leader_timer, leader_follow_pos, leader_follow_points))

	np.savez('leader_drone.npz', leader_data=leader_data, leader_path=leader_path, leader_timer=leader_timer, leader_follow_points=leader_follow_points, leader_follow_pos=leader_follow_pos)
	pdb.set_trace()
	"""
	pdb.set_trace()
	path = a_star(start, goal, filtered_points)
	pdb.set_trace()
	start_waypoint = np.array([0,0,0,0])
	goal_waypoint = np.array([-600, -600, 200, 0])
	# shortened_path: A 2d array containing coarse waypoints
	# states: A 2d array containing the sampled waypoints
	# connections: A dictionary containing goal : start pairs `state` array index mappings
	shortened_path, states, connections = rrt(obstacles, start_waypoint,  goal_waypoint)
	np.savez('state_space.npz', shortened_path=shortened_path, states=states, connections=connections, start_waypoint=start_waypoint, goal_waypoint=goal_waypoint)
	pdb.set_trace()
	

	conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
	drone = PathPlanning(conn, obstacles, shortened_path, geodetic_home)

	time.sleep(2)
	drone.start()
	"""
	geodetic_home = np.array([-122.397450,  37.792480, 0.0])
	pdb.set_trace()
	conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
	drone = PathPlanning(conn, obstacles, geodetic_home, leader_data, filtered_points)
	time.sleep(2)
	drone.start()

	"""
	States:
		
		Sn = [[Sx1, Sy2, Sz1, Spsi1], ... [Sxn, Syn, Szn, Spsin]]

	Obstacles:

		Ob = [[x1, y1, z1, dx1, dy1, dz1], ..., [xn, yn, zn, dxn, dyn, dzn]]
	0. Use a star to make a coarse path for the leader to follow to a particular point on the map
	1. Position update
	2. Advance the position of the leader:
	   If the leader is more than M meters from the goal location on the map:
	     Use the potential field to advance towards the next path waypoint
	     while avoiding artificial obstacles
	   If the leader is within M meters from the goal location on the map:
	     Use the potential field to advance towards the goal location while avoiding 
	     artificial obstacles
	3. Return a follow position that's a set number of meters behind the leader in the unit direction
	   from the leader's resulting direction of motion to the leader.
	"""




	

	