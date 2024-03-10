"""
Areas for improvement
.The potential field hitting local minima
.Computational efficiency

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
from scipy.spatial import KDTree
import heapq
import csv

class States(Enum):
	# Manual flight mode
	MANUAL = auto()

	# Motors are being armed
	ARMING = auto()

	# Landing is occuring
	LANDING = auto()

	# The engines are being disarmed
	DISARMING = auto()

	# The drone is tracking a moving target
	TRACK = auto()

	# Global planning only
	GLOBAL = auto()

	# The drone is hovering
	HOVER = auto()

class PathPlanner(Drone):
	def __init__(self, connection, states, states_kd, obstacles, obstacles_kd):
		super().__init__(connection)
		self.in_mission = True
		self.check_state = {}
		self.target = None

		# These are data used in path planning
		self.leader_drone = leader_drone
		self.states = states
		self.states_kd = states_kd
		self.obstacles = obstacles
		self.obstacles_kd = obstacles_kd

		# Extract geodetic home from file
		self.geo_home = geo_home("colliders.csv")

		# Initial state
		self.flight_state = States.MANUAL
		
		# Callback registration
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.STATE, self.state_callback)

	def local_position_callback(self):




	def hover_transition(self):
		print("Transitioning to a hover state")
		self.flight_state = States.HOVER

	def plan_path(self, target):



	def state_callback(self):
		if self.in_mission:
			if self.flight_state == States.MANUAL:
				self.arming_transition()
			elif self.flight_state == States.ARMING:
				if self.armed:
					self.set_home_position(*self.geo_home)
					self.plan_path(self.leader_drone.current_pos)


	def manual_transition(self):
		pass

	def arming_transition(self):
		self.flight_state = States.ARMING
		print("Arming transition")
		self.arm()
		time.sleep(2)
		self.take_control()

	def waypoint_transition(self):
		if len(self.path) > 0:
			self.target = self.path.pop(0)
			print("Waypoint transition", self.target)

	def landing_transition(self):
		pass

	def disarming_transition(self):
		pass

	def following_transition(self):
		pass

	def start(self):
		self.start_log("Logs", "NavLog.txt")
		print("Starting connection")
		self.connection.start()
		self.stop_log()

# Helper functions
def sample_random_state(bounds):
	return np.append(np.random.uniform(bounds[::2], bounds[1::2], 3), 0)

def check_collision(start_point, end_point, obstacles):
	"""Returns `True` if a collision is detected""" 

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


def rrt(start, goal, obstacles):
	"""
	start and goal are numpy arrays x,y,z,heading
	"""
	obstacles = np.genfromtxt('colliders.csv', delimiter=',', skip_header=2)
	x_center_min = obstacles[np.argmin(obstacles[:, 0])][0]
	y_center_min = obstacles[np.argmin(obstacles[:, 1])][1]
	z_center_min = obstacles[np.argmin(obstacles[:, 2])][2]
	dx_max = obstacles[np.argmax(obstacles[:, 3])][3]
	dy_max = obstacles[np.argmax(obstacles[:, 4])][4]
	safety = np.hypot(dx_max, dy_max)
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
	# RRT code
	states = np.empty((0, 4))
	step_dist = 3
	connections = {}
	s_start = start
	s_goal = goal
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
		pdb.set_trace()
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

		return shortened_path

def construct_obstacles(filename):
	"""Retruns a 2d array with obstacle x, y, z, dx, dy, dz values along with a KDTree referencing ground positions of obstacles x, y"""
	return np.genfromtxt(filename, delimiter=',', skip_header=2)

def is_inside_obstacle(states, obstacle):
	center, halfsizes = obstacle[:3], obstacle[3:]
	return np.all(np.abs(states - center) <= halfsizes, axis=1)

def geo_home(filename):
	"""Extracts and returns geodetic home from the colliders file"""
	with open(filename) as file:
		reader = csv.reader(file)
		first_line = next(reader)
	lon = float(first_line[1].split(' lon0 ')[1])
	lat = float(first_line[0].split('lat0 ')[1])
	alt = 0.
	return np.array([lon, lat, alt])

def geo_to_local(geo, geo_home):
	"""Converts a geodetic coordinate into a local coordinate, then returns it"""
	pass

def nearest_obstacle(point, obstacles, obstacles_kd):

	obs_idx = obstacles_kd.query(np.array([point[:2]]), k=10)[1][0]
	nearest_obs = obstacles[0] # Default nearest obstacle
	for idx in obs_idx:
		if obstacles[idx][2] + obstacles[idx][5] > point[2]:
			nearest_obs = obstacles[idx]
			break
	return nearest_obs


if __name__ == "__main__":
	obstacles = construct_obstacles("colliders.csv")
	obstacles_kd = KDTree(obstacles[:,:2])
	resolution = 25.
	states = discretize_environment(obstacles, resolution)
	states_kd = KDTree(states)
	leader_start = np.array([50.,50.,25.])#np.array([50.,50.,200.])
	leader_goal =  np.array([-175.,200.,50.])#[100., 100., 50.] [-300., -300., 50.] [300, 500, 100]localmin np.array([200., 200., 200.])np.array([100., 100., 50.])
	leader = Leader(leader_start, leader_goal, states, states_kd, obstacles, obstacles_kd)

	conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
	drone = PathPlanner(conn, leader, states, states_kd, obstacles, obstacles_kd)
	time.sleep(2)
	drone.start()