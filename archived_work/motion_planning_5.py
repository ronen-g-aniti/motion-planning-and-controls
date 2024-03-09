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
from enum import Enum, auto
from environment import Environment
from geodetic_position import GeodeticPosition
from halfsize import HalfSize
from local_position import LocalPosition
import msgpack
import numpy as np
from obstacle import Obstacle
from obstacle_collection import ObstacleCollection
from obstacle_file_reader import ObstacleFileReader
from potential_field import PotentialField
from prm import ProbabilisticRoadmap
import receding_horizon_utils as rh
from rrt import RapidlyExploringRandomTree
from sklearn.neighbors import KDTree
from state import State
import time
from typing import List
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID


import pdb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class States(Enum):
	MANUAL = auto()
	ARMING = auto()
	TAKEOFF = auto()
	WAYPOINT = auto()
	LANDING = auto()
	DISARMING = auto()


class PathPlanning(Drone):
	def __init__(self, connection: MavlinkConnection, environment: Environment, waypoints: List[List[float]], geodetic_home: GeodeticPosition):
		super().__init__(connection)

		self.next_waypoint = np.array([0.0, 0.0, 0.0, 0.0])
		self.target_position = np.array([0.0, 0.0, 0.0, 0.0])
		self.waypoints = waypoints
		self.goal_altitude = waypoints[-1][2]
		self.environment = environment
		self.in_mission = True
		self.check_state = {}
		
		self.flight_state = States.MANUAL

		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.STATE, self.state_callback)
		
	def local_position_callback(self):

		# Have the receding horizon planner command an available position on the edge of 
		# the horizon that's in the direction of the next waypoint.
		deadband_radius = 2 # meters
		if self.flight_state == States.TAKEOFF:
			if abs(self.local_position[2]) > 0.95 * self.goal_altitude:
				self.waypoint_transition()
		elif self.flight_state == States.WAYPOINT:
			self.local_planner()
			if np.linalg.norm(self.local_position[0:2] - self.next_waypoint[0:2]) < deadband_radius:
				self.waypoint_transition()

	def velocity_callback(self):
		pass

	def state_callback(self):
		if self.in_mission:
			if self.flight_state == States.MANUAL:
				self.arming_transition()
			elif self.flight_state == States.ARMING:
				if self.armed:
					self.set_home_position(geodetic_home.longitude, geodetic_home.latitude, geodetic_home.altitude)
					self.takeoff_transition()

	def local_planner(self):
		
		PLANNING_RADIUS = 40 # meters
		OBSTACLE_DETECTION_RADIUS = 15
		# Apply an obstacle avoidance direction vector when drone is within ~14 meters of the point obstacle.
		distances = np.array([np.linalg.norm(obstacle.ground_position - self.local_position[0:2]) for obstacle in self.environment.obstacles.detected_obstacle_list])
		min_distance_index = np.argmin(distances)
		closest_obstacle = self.environment.obstacles.detected_obstacle_list[min_distance_index]
		unit_vector_from_detected_obs = np.array([0, 0])
		if distances[min_distance_index] < OBSTACLE_DETECTION_RADIUS:
			print("Obstacle detected nearby")
			if distances[min_distance_index] <= closest_obstacle.safety:
				print("Applying potential-field obstacle avoidance")
				vector_from_detected_obs_to_drone = self.local_position[0:2] - closest_obstacle.ground_position
				print("within this many meters now ",np.linalg.norm(vector_from_detected_obs_to_drone))
				unit_vector_from_detected_obs = vector_from_detected_obs_to_drone / np.linalg.norm(vector_from_detected_obs_to_drone)

		# Determine the direction vector the the next waypoint
		vector_from_drone_to_next_waypoint = self.next_waypoint[0:2] - self.local_position[0:2]
		unit_vector_to_next_waypoint = vector_from_drone_to_next_waypoint / np.linalg.norm(vector_from_drone_to_next_waypoint)

		# Add the two direction vectors being applied and get the direction of that
		unit_vector_sum = unit_vector_to_next_waypoint + 0.25 * unit_vector_from_detected_obs
		result_unit_vector = unit_vector_sum / np.linalg.norm(unit_vector_sum)

		# Command the drone to travel in that direction
		command_position = self.local_position[0:2] + PLANNING_RADIUS * result_unit_vector
		self.cmd_position(command_position[0], command_position[1], self.next_waypoint[2], 0)

		
	def manual_transition(self):
		pass

	def arming_transition(self):
		self.flight_state = States.ARMING
		print("Arming transition")
		self.arm()
		time.sleep(2)
		self.take_control()



	def waypoint_transition(self):
		self.flight_state = States.WAYPOINT
		self.next_waypoint = self.waypoints.pop(0)

	def takeoff_transition(self):
		self.flight_state = States.TAKEOFF
		print("Transitioning to takeoff state")
		self.takeoff(self.goal_altitude)

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
		self.stop_log()
		







if __name__ == "__main__":
	filename = "colliders.csv"
	reader = ObstacleFileReader(filename)
	geodetic_home = reader.extract_geodetic_home()
	obstacle_array = reader.extract_obstacles_as_array()
	obstacle_collection = ObstacleCollection(obstacle_array, geodetic_home)
	geodetic_goal = GeodeticPosition(-122.3994, 37.7951, 10)
	environment = Environment(geodetic_home, obstacle_collection)
	current_geodetic_position = geodetic_home
	current_local_position = current_geodetic_position.local_relative_to(geodetic_home)
	goal_position_in_local_frame = geodetic_goal.local_relative_to(geodetic_home)
	start_state = State(environment, goal_position_in_local_frame, current_local_position)
	goal_state = State(environment, goal_position_in_local_frame, goal_position_in_local_frame)
	

	# Generate the maps.  #
	# State Space Generation #
	# Generate a state space
	# Create waypoints
	# Those waypoints are the global plan
	# --------------------------

	# Rapidly-exploring random tree (RRT)
	print("Computing a global plan from the start state to the goal state using an RRT algorithm.")
	rrt = RapidlyExploringRandomTree(environment, start_state, goal_state)
	rrt_waypoints = rrt.run()
	print(rrt_waypoints)


	# Detect more obstacles in the environment not initially included on the map of the environment
	standard_halfsize = HalfSize(10, 10, 100)
	local_position = LocalPosition(20, 20, 100)
	environment.obstacles.add_detected_obstacle(Obstacle(local_position, standard_halfsize))
	local_position = LocalPosition(40, 40, 100)
	environment.obstacles.add_detected_obstacle(Obstacle(local_position, standard_halfsize))
	for i in range(30):
		local_position = LocalPosition(np.random.uniform(low=-200, high=400), np.random.uniform(low=-200, high=400), 100)
		environment.obstacles.add_detected_obstacle(Obstacle(local_position, standard_halfsize))

	rrt.visualize(plot_entire_state_space=True)
	
	# Probablistic roadmap (PRM)
	#TODO: Needs a shortcut algorithm.
	#TODO: Needs to compute cost.
	print("Computing a global plan from the start state to the goal state using a PRM algorithm.")
	#prm = ProbabilisticRoadmap(environment, start_state, goal_state)
	#prm_waypoints = prm.determine_waypoints()
	#prm.visualize(plot_entire_state_space=True)
	
	# Potential field #1
	print("Computing a global plan from the start state to the goal state using a potential field algorithm.")
	#potential_field_1 = PotentialField(environment, start_state, goal_state)
	#potential_field_1_waypoints = potential_field_1.return_waypoints()
	#potential_field_1.visualize(plot_original_trajectory=True, plot_with_gradient_field=True, plot_shortened_path=True)
	

	# Establish connection with drone simulator
	conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
	drone = PathPlanning(conn, environment, rrt_waypoints, geodetic_home)
	time.sleep(2)
	drone.start()

	"""
	# Potential field #1 (check)
	print("Computing a global plan from the start state to the goal state using a potential field algorithm.")
	potential_field_1 = PotentialField(environment, start_state, goal_state)
	potential_field_1_waypoints = potential_field_1.determine_waypoints()


	# Potential field #2
	print("Computing a global plan from the start state to the goal state using a potential field algorithm.")

	# 2D Array (Distance transform)
	print("Computing a global plan from the start state to the goal state using a 2D grid search algorithm.")

	# Medial axis transform grid
	print("Discretizing the environment into a grid data structure and searching it using A* to determine a pat.h")

	# Uniform mesh of sample states (check)
	print("Discretizing the environment into a uniform mesh of sample states and searching the resulting graph using an A* search algorithm.")

	# Voronoi graph
	print("Discretizing the environment into a graph data structure using a Voronoi graph algorithm and searching the resulting state space using an A* search routine.")

	"""