import argparse
import csv
from enum import Enum, auto
import msgpack
import numpy as np
import pdb
import time
from typing import Tuple, List, Dict
from udacidrone import Drone 
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

from environment import Environment
from geodetic_position import GeodeticPosition
from graph import Graph
from grid import Grid, HeightMap
from halfsize import HalfSize
from local_position import LocalPosition
import matplotlib.pyplot as plt
from prm import ProbabilisticRoadmap
from start_goal_pair import StartGoalPair
from state import State
from obstacle import Obstacle
from obstacle_collection import ObstacleCollection
from obstacle_file_reader import ObstacleFileReader
from rrt import RapidlyExploringRandomTree
from potential_field import PotentialField
import time
import pdb


class States(Enum): 

	MANUAL = auto()
	ARMING = auto()
	TAKEOFF = auto()
	WAYPOINT = auto()
	LANDING = auto()
	DISARMING = auto()
	PLANNING = auto()

class MotionPlanning(Drone):
	"""
	# Initialize the drone
	# Position callback
	# Plan Path
		# Run A Star search on the search space from current location on grid to a target location within the prediction horizon.
	"""

	def __init__(self, connection, waypoints, geodetic_home, start_state, goal_state):
		super().__init__(connection)

		self.geodetic_home = geodetic_home
		self.start_state = start_state
		self.goal_state = goal_state
		self.target_local = np.array([0.0, 0.0, 0.0, 0.0])
		self.waypoints = waypoints[1:]
		print(self.waypoints)
		self.in_mission = True
		self.check_state = {}

		self.flight_state = States.MANUAL

		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
		self.register_callback(MsgID.STATE, self.state_callback)


	def local_position_callback(self):
		print(self.flight_state)
		print(abs(self.local_position[2]))
		print(self.goal_state.local_position.down)
		if self.flight_state == States.TAKEOFF:
			if abs(self.local_position[2]) > 0.95 * self.goal_state.local_position.down:
				self.waypoint_transition()
		drone_speed = np.linalg.norm(self.local_velocity)
		deadband = 0.25 + 0.25 * drone_speed
		if self.flight_state == States.WAYPOINT:
			if np.linalg.norm(self.local_position[0:2] - self.goal_state.position_in_3d[0:2]) < deadband:
				self.landing_transition()
			if np.linalg.norm(self.local_position[0:2] - self.target_local[0:2]) < deadband:
				self.waypoint_transition()

	def velocity_callback(self):
		if self.flight_state == States.LANDING:
			if (self.global_position[2] - self.geodetic_home.altitude[2]) < 0.1:
				if abs(self.local_position[2]) < 0.01:
					self.disarming_transition()

	def state_callback(self):
		if self.in_mission:
			if self.flight_state == States.MANUAL:
				self.arming_transition()
			elif self.flight_state == States.ARMING:
				if self.armed:
					self.plan_path()
			elif self.flight_state == States.PLANNING:
				self.takeoff_transition()
			elif self.flight_state == States.DISARMING:
				if ~self.armed & ~self.guided:
					self.manual_transition()
	
	def manual_transition(self):
		self.flight_state = States.MANUAL
		print("Manual transition")
		self.stop()
		self.in_mission = False

	def arming_transition(self): 
		self.flight_state = States.ARMING
		global_home = self.geodetic_home
		self.set_home_position(global_home.longitude, global_home.latitude, global_home.altitude)
		print("Arming transition")
		self.take_control()
		self.arm()

	def takeoff_transition(self):
		self.flight_state = States.TAKEOFF
		print("Takeoff transition")
		self.takeoff(abs(self.target_local[2]))
		print(self.target_local[2])

	def waypoint_transition(self):
		self.flight_state = States.WAYPOINT
		print("Waypoint transition")
		print(self.target_local)
		self.cmd_position(self.target_local[0], self.target_local[1], self.target_local[2], self.target_local[3])

	def landing_transition(self):
		self.flight_state = States.LANDING
		print("Landing transition")
		self.land()

	def disarming_transition(self):
		self.flight_state = States.DISARMING
		print("Disarming transition")
		self.disarm()
		self.release_control()

	def plan_path(self):
		self.flight_state = States.PLANNING
		self.target_local[2] = self.goal_state.local_position.down

	def send_waypoints(self):
		print("Sending waypoints to simulator...")
		data = msgpack.dumps(self.waypoints)
		self.connection._master.write(data)

	def start(self):
		self.start_log("Logs", "NavLog.txt")
		print("Starting connection")
		super().start()
		self.stop_log()



if __name__ == "__main__":
	filename = "colliders.csv"
	reader = ObstacleFileReader(filename) 
	geodetic_home = reader.extract_geodetic_home()
	obstacle_array = reader.extract_obstacles_as_array() 
	obstacle_collection = ObstacleCollection(obstacle_array, geodetic_home)
	geodetic_goal = GeodeticPosition(-122.3994, 37.7951, 10)
	environment = Environment(geodetic_home, obstacle_collection)
	current_geodetic_position = GeodeticPosition(-122.39745, 37.79248, 0) #The geodetic position of the drone at the position update
	current_local_position = current_geodetic_position.local_relative_to(geodetic_home)
	goal_position_in_local_frame = geodetic_goal.local_relative_to(geodetic_home)
	current_state = State(environment, goal_position_in_local_frame, current_local_position)
	goal_state = State(environment, goal_position_in_local_frame, goal_position_in_local_frame)
	rapidly_exploring_random_tree = RapidlyExploringRandomTree(environment, current_state, goal_state)
	waypoints = rapidly_exploring_random_tree.run()
	pdb.set_trace()

	#conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True)
	conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
	drone = MotionPlanning(conn, waypoints, geodetic_home, current_state, goal_state)
	time.sleep(1)

	drone.start()
