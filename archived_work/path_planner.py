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

	TRACK = auto()

	HOVER = auto()

class PathPlanner(Drone):
	def __init__(self, connection, leader_drone, states, states_kd, obstacles, obstacles_kd):
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
		
		# Replanning counter to aid in simulating the leader-follower scenario
		self.count = 0

		# Initial state
		self.flight_state = States.MANUAL
		
		# Callback registration
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.STATE, self.state_callback)

		# Record flight data for analysis
		self.position_history_drone = []
		self.position_history_target = []


	def local_position_callback(self):
		self.count += 1
		leader.step_forward()
		

	
		self.current_pos = np.array([self.local_position[0], self.local_position[1], self.local_position[2] * -1.0])




		if self.flight_state == States.TRACK:
			deadband_dist = 15.
			if np.linalg.norm(self.current_pos[:2] - self.target[:2]) <= deadband_dist:
				if np.linalg.norm(self.current_pos[2] - self.target[2]) <= deadband_dist:
					self.waypoint_transition()
			# Set target to be leader drone (instead of next path waypoint) if this drone is within 50 meters of the leader
			if np.linalg.norm(self.leader_drone.current_pos - self.current_pos) <= 50.0:
				print("within this many meters of target: ", np.linalg.norm(self.leader_drone.current_pos - self.current_pos))
				self.path = []
				self.target = self.leader_drone.current_pos
				if self.leader_drone.at_goal:
					if np.linalg.norm(self.leader_drone.current_pos - self.current_pos) < 10.0:
						self.hover_transition()

			else:
				if self.count % 300 == 0 and self.count != 0:
					self.plan_path(self.leader_drone.current_pos)
				if len(self.path) == 1:
					self.plan_path(self.leader_drone.current_pos)

			self.tracker()

		# Recording data for analysis
		self.position_history_drone.append(self.current_pos)
		self.position_history_target.append(self.leader_drone.current_pos)
		if self.count > 700 or self.flight_state == States.HOVER:
			self.position_history_target = np.array(self.position_history_target)
			self.position_history_drone = np.array(self.position_history_drone)
			vis(self.position_history_target, self.position_history_drone, self.obstacles)
			pdb.set_trace()



	def hover_transition(self):
		print("Transitioning to a hover state")
		self.flight_state = States.HOVER

	def plan_path(self, target):
		print("Planning a path to ", target)
		self.cmd_position(*self.local_position[:2], self.local_position[2] * -1., 0) # Stay in place while planning
		self.current_pos = np.array([self.local_position[0], self.local_position[1], self.local_position[2] * -1.0])
		self.path = coarse_path(self.current_pos, target, self.states, self.states_kd) # A star
		if len(self.path) > 0:
			self.target = self.path.pop(0)
		self.flight_state = States.TRACK

	def tracker(self):
		cmd_dir = command_direction(self.current_pos, self.target, self.obstacles, self.obstacles_kd)
		step = 40. #40.
		self.current_pos = np.array([self.local_position[0], self.local_position[1], self.local_position[2] * -1.0])
		cmd_pos = self.current_pos + step * cmd_dir
		self.cmd_position(*cmd_pos, 0.)

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


class Follower:
	def __init__(self, start, goal, states, states_kd, obstacles, obstacles_kd, leader):
		self.start = start
		self.goal = goal
		self.states = states
		self.states_kd = states_kd
		self.obstacles = obstacles
		self.obstacles_kd = obstacles_kd
		self.path = coarse_path(start, goal, states, states_kd)
		self.target = self.path.pop(0)
		self.current_pos = self.start
		self.count = 0
		self.leader = leader

		# For visualization
		self.points = [start]

	def step_forward(self):
		"""Advance the leader drone N steps through its trajectory"""
		
		self.count += 1

		# Get current position
		deadband_dist = 5.0
		if np.linalg.norm(self.current_pos - self.target) <= deadband_dist:
			if len(self.path) > 0:
				self.target = self.path.pop(0)

		# Advance leader drone by some step amount every N position updates
		self.leader.step_forward()

		# Replan every 50 steps
		if self.count % 50 == 0 and self.count != 0 and np.linalg.norm(self.leader.current_pos - self.current_pos) > 50.0:
			self.path = coarse_path(self.current_pos, self.leader.current_pos, states, states_kd)
			self.target = self.path.pop(0)

		# Set target to be leader drone (instead of next path waypoint) if within 50 meters of the leader
		if np.linalg.norm(self.leader.current_pos - self.current_pos) <= 50.0:
			print("within this many meters of target: ", np.linalg.norm(self.leader.current_pos - self.current_pos))
			self.path = []
			self.target = self.leader.current_pos

		# Compute the command direction from the attractive and repulsive fields
		cmd_dir = command_direction(self.current_pos, self.target, self.obstacles, self.obstacles_kd)

		# Update position based on rule
		step = 2.0
		self.current_pos = self.current_pos + step * cmd_dir

		# For visualization
		self.points.append(self.current_pos)

class Leader:
	def __init__(self, start, goal, states, states_kd, obstacles, obstacles_kd):
		self.start = start
		self.goal = goal
		self.states = states
		self.states_kd = states_kd
		self.obstacles = obstacles
		self.obstacles_kd = obstacles_kd
		self.path = coarse_path(start, goal, states, states_kd)
		self.at_goal = False 

		# For visualization 
		self.original_path = self.path.copy()
		self.target = self.path.pop(0)
		self.current_pos = self.start

		# For visualization
		self.points = [start]

	def step_forward(self):
		"""Advance the leader drone N steps through its trajectory"""
		
		# Get current position
		deadband_dist = 5.0
		if np.linalg.norm(self.current_pos - self.target) <= deadband_dist:
			if len(self.path) > 0:
				self.target = self.path.pop(0)
			else:
				self.at_goal = True

		# Compute the command direction from the attractive and repulsive fields
		cmd_dir = command_direction(self.current_pos, self.target, self.obstacles, self.obstacles_kd)

		# Update position based on rule
		step = 2.0

		if self.at_goal:
			step = 0.0

		self.current_pos = self.current_pos + step * cmd_dir

		# For visualization
		self.points.append(self.current_pos)


def construct_obstacles(filename):
	"""Retruns a 2d array with obstacle x, y, z, dx, dy, dz values along with a KDTree referencing ground positions of obstacles x, y"""
	return np.genfromtxt(filename, delimiter=',', skip_header=2)

def discretize_environment(obstacles, resolution):
	"""Returns a 2d array of coarse states x, y, z that don't collide with obstacles"""
	xmin, ymin, zmin = np.min(obstacles[:,:3] - obstacles[:,3:], axis=0)
	xmax, ymax, zmax = np.max(obstacles[:,:3] + obstacles[:,3:], axis=0)
	x = np.arange(xmin, xmax, resolution)
	y = np.arange(ymin, ymax, resolution)
	z = np.arange(10.0, zmax, resolution)

	grid = np.array(np.meshgrid(x, y, z)).T.reshape(-1,3)
	
	mask = np.zeros(grid.shape[0], dtype=bool)
	for obstacle in obstacles:
			mask |= is_inside_obstacle(grid, obstacle)

	return grid[~mask]

def is_inside_obstacle(states, obstacle):
	center, halfsizes = obstacle[:3], obstacle[3:]
	return np.all(np.abs(states - center) <= halfsizes, axis=1)

def coarse_path(start, goal, states, states_kd):
	"""Returns the shortest path between start and goal through coarse states"""
	start_idx = states_kd.query([start], k=1)[1][0]
	goal_idx = states_kd.query([goal], k=1)[1][0]

	frontier = [(0, start_idx)]
	came_from = {start_idx: None}
	cost_so_far = {start_idx: 0}
	goal_reached = False

	while frontier:
		current_priority, current_idx = heapq.heappop(frontier)
		if current_idx == goal_idx:
			goal_reached = True
			print("Rerouting successful")
			break

		current_point = states[current_idx]
		_, neighbors = states_kd.query([current_point], k=7)  # adjust k based on how many neighbors you want
		neighbors = neighbors[0]

		for neighbor_idx in neighbors:
			if neighbor_idx != current_idx:  # Avoid self as neighbor due to floating point inaccuracies
				if np.linalg.norm(states[current_idx] - states[neighbor_idx]) < 25.0 + 1.0: # Avoid navigating to further away neighbors that have potential for collision.
					new_cost = cost_so_far[current_idx] + np.linalg.norm(states[current_idx] - states[neighbor_idx])
					if neighbor_idx not in cost_so_far or new_cost < cost_so_far[neighbor_idx]:
						cost_so_far[neighbor_idx] = new_cost
						priority = new_cost + np.linalg.norm(states[goal_idx] - states[neighbor_idx])
						heapq.heappush(frontier, (priority, neighbor_idx))
						came_from[neighbor_idx] = current_idx

	if not goal_reached:
		return []

	# Reconstruct path
	current_idx = goal_idx
	path = []
	while current_idx is not None:
		path.append(states[current_idx])
		current_idx = came_from[current_idx]
	path.reverse()
	path.append(goal)

	return path

def attractive_vector(current_point, attraction_point):
	"""Returns the direction vector of attraction"""
	attr_dir = (attraction_point - current_point) / np.linalg.norm(attraction_point - current_point)
	attr_gain = 20.0
	attr_vec = attr_gain * attr_dir
	return attr_vec

def repulsive_vector(current_point, obstacles, obstacles_kd):
	"""Returns the direction vector of repulsion"""
	# Compute the repulsive vector
	nearest_obs = nearest_obstacle(current_point, obstacles, obstacles_kd)

	# Repulsive base direction vector
	repuls_vec = current_point - np.array([nearest_obs[0], nearest_obs[1], current_point[2]])
	repuls_dir = repuls_vec / np.linalg.norm(repuls_vec)
	dist_to_obs = np.linalg.norm(repuls_vec)
	
	# Repulsive gain
	collision_dist = 10.
	detection_dist = 40.
	if dist_to_obs <= detection_dist:
		repuls_gain = 20.0 * (1 - ((dist_to_obs - collision_dist) / (detection_dist - collision_dist)))
	else:
		repuls_gain = 0.0

	# Repulsive vector 
	repuls_vec = repuls_dir * repuls_gain
	return repuls_vec

def command_direction(current_point, target, obstacles, obstacles_kd):
	"""Returns the result direction vector"""
	attr_vec = attractive_vector(current_point, target)
	repuls_vec = repulsive_vector(current_point, obstacles, obstacles_kd)
	result_vec = attr_vec + repuls_vec
	cmd_dir = result_vec / np.linalg.norm(result_vec)
	return cmd_dir

def command_point(current_point, command_direction):
	"""Returns the command point to fly to"""
	pass

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

def nearest_state(point, states, states_kd):
	pass

def vis(position_history_target, position_history_drone, obstacles):
	import matplotlib.pyplot as plt
	import matplotlib.animation as animation
	import numpy as np
	from PIL import Image

	fig, ax = plt.subplots()
	drone_scatter = ax.scatter([], [], s=3, label="Drone", color="blue")
	target_scatter = ax.scatter([], [], s=3, label="Target", color="lime")
	obstacles_scatter = ax.scatter(obstacles[:, 0], obstacles[:, 1], s=1, color="red")

	def animate(frame):
		k = frame  # Index of the point to plot on this frame

		# Select every 10th point for drone and target arrays
		drone_data = position_history_drone[:k+1, :2]  # Select first two columns, every 10th point
		target_data = position_history_target[:k+1, :2]  # Select first two columns, every 10th point

		drone_scatter.set_offsets(drone_data)
		target_scatter.set_offsets(target_data)

		return [drone_scatter, target_scatter]

	def init():
		drone_scatter.set_offsets([])
		target_scatter.set_offsets([])

		return [drone_scatter, target_scatter, obstacles_scatter]

	num_frames = min(len(position_history_drone), len(position_history_target))
	plt.title("The Drone Tracking a Moving Target")
	plt.xlabel("North (m)")
	plt.ylabel("East (m)")
	plt.legend()

	# Create a list to store each frame of the animation
	frames = []

	# Create the animation and append each frame to the frames list
	ani = animation.FuncAnimation(fig, animate, frames=num_frames, init_func=init, blit=True, interval=25)
	for frame in range(num_frames):
		ax.clear()
		animate(frame)
		obstacles_scatter = ax.scatter(obstacles[:, 0], obstacles[:, 1], s=1, color="red")  # Add this line to redraw the obstacles in each frame
		fig.canvas.draw()
		image = Image.frombytes('RGB', fig.canvas.get_width_height(), fig.canvas.tostring_rgb())
		frames.append(image)

	# Save the frames as a GIF using PIL
	frames[0].save('animation.gif', save_all=True, append_images=frames[1:], optimize=False, duration=200, loop=0)

	plt.show()

if __name__ == "__main__":
	obstacles = construct_obstacles("colliders.csv")
	obstacles_kd = KDTree(obstacles[:,:2])
	resolution = 25.
	states = discretize_environment(obstacles, resolution)
	states_kd = KDTree(states)
	leader_start = np.array([50.,50.,25.])#np.array([50.,50.,200.])
	leader_goal =  np.array([-175.,200.,50.])#[100., 100., 50.] [-300., -300., 50.] [300, 500, 100]localmin np.array([200., 200., 200.])np.array([100., 100., 50.])
	leader = Leader(leader_start, leader_goal, states, states_kd, obstacles, obstacles_kd)
	#follower_start = np.array([0, 0, 0])
	#follower_goal = leader_start
	#follower = Follower(follower_start, follower_goal, states, states_kd, obstacles, obstacles_kd, leader)

	"""
	# Quick visual
	self.position_history_target = [element for element in self.position_history_target if element is not None]
	self.position_history_drone = np.array(self.position_history_drone)
	self.position_history_target = np.array(self.position_history_target)
	plt.scatter(obstacles[:, 0], obstacles[:, 1], s=1, color="red"); plt.scatter(self.position_history_drone[:,0], self.position_history_drone[:,1]); plt.scatter(self.position_history_target[:,0], self.position_history_target[:,1]);plt.show()
	
	import matplotlib.pyplot as plt
	from mpl_toolkits.mplot3d import Axes3D
	import matplotlib.animation as animation

	# Create the figure
	fig = plt.figure()

	# Add 3D subplot
	ax = fig.add_subplot(111, projection='3d')

	# Scatter plot of the grid points
	scatter = ax.scatter(states[:, 0], states[:, 1], states[:, 2], s=1, alpha=1, color='lime')

	# Set labels
	ax.set_xlabel('North (m)')
	ax.set_ylabel('East (m)')
	ax.set_zlabel('Altitude (m)')

	# Define altitude ranges
	altitudes = np.arange(min(states[:, 2]), max(states[:, 2])+1, resolution)  # Adjust the step size as needed
	max_altitude = max(states[:, 2])  # Get maximum altitude

	# Set z lim
	ax.set_zlim(0, max_altitude)

	# Set title
	ax.set_title(f'The Configuration Space')  # Adjust the range as needed

	# Function to update the plot for each frame
	def update(frame):
		ax.collections.clear()  # Clear points

		# Filter states for the current altitude range
		mask = (states[:, 2] >= altitudes[frame]) & (states[:, 2] < altitudes[frame] + resolution)
		states_in_range = states[mask]

		if len(states_in_range) > 0:  # Check that there are points in the slice
			# Scatter plot of the filtered points
			scatter = ax.scatter(states_in_range[:, 0], states_in_range[:, 1], states_in_range[:, 2], s=1, alpha=1, color='lime')

		return scatter,  # Return a tuple containing the scatter plot as the only artist to update

	# Create animation
	ani = animation.FuncAnimation(fig, update, frames=len(altitudes), interval=200, blit=True)

	# Show the animation
	plt.show()


	# Set up formatting for the movie files
	plt.rcParams['animation.convert_path'] = "C:\\Program Files\\ImageMagick-7.1.1-Q16-HDRI\\magick.exe"
	Writer = animation.writers['imagemagick']
	writer = Writer(fps=2, metadata=dict(artist='Me'), bitrate=1800)
	# Save the animation
	ani.save('3Dscatter.gif', writer=writer)

	#plt.show()
	pdb.set_trace()
	
	"""


	"""
	
	plt.scatter([point[0] for point in leader.path],[point[1] for point in leader.path], color="lime")
	plt.scatter([point[0] for point in follower.path],[point[1] for point in follower.path], color="magenta")
	
	step = 0
	leader_pos = []
	follower_pos = []
	leader_path = []
	follower_path = []
	leader_target = []
	follower_target = []

	for _ in range(500):
		follower.step_forward()
		step += 1
		leader_pos.append(leader.current_pos) # numpy 3-element 1-row array x,y,z
		follower_pos.append(follower.current_pos) # numpy 3-element 1-row array x,y,z
		leader_path.append(leader.original_path) # List of numpy 3-element 1-row arrays of x,y,z state position
		follower_path.append(follower.path) # List of numpy 3-element 1-row arrays of x,y,z state position
		leader_target.append(leader.target) # Numpy 3-element 1-row array of x,y,z state position
		follower_target.append(follower.target) # Numpy 3-element 1-row array of x,y,z state position


	#while len(leader.path) > 0:
	#   leader.step_forward()
	blue = plt.scatter([point[0] for point in leader.points],[point[1] for point in leader.points], s=1, color="blue")
	orange = plt.scatter([point[0] for point in follower.points],[point[1] for point in follower.points], s=1, color="orange")
	plt.scatter(obstacles[:, 0], obstacles[:, 1], s=1, color="red")
	plt.scatter([leader.start[0], leader.goal[0]], [leader.start[1], leader.goal[1]], color="brown")
	plt.scatter([follower.start[0], follower.goal[0]], [follower.start[1], follower.goal[1]], color="brown")
	plt.scatter([state[0] for state in states], [state[1] for state in states], color = "lime", s=10)
	plt.show()

	plt.plot([point[2] for point in leader.points])
	plt.plot([point[2] for point in follower.points])
	plt.show()


	import matplotlib.pyplot as plt
	import matplotlib.animation as animation
	import numpy as np
	leader_pos = np.array(leader_pos)
	follower_pos = np.array(follower_pos)
	leader_path = np.array(leader_path)
	follower_path = np.array(follower_path)
	leader_target = np.array(leader_target)
	follower_target = np.array(follower_target)
	fig, ax = plt.subplots()
	plt.title('The Leader-Follower Scenario')
	plt.xlabel('North (m)')
	plt.ylabel('East (m)')

	# Initial scatter objects
	obstacles_plot = plt.scatter(obstacles[:, 0], obstacles[:, 1], s=3, color="red")
	states_plot = plt.scatter(states[:, 0], states[:, 1], s=1, color="green",)
	leader_pos_plot = plt.scatter([], [], color="blue", label="Leader", linewidth=1)
	follower_pos_plot = plt.scatter([], [], color="magenta", label="Follower", linewidth=1)
	leader_path_plot, = plt.plot([], [], color="blue")
	follower_path_plot, = plt.plot([], [], color="magenta")
	leader_target_plot = plt.scatter([], [], s=15, color="blue", marker='*')
	follower_target_plot = plt.scatter([], [], s=15, color="magenta", marker='*')
	plt.legend()
	def update(num):
		print("Leader height", leader_pos[num][2])
		print("Follower height", follower_pos[num][2])

		# Check if the points exist for this frame
		if num < len(leader_pos):
			# Update the positions
			leader_pos_plot.set_offsets(leader_pos[num][:2])
		if num < len(follower_pos):
			follower_pos_plot.set_offsets(follower_pos[num][:2])
		if num < len(leader_path) and len(leader_path[num]) > 0:
			# Convert list of points to numpy array before updating the paths
			points = np.array(leader_path[num])
			leader_path_plot.set_data(points[:, 0], points[:, 1])
		if num < len(follower_path) and len(follower_path[num]) > 0:
			points = np.array(follower_path[num])
			follower_path_plot.set_data(points[:, 0], points[:, 1])
		if num < len(leader_target):
			# Update the targets
			leader_target_plot.set_offsets(leader_target[num][:2])
		if num < len(follower_target):
			follower_target_plot.set_offsets(follower_target[num][:2])

	ani = animation.FuncAnimation(fig, update, frames=range(1200), interval=30)


	plt.show()
	pdb.set_trace()
	"""
	conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
	drone = PathPlanner(conn, leader, states, states_kd, obstacles, obstacles_kd)
	time.sleep(2)
	drone.start()