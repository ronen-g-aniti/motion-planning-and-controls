import numpy as np
from reference_frame import global_to_local
from collision_check import collision_check_vectorized, inside_environment
from environment_data import EnvironmentData
from rrt_r2 import RRT
import pdb

class RecedingHorizonPlanner:
	def __init__(self, environment_data_object, ATTRACTIVE_GAIN=1.0, REPULSIVE_GAIN=1.0, HORIZON_RADIUS=10.0, QUERY_RADIUS=10.0, TIMESTEP=0.10, SECONDS_AHEAD_START=1.0):
		self._environment = environment_data_object
		self._ATTRACTIVE_GAIN = ATTRACTIVE_GAIN
		self._REPULSIVE_GAIN = REPULSIVE_GAIN
		self._HORIZON_RADIUS = HORIZON_RADIUS
		self._QUERY_RADIUS = QUERY_RADIUS
		self._TIMESTEP = TIMESTEP
		self._SECONDS_AHEAD_START = SECONDS_AHEAD_START

	def integrate_from_this_position(self, current_local_position, current_local_velocity, active_waypoint_position, integration_period):
		# 0. Start the integration process from a location ahead of the current_local_position that is scaled by the current velocity by N seconds of scaling factor
		# While the distance between x_i and current_local_position is less than the horizon radius, do the following steps:
		# 1. Return the positions of all obstacles in the horizon
		# 2. For each obstacle returned do the following:
		# 2.1  Compute o_i, the push force assigned to that obstacle
		# 3. Add up all o_i to make a resultant obstacle push vector called o_push
		# 4. Compute a_pull, the attractive vector
		# 5. Combine o_push and a_pull according to a mathematical rule that I specify later
		# 6. Assign the magnitude of the resulting vector to "speed"
		# 7. Assign the direction of the resulting vector to "direction"
		# 8. Advance the state according to a mathematical rule that adds to the current position speed * TIMESTEP * direction.
		## Note to self: What I've created is essentially an "artificial velocity field"

		current_speed = np.linalg.norm(current_local_velocity)
		direction_of_velocity = current_local_velocity / current_speed
		start_pos = current_local_position + self._SECONDS_AHEAD_START * direction_of_velocity * current_speed
		x_i = start_pos
		intermediate_path = [x_i]
		t = 0.0
		while np.linalg.norm(x_i - start_pos) < self._HORIZON_RADIUS and t < integration_period:
			obstacle_positions = obstacle_position_finder(x_i)
			push_forces = []
			for obstacle_position in obstacle_positions:
				push_forces.append(obstacle_force_calculator(obstacle_position, x_i))
			combined_push_force = combine_push_forces(push_forces)
			pull_force = attractive_force_calculator(active_waypoint_position, x_i)
			combined_force = combine_push_and_pull(combined_push_force, pull_force)
			x_prior = x_i.copy()
			x_i = update_state(x_i, combined_force)
			if collision_check_vectorized(self._environment, x_i):
				break
			if not inside_environment(self._environment, x_i):
				break
			intermediate_path.append(x_i)
			t += current_speed * self._TIMESTEP
		return intermediate_path[-1]

	def update_state(self, x_i, combined_force):

		# x_i = x_i + combined_force * timestep

		return x_new

	def combine_push_and_pull(self, combined_push_force, pull_force):

		# Add the push vector to the pull vector and return that new vector

		return combined_force

	def combine_push_forces(self, push_forces):

		# Add the push vectors
		# Return the result vector

		return combined_push_force

	def obstacle_force_calculator(self, obstacle_center_position, simulated_position):
		
		# 1. Compute the distance to the given obstacle
		# 2. Compute the unit vector to the given obstacle
		# 3. Scale the unit vector by the inverse of the squared distance, multiplied by the repulsive gain
		# 4. Return the scaled vector


		return obstacle_force_i

	def attractive_force_calculator(self, active_waypoint_position, simulated_position):

		# 1. The unit vector from x_i to the active waypoint position
		# 2. The distance from x_i to the active waypoint position
		# 3. Scale this unit vector by the attractive gain
		# 4. Return the scaled attractive vector


		return attractive_force

	def obstacle_position_finder(self, query_position):
		# Get indices of all of the obstacles that meet these criteria:
		# 1. They have a ground position within the querry radius of x_i
		# 2. They have a height at or above x_i

		# For each of those obstacles, record each of these criteria:
		# 1. The ground distance from x_i to the obstacle's boundary (center minus a lateral safety distance)
		# 2. The unit vector between x_i and the obstacle's center position, adjusted for the height of x_i

		# Return a list organized as follows:
		# [(unit_vector, ground_distance)_i for each obstacle in those obstacle that meet the criteria]

		return list_of_obstacle_positions_adjusted_for_x_i_altitude



class DroneDummy:

	def __init__(self, mapdata, rhp, x_start, v_start, path_to_follow, deadband_radius=5.0, update_frequency=100):
		self.mapdata = mapdata
		self.rhp = rhp
		self.x_start = x_start
		self.x = x_start
		self.v_start = v_start
		self.v = v_start
		self.path_to_follow = [np.array([point[0], point[1], point[2]]) for point in path_to_follow] # Assuming path_to_follow includes orientation
		self.deadband_radius = deadband_radius
		self.update_frequency = update_frequency
		self.integration_period = self.v / self.update_frequency
		self.active_waypoint = self.path_to_follow[1] # The 0th point is the current position -> so the target should be the located at index 1.

	def step_forward(self):
		self.x = self.rhp.integrate_from_this_position(self.x, self.v, self.active_waypoint, self.integration_period)

		if np.linalg.norm(self.x) <= self.deadband_radius:
			self.active_waypoint = self.path_to_follow.pop(0)

	def update_plot_with_new_x(self):
		pass

	def create_base_plot(self):
		pass

# Create a map
mapping_data = EnvironmentData('colliders.csv', 5.0)

# Get the RHP up and running
rhp = RecedingHorizonPlanner(mapping_data)

# Create some path using RRT
#lon, lat, alt
start_gps = [-122.39745, 37.79248, 0]
goal_gps = [-122.39645,  37.79248, 200]
rrt = RRT(mapping_data, start_gps, goal_gps, GOAL_BIAS=0.8)
path = rrt.run()

start_position = path[0]
start_velocity = np.array([1,1,1]) 
drone = DroneDummy(mapping_data, rhp, start_position, start_velocity, path)
pdb.set_trace()
goal_tolerance = 1.0
drone.create_base_plot()
while np.linalg.norm(drone.x - path[-1]) > goal_tolerance:
	drone.step_forward()
	drone.update_plot_with_new_x()






# If the algorithm is to have a chance of being useful in the simulation environment, then it must
# be able to perform the integration / path calculation quickly enough such that the commanded intermediate path
# and particularly the next commanded waypoint is located at a position that makes sense given the drone's position 
# after the calculation has been computed. Or I could work around this problem by simulating the drone's starting position to be
# X meters ahead of its current velocity vector's direction, where X is scaled according to the magnitude of its velocity. 

# Overall, working on the receding horizon planner broadened by understanding of motion planning by inspiring me to think about
# how to balance the tradeoff between resolution and speed and also deal with updates to the map in real time.


# From current GPS location return local position in NED coordinates relative to global home

# Enter the "Integration module"

# Find the obstacles that are within a bounding circle (horizon) of the drone, given a radius and the environment mapping data object

# Set the drone's current location to x_i

# For each of those obstacles, construct a force vector that pushes the drone in the direction from the obstacle's center to x_i

# Construct a force vector that pulls the drone from location x_i to the location of the next waypoint p_active

# Combine the vectors according to a mathematical rule I design

# Check if the vector sum has a magnitude of zero. If it does, that implies that the drone will have trouble advancing to the next step. Proceed anyway. Realize
# also that with a cluttered environment, it would be reasonable to expect the drone to have trouble advancing towards some waypoints. Furthermore, it would be 
# reasonable to expect that the drone follow paths that are disjointed since the gradient of the vector field is generally not smooth. (I think). This could
# manifest as oscilitory motion when a path is drawn out.

# Set speed to be the magnitude of the combination of the force vectors. Set direction to be the direction of the resulting force vector.

# Integrate forward by multiplying the speed by dt, then multiplying that by the direction vector. Then adding that result to the prior x_i. 

# Continue this process until the integrated state x_i has reached the radius distance from the drone (the edge of the receding horizon)

# On each iteration, append x_i to path.

# Return Path

# Exit the integration module

# Therefore, the path represents a safe-to-fly route that brings the drone from its current location towards its next waypoint. It can be used to complement
# a global planning scheme by enabling planning at higher resolutions and in such a way that is responsive to changes in the environment.