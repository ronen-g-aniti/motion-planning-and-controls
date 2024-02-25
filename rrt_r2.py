import numpy as np
from reference_frame import global_to_local
from collision_check import collision_check_vectorized, inside_environment
import pdb

# TIME INTERGVAL IS 10; GOAL BIAS IS 0.5

class RRT:
	def __init__(self, environment_data_object, start_gps, goal_gps, GOAL_BIAS=0.5, MAX_STEER_ANGLE_RATE=np.pi/24, TIME_STEP=0.1, 
	  TIME_INTERVAL=5.0, SPEED=2.0, MAX_ITERATIONS=10000, GOAL_TOLERANCE=1.0, STEERING=True):
		self._environment = environment_data_object
		self._start_gps = start_gps
		self._goal_gps = goal_gps
		self._GOAL_BIAS = GOAL_BIAS
		self._MAX_STEER_ANGLE = MAX_STEER_ANGLE_RATE * TIME_STEP # Radians per iteration
		self._TIME_STEP = TIME_STEP
		self._SPEED = SPEED
		self._MAX_ITERATIONS = MAX_ITERATIONS
		self._GOAL_TOLERANCE = GOAL_TOLERANCE
		self._TIME_INTERVAL = TIME_INTERVAL
		self._INTEGRATION_STEPS = int(self._TIME_INTERVAL / self._TIME_STEP) # Iterations of each branch expansion process

		self._goal_is_found = False

		# Convert GPS positions to NED coordinates
		self._start_pos = global_to_local(start_gps, environment_data_object.gps_home)
		self._goal_pos = global_to_local(goal_gps, environment_data_object.gps_home)

		# Check is positions are in free space
		if collision_check_vectorized(self._environment, self._start_pos):
			raise ValueError("Start position is not in free space")
		if collision_check_vectorized(self._environment, self._goal_pos):
			raise ValueError("Goal position is not in free space")
		if not inside_environment(self._environment, self._start_pos):
			raise ValueError("Start position is outside of the bounds of the environment")
		if not inside_environment(self._environment, self._goal_pos):
			raise ValueError("Goal position is outside of the bounds of the environment")

		# Define a starting orientation
		# Point towards the goal state, initially
		delta = self._goal_pos - self._start_pos
		start_orientation = delta / np.linalg.norm(delta)

		# Define a start state (position and orientation)
		start_state = np.concatenate((self._start_pos, start_orientation))

		# Make the start state the root of a tree
		self._states = np.array([start_state])
		self._edges = {0: None}

		# Toggle off steering
		self.STEERING = STEERING

	def run(self):

		for _ in range(self._MAX_ITERATIONS):
			p_sample = self.sample_with_bias()
			x_near, index_near = self.find_nearest_state(p_sample)
			x_new = self.integrate_forward(x_near, p_sample)
			if self._goal_is_found:
				self._states = np.vstack([self._states, x_new])
				index_of_new = len(self._states) - 1
				self._edges[index_of_new] = index_near 
				break
			self._states = np.vstack([self._states, x_new])
			index_of_new = len(self._states) - 1 
			self._edges[index_of_new] = index_near

		if self._goal_is_found:
			path = [index_of_new]
			parent = self._edges[index_of_new]
			while parent != 0:
				path.append(parent)
				parent = self._edges[parent]
			path.append(parent)
			path = path[::-1]
			path_as_states=[self._states[path_index] for path_index in path]
			return path_as_states
		else:
			print("Path not found")

	def sample_with_bias(self):
		random_number = np.random.rand()
		if random_number < self._GOAL_BIAS:
			p_sample = self._goal_pos
		else:
			p_sample = np.array([np.random.uniform(self._environment.xbounds[0], self._environment.xbounds[1]),
							 np.random.uniform(self._environment.ybounds[0], self._environment.ybounds[1]),
							 np.random.uniform(self._environment.zbounds[0], self._environment.zbounds[1])])
		return p_sample

	def find_nearest_state(self, query_pos):
		positions = self._states[:, :3]
		distances = np.sum((positions - query_pos)**2, axis=1)
		min_index = np.argmin(distances)
		x_near = self._states[min_index]
		return x_near, min_index

	def integrate_forward(self, x_near, p_sample):
		x_i = x_near
		if not self.STEERING:
			return x_i + 5.0 * (p_sample-x_near[:3]) / np.linalg.norm(p_sample-x_near[:3])
		for _ in range(self._INTEGRATION_STEPS):
			x_prior = x_i.copy()
			x_i = self.update_state(x_i, p_sample)
			if np.linalg.norm(x_i[:3] - self._goal_pos) <= self._GOAL_TOLERANCE:
				self._goal_is_found = True
				print("Path found")
				break
			if collision_check_vectorized(self._environment, x_i[:3]):
				return x_prior
			if not inside_environment(self._environment, x_i[:3]):
				return x_prior
		return x_i



	def update_state(self, x_near, p_sample):
		prior_attitude = x_near[3:]
		prior_position = x_near[:3]
		u_1 = (p_sample - prior_position) / np.linalg.norm(p_sample - prior_position)
		axis_vector = np.cross(prior_attitude, u_1)
		if np.linalg.norm(axis_vector) <= 0.01:
			R = np.eye(3)        
		else:
			u_2 = axis_vector / np.linalg.norm(axis_vector)
			theta = np.arccos((np.dot(prior_attitude, u_1))/(np.linalg.norm(u_1)*np.linalg.norm(prior_attitude)))
			steer_angle = np.clip(theta, -self._MAX_STEER_ANGLE, self._MAX_STEER_ANGLE)
			K = np.array([[0, -u_2[2], u_2[1]],[u_2[2], 0, -u_2[0]],[-u_2[1],u_2[0],0]])
			R = np.eye(3) + np.sin(steer_angle) * K + (1 - np.cos(steer_angle)) * np.dot(K, K)
		new_attitude = np.dot(R, prior_attitude)
		new_position = prior_position + new_attitude * self._SPEED * self._TIME_STEP
		x_new = np.concatenate((new_position, new_attitude))
		return x_new

	def update_state_no_steer(self, x_near, p_sample):
		pass

