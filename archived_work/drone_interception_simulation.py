import numpy as np
import matplotlib.pyplot as plt
import pdb

class Environment:
	def __init__(self, x_bound: np.ndarray, y_bound: np.ndarray, num_obs: int):
		self.x_bound = x_bound
		self.y_bound = y_bound 
		self.num_obs = num_obs
		self.obstacles = [Obstacle(np.array([np.random.uniform(low=self.x_bound[0]+100, high=self.x_bound[1]-100), np.random.uniform(low=self.x_bound[0]+100, high=self.x_bound[1]-100)])) for _ in range(self.num_obs)]

	def visualize(self):
		obstacles_x = [obs.xy_pos[0] for obs in self.obstacles]
		obstacles_y = [obs.xy_pos[1] for obs in self.obstacles]
		plt.scatter(obstacles_x, obstacles_y)
		plt.xlim(self.x_bound[0], self.x_bound[1])
		plt.ylim(self.y_bound[0], self.y_bound[1])
		

class Obstacle:
	def __init__(self, xy_pos: np.ndarray):
		self.xy_pos = xy_pos 
		self.collision_radius = 2.5
		self.detection_radius = 100

class Target: 
	def __init__(self, xy_pos: np.ndarray, velocity_magnitude: float, unit_vector: np.ndarray):
		self.xy_pos = [xy_pos]
		self.velocity_magnitude = velocity_magnitude
		self.unit_vector = unit_vector
		self.time_step = 0.1
		self.timer = [0]

	def compute_new_unit_vector(self):
		delta_direction = np.array([np.random.uniform(low=0, high=0.1), np.random.uniform(low=-0.1, high=0.1)])
		unit_vector_plus_delta = self.unit_vector + delta_direction
		self.unit_vector = unit_vector_plus_delta/np.linalg.norm(unit_vector_plus_delta)
		return self.unit_vector

	def return_a_new_pos(self):
		"""Simulate a new position. Call repeatedly to generate a trajectory."""
		self.xy_pos.append(self.xy_pos[-1] + self.time_step * self.velocity_magnitude * self.compute_new_unit_vector())
		self.timer.append(self.timer[-1] + self.time_step)

	def visualize(self):
		plt.plot([xy_pos[0] for xy_pos in self.xy_pos], [xy_pos[1] for xy_pos in self.xy_pos], color="orange")


class Interceptor: 
	def __init__(self, xy_pos: np.ndarray, xy_velocity: np.ndarray, environment: Environment, target: Target):
		self.environment = environment
		self.target = target
		self.xy_pos = [xy_pos]
		self.xy_velocity = [xy_velocity]
		self.xy_acceleration = []
		self.time_step = 0.1
		self.timer = [0]
		self.mass = 200.0 # kg
		self.nearest_obs = None
	
	def determine_pos_of_nearest_obs(self) -> np.ndarray:
		obs_positions = np.array([obs.xy_pos for obs in self.environment.obstacles])
		current_pos = self.xy_pos[-1]
		distances = np.linalg.norm(obs_positions - current_pos, axis=1)
		nearest_obs_index = np.argmin(distances)
		self.nearest_obs = self.environment.obstacles[nearest_obs_index]
		nearest_obs_pos = self.environment.obstacles[nearest_obs_index].xy_pos
		return nearest_obs_pos

	def determine_vector_from_nearest_obs(self, nearest_obs_pos: np.ndarray) -> np.ndarray:
		vector_from_nearest_obs = self.xy_pos[-1] - nearest_obs_pos
		return vector_from_nearest_obs

	def determine_repulsive_force(self, vector_from_nearest_obs: np.ndarray) -> np.ndarray:
		distance_to_nearest_obs = np.linalg.norm(vector_from_nearest_obs)
		repulsive_gain = 0.0
		if distance_to_nearest_obs < self.nearest_obs.detection_radius:
			repulsive_gain = 2500.0
			print("Applying obstacle avoidance")
		
		direction = vector_from_nearest_obs / distance_to_nearest_obs
		repulsive_force = repulsive_gain * direction
		return repulsive_force 

	def determine_vector_to_target(self) -> np.ndarray:
		vector_to_target = target.xy_pos[-1] - self.xy_pos[-1]
		print("target xy ", target.xy_pos[-1])
		print("interceptor xy ", self.xy_pos[-1])
		print("vector to target ", vector_to_target)
		return vector_to_target

	def determine_attractive_force(self, vector_to_target: np.ndarray) -> np.ndarray:
		distance_to_target = np.linalg.norm(vector_to_target)
		attractive_gain = 1000000
		direction = vector_to_target / distance_to_target
		attractive_force = attractive_gain * direction
		return attractive_force

	def determine_net_force(self, repulsive_force: np.ndarray, attractive_force: np.ndarray) -> np.ndarray:
		net_force = attractive_force + repulsive_force
		return net_force
		
	def determine_acceleration_vector(self, net_force: np.ndarray) -> np.ndarray:
		acceleration_vector = net_force / self.mass
		self.xy_acceleration.append(acceleration_vector)
		return acceleration_vector

	def determine_velocity_vector(self, acceleration_vector: np.ndarray) -> np.ndarray:
		velocity_vector = self.xy_velocity[-1] + self.xy_acceleration[-1] * self.time_step
		self.xy_velocity.append(velocity_vector)
		return velocity_vector 

	def determine_pos_vector(self, velocity_vector: np.ndarray, acceleration_vector: np.ndarray) -> np.ndarray:
		pos_vector = self.xy_pos[-1] + self.xy_velocity[-1] * self.time_step + 1/2 * self.xy_acceleration[-1] * self.time_step**2
		self.xy_pos.append(pos_vector)
		self.timer.append(self.timer[-1] + self.time_step)
		return pos_vector

	def advance_pos(self):
		nearest_obs_pos = self.determine_pos_of_nearest_obs()
		vector_from_nearest_obs = self.determine_vector_from_nearest_obs(nearest_obs_pos)
		repulsive_force = self.determine_repulsive_force(vector_from_nearest_obs)
		vector_to_target = self.determine_vector_to_target()
		attractive_force = self.determine_attractive_force(vector_to_target)
		net_force = self.determine_net_force(repulsive_force, attractive_force)
		acceleration_vector = self.determine_acceleration_vector(net_force)
		velocity_vector = self.determine_velocity_vector(acceleration_vector)
		position_vector = self.determine_pos_vector(velocity_vector, acceleration_vector)
		return position_vector

	def visualize(self):
		#plt.quiver([xy_pos[0] for xy_pos in self.xy_pos], [xy_pos[1] for xy_pos in self.xy_pos], [xy_accel[0] for xy_accel in self.xy_acceleration], [xy_accel[1] for xy_accel in self.xy_acceleration])
		plt.scatter([xy_pos[0] for xy_pos in self.xy_pos], [xy_pos[1] for xy_pos in self.xy_pos])
environment = Environment(np.array([0, 1000]), np.array([0, 1000]), 20)
target = Target(np.array([100,900]), 50, np.array([1,0]))
interceptor = Interceptor(np.array([0, 0]), np.array([0, 0]), environment, target)
#follower = Interceptor(np.array([100, 100], np.array([0,0]), environment, interceptor))


timesteps = 20
for time_step in range(timesteps):
	target.return_a_new_pos()
	interceptor.advance_pos()
	#follower.advance_pos()
	print((time_step, target.xy_pos[-1], interceptor.xy_pos[-1], np.linalg.norm( target.xy_pos[-1] - interceptor.xy_pos[-1])))
environment.visualize()
target.visualize()
interceptor.visualize()
plt.show()
pdb.set_trace()

