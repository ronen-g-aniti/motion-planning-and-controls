from matplotlib.patches import Circle
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
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
		plt.scatter(obstacles_x, obstacles_y, color="red", s=1, alpha=0.5, label="Obstacle")
		# Add a circle around each obstacle
		for obs in self.obstacles:
			proximity_circle = patches.Circle(obs.xy_pos, obs.detection_radius, edgecolor=(0, 0, 0.7), facecolor='none', alpha=1, linewidth=1)
			plt.gca().add_patch(proximity_circle)
			collision_circle = patches.Circle(obs.xy_pos, obs.collision_radius, edgecolor='red', facecolor='none', alpha=1, linewidth=1)
			plt.gca().add_patch(collision_circle)
		plt.xlim(self.x_bound[0], self.x_bound[1])
		plt.ylim(self.y_bound[0], self.y_bound[1])
		plt.title("Interceptor with obstacle avoidance capability")
		plt.xlabel("X Position")
		plt.ylabel("Y Position")

class Obstacle:
	def __init__(self, xy_pos: np.ndarray):
		self.xy_pos = xy_pos 
		self.collision_radius = 10
		self.detection_radius = 50

class Target: 
	def __init__(self, xy_pos: np.ndarray, velocity_magnitude: float, unit_vector: np.ndarray):
		self.xy_pos = [xy_pos]
		self.velocity_magnitude = velocity_magnitude
		self.unit_vector = unit_vector
		self.time_step = 0.1
		self.timer = [0]

	def compute_new_unit_vector(self):
		delta_direction = np.array([np.random.uniform(low=0, high=0.2), 0.1])
		unit_vector_plus_delta = self.unit_vector + delta_direction
		self.unit_vector = unit_vector_plus_delta/np.linalg.norm(unit_vector_plus_delta)
		return self.unit_vector

	def return_a_new_pos(self):
		"""Simulate a new position. Call repeatedly to generate a trajectory."""
		self.xy_pos.append(self.xy_pos[-1] + self.time_step * self.velocity_magnitude * self.compute_new_unit_vector())
		self.timer.append(self.timer[-1] + self.time_step)

	def visualize(self):
		plt.scatter([xy_pos[0] for xy_pos in self.xy_pos], [xy_pos[1] for xy_pos in self.xy_pos], color=(1,0,1), s=1, label="Target")

class Interceptor: 
	def __init__(self, xy_pos: np.ndarray, xy_velocity: np.ndarray, environment: Environment, target: Target):
		self.environment = environment
		self.target = target
		self.xy_pos = [xy_pos]
		self.xy_velocity = [xy_velocity]
		self.time_step = 0.1
		self.timer = [0]
		self.nearest_obs = None
		self.max_steering_angle = np.pi/6 # rad
	
	def determine_pos_of_nearest_obs(self) -> np.ndarray:
		obs_positions = np.array([obs.xy_pos for obs in self.environment.obstacles])
		current_pos = self.xy_pos[-1]
		distances = np.linalg.norm(obs_positions - current_pos, axis=1)
		nearest_obs_index = np.argmin(distances)
		self.nearest_obs = self.environment.obstacles[nearest_obs_index]
		nearest_obs_pos = self.environment.obstacles[nearest_obs_index].xy_pos
		if np.linalg.norm(self.xy_pos[-1] - nearest_obs_pos) < self.environment.obstacles[0].collision_radius:
			print("The mission failed because the interceptor collided with an obstacle.")
			print("The interceptor approached " + str(np.linalg.norm(self.xy_pos[-1] - nearest_obs_pos)) + " of a point obstacle, thus failing to avoid a collision.")
		return nearest_obs_pos

	def compute_vector_from_nearest_obs(self, nearest_obs_pos: np.ndarray) -> np.ndarray:
		vector_from_nearest_obs = self.xy_pos[-1] - nearest_obs_pos
		return vector_from_nearest_obs

	def determine_repulsive_velocity(self, vector_from_nearest_obs: np.ndarray) -> np.ndarray:
		distance_to_nearest_obs = np.linalg.norm(vector_from_nearest_obs)
		repulsive_gain = 0.0
		if distance_to_nearest_obs < self.nearest_obs.detection_radius:
			# The repulsive gain is proportional to how close the interceptor is to the collision radius of the obstacle.
			repulsive_gain = 50.0 * (1- ((distance_to_nearest_obs - self.nearest_obs.collision_radius) / (self.nearest_obs.detection_radius - self.nearest_obs.collision_radius)) )
			print("Applying obstacle avoidance")
		
		direction = vector_from_nearest_obs / distance_to_nearest_obs
		repulsive_velocity = repulsive_gain * direction
		return repulsive_velocity

	def determine_vector_to_target(self) -> np.ndarray:
		vector_to_target = target.xy_pos[-1] - self.xy_pos[-1]
		return vector_to_target

	def determine_attractive_velocity(self, vector_to_target: np.ndarray) -> np.ndarray:
		distance_to_target = np.linalg.norm(vector_to_target)
		attractive_gain = 50.0 
		direction = vector_to_target / distance_to_target
		attractive_velocity = attractive_gain * direction
		return attractive_velocity

	def determine_command_velocity(self, repulsive_velocity: np.ndarray, attractive_velocity: np.ndarray) -> np.ndarray:
		command_velocity = attractive_velocity + repulsive_velocity
		if len(self.xy_pos) > 1:
			if abs(np.arctan2(command_velocity[1], command_velocity[0]) - np.arctan2(self.xy_velocity[-1][1], self.xy_velocity[-1][0])) > self.max_steering_angle:
				print("The mission failed because the intercepter commanded a steering angle greater than the maximum allowable.")
				print(self.xy_pos[-1]) 
		self.xy_velocity.append(command_velocity)
		return command_velocity

	def determine_new_pos(self, command_velocity: np.ndarray) -> np.ndarray:
		new_pos = self.xy_pos[-1] + command_velocity * self.time_step
		self.xy_pos.append(new_pos)
		self.timer.append(self.timer[-1] + self.time_step)
		return new_pos

	def advance_pos(self):
		nearest_obs_pos = self.determine_pos_of_nearest_obs()
		vector_from_nearest_obs = self.compute_vector_from_nearest_obs(nearest_obs_pos)
		repulsive_velocity = self.determine_repulsive_velocity(vector_from_nearest_obs)
		vector_to_target = self.determine_vector_to_target()
		attractive_velocity = self.determine_attractive_velocity(vector_to_target)
		command_velocity = self.determine_command_velocity(repulsive_velocity, attractive_velocity)
		new_pos = self.determine_new_pos(command_velocity)

	def visualize(self):
		#plt.quiver([xy_pos[0] for xy_pos in self.xy_pos], [xy_pos[1] for xy_pos in self.xy_pos], [xy_velocity[0] for xy_velocity in self.xy_velocity], [xy_velocity[1] for xy_velocity in self.xy_velocity], width=0.002)
		plt.scatter([xy_pos[0] for xy_pos in self.xy_pos], [xy_pos[1] for xy_pos in self.xy_pos], color=(0,0,1), marker="o", s=1)

environment = Environment(np.array([0, 2000]), np.array([0, 2000]), 85)
target = Target(np.array([250,900]), 25, np.array([np.cos(np.pi/3), np.sin(np.pi/3)]))
interceptor = Interceptor(np.array([100, 0]), np.array([0, 0]), environment, target)
follower = Interceptor(np.array([500, 500]), np.array([0, 0]), environment, interceptor)

timesteps = 1000
for time_step in range(timesteps):
	target.return_a_new_pos()
	interceptor.advance_pos()
	follower.advance_pos()
	if np.linalg.norm(target.xy_pos[-1] - interceptor.xy_pos[-1]) < 5:
		print("Target intercepted")
		break

environment.visualize()
target.visualize()
interceptor.visualize()
follower.visualize() ##
plt.legend()
plt.savefig('trajectories.png')

plt.show()

import matplotlib.animation as animation


# Create the figure and axes
fig, ax = plt.subplots()

# Initialize scatter plots for target and interceptor
skip_count = 10
target_scatter = ax.scatter([xy_pos[0] for xy_pos in target.xy_pos], [xy_pos[1] for xy_pos in target.xy_pos], color=(1, 0, 1), s=15, marker="o", label="Target")
interceptor_scatter = ax.scatter([xy_pos[0] for xy_pos in interceptor.xy_pos], [xy_pos[1] for xy_pos in interceptor.xy_pos], color=(0, 0, 1), marker="*", s=15, label="Interceptor")
follower_scatter = ax.scatter([xy_pos[0] for xy_pos in follower.xy_pos], [xy_pos[1] for xy_pos in follower.xy_pos], color=(0, 1, 1), marker="*", s=15, label="Follower") ##
environment.visualize()
plt.title("Interceptor with obstacle avoidance capability")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.legend()
# Function to update the scatter plots for each frame of the animation
def update(frame):
	target_scatter.set_offsets([target.xy_pos[frame * skip_count]])
	interceptor_scatter.set_offsets([interceptor.xy_pos[frame * skip_count]])
	follower_scatter.set_offsets([follower.xy_pos[frame * skip_count]]) ##
	return target_scatter, interceptor_scatter, follower_scatter

# Function to initialize the animation
def init():
	target_scatter.set_offsets([])
	interceptor_scatter.set_offsets([])
	follower_scatter.set_offsets([])
	return target_scatter, interceptor_scatter, follower_scatter

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=len(target.xy_pos) // skip_count,
							  init_func=init, blit=True)

# Set up formatting for the movie files
plt.rcParams['animation.convert_path'] = "C:\\Program Files\\ImageMagick-7.1.1-Q16-HDRI\\magick.exe"
Writer = animation.writers['imagemagick']
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

# Save the animation
ani.save('animation.gif', writer=writer)


# Display the animation
plt.show()