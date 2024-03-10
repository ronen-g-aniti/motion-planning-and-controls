import numpy as np
import matplotlib.pyplot as plt
import pdb
import matplotlib.animation as animation

class RRT:
	def __init__(self, start, obstacles):
		"""Initializes the tree"""
		self.start = start
		self.goal = None
		self.explored = np.array([start])
		self.edges = {}		
		self.obstacle_centers = obstacles[:, :3]
		self.obstacle_halfsizes = obstacles[:, 3:]
		self.xbounds, self.ybounds, self.zbounds = self.bounds()
		self.lines = []



	def bounds(self):
		obstacle_centers = self.obstacle_centers
		obstacle_halfsizes = self.obstacle_halfsizes 
		xmin = np.min(obstacle_centers[:,0] - obstacle_halfsizes[:,0])
		xmax = np.max(obstacle_centers[:,0] + obstacle_halfsizes[:,0])
		ymin = np.min(obstacle_centers[:,1] - obstacle_halfsizes[:,1])
		ymax = np.max(obstacle_centers[:,1] + obstacle_halfsizes[:,1])
		zmin = np.min(obstacle_centers[:,2] - obstacle_halfsizes[:,2])
		zmax = np.max(obstacle_centers[:,2] + obstacle_halfsizes[:,2])
		xbounds = (xmin, xmax)
		ybounds = (ymin, ymax)
		zbounds = (zmin, zmax)
		return xbounds, ybounds, zbounds

	def run(self, goal, step=15.0, max_iters=100000):
		"""Runs the main algorithm. This is a form of the rapidly-exploring random tree algorithm"""
		
		# Set the goal state
		self.goal = goal

		goal_found = False

		# Set the current state to be the start state
		current = self.start

		# Loop until the goal is reached
		iteration = 0
		while not self.goal_reached(current) and iteration < max_iters:
			iteration += 1
			s_sample = self.sample()
			if iteration % 20 == 0: 
				s_sample = self.goal

			# Returns the nearest tree node `s_nearest` to `s_sample`
			s_nearest_index = np.argmin(np.linalg.norm(s_sample[:3] - self.explored[:, :3], axis=1))
			s_nearest = self.explored[s_nearest_index]

			# Compute s_new, a state that is `step` distance away from s_nearest in the direction `u_hat` of s_sample
			u_hat = (s_sample - s_nearest) / np.linalg.norm(s_sample - s_nearest)
			s_new = s_nearest + u_hat * step
			
			# Add a branch of maximum length possible
			#for substep in np.linspace(step, 1.0, num=10):
			#	s_new = s_nearest + u_hat * substep
			#	if not self.collides(s_nearest, s_new):
			#		break

			# Add s_new to the tree only if the segment connecting it to s_nearest doesn't collide with any obstacles
			if not self.collides(s_nearest, s_new):
				self.explored = np.vstack((self.explored, s_new))
				# Add a key-value pair to `edges` to represent the connection between s_new and s_nearest
				s_new_index = len(self.explored) - 1
				self.edges[s_new_index] = s_nearest_index


			# Set the current node to be s_new
			current = s_new

			# Break the loop if the current node is close enough to the goal state
			if self.goal_reached(current):
				print("Goal is reached")
				goal_found = True
				break
				
		# Reconstruct and shorten the path if the goal is reached, then return it.
		if goal_found:
			path = self.reconstruct()
			# Appends the actual goal state to path if doing so doesn't register a collision 
			if not self.collides(current, self.goal):
				path = np.vstack((path, self.goal))
			path = self.shorten(path)
			print(path)
		
		# Call the animate method at the end of the run method
		self.animate(path)
		
		return path

	def sample(self):
		"""Samples a random state from within the bounds of the obstacle space"""
		x = np.random.uniform(low=self.xbounds[0], high=self.xbounds[1], size=1)[0]
		y = np.random.uniform(low=self.ybounds[0], high=self.ybounds[1], size=1)[0]
		# Restrict z to goal altitude
		z = self.goal[2]
		#z = np.random.uniform(low=self.zbounds[0], high=self.zbounds[1], size=1)[0]
		return np.array([x, y, z])

	def collides(self, start_point, end_point):
		"""
		Checks if the bounding box of the line segment connecting the start point and the end point intersects the bounding 
		box of any obstacle from the obstacle space. This collision checking routine is known as the "Axis-aligned Bounding Box
		Algorithm" and serves the purpose in this code as a simple way of checking whether or not a sampled point should be added 
		to the tree. 
		"""
		obstacle_centers = self.obstacle_centers
		obstacle_halfsizes = self.obstacle_halfsizes
		# Defines the maxima and minima x, y, z positions of the bounding box around the line segment
		minima = np.minimum(start_point, end_point)
		maxima = np.maximum(start_point, end_point)

		# Checks whether or not the bounding box around each obstacle collides with the bounding box around the line segment. 
		collision_mask = np.all((obstacle_centers - obstacle_halfsizes) <= maxima, axis=1) & np.all((obstacle_centers + obstacle_halfsizes) >= minima, axis=1)

		# Returns True if a collision with any obstacle is detected and false otherwise
		return np.any(collision_mask)

	def goal_reached(self, current, tolerance=15.0):
		"""Checks whether or not the goal state has been reached within the tolerance specified"""
		current = np.array([current[0], current[1], current[2]])
		if np.linalg.norm(current - self.goal) < tolerance:
			return True
		return False


	def reconstruct(self):
		"""Reconstructs and returns the path from start state to goal state"""
		goal_index = len(self.explored) - 1
		start_index = 0
		path = [goal_index]
		current_index = goal_index
		while current_index != start_index:
			came_from_index = self.edges[current_index]
			path.append(came_from_index)
			current_index = came_from_index
		path = path[::-1]
		path = self.explored[path]
		return path

	def shorten(self, path):
		# Initialize an empty list for the shortened path
		shortened_path = [self.start]

		# Set the current state equal to the start state
		current_state = path[0]

		# Check for collision between the start state and all other states in the path
		collision_results = np.array([self.collides(path[0][:3], state[:3]) for state in path])

		# Get the maximum index that is False (indicating no collision)
		last_false_index = np.where(collision_results == False)[0][-1]

		# Append the path state corresponding to the max_false_index to the shortened_path list
		shortened_path.append(path[last_false_index])

		# Repeat steps 3-5 until reaching the end state
		while not np.array_equal(current_state, path[-1]):
			# Update the current state to be the last state added to the shortened path
			current_state = shortened_path[-1]

			# Check for collision between the current state and all other states in the path
			collision_results = np.array([self.collides(current_state[:3], state[:3]) for state in path])

			# Get the maximum index that is False (indicating no collision)
			last_false_index = np.where(collision_results == False)[0][-1]

			# Append the path state corresponding to the max_false_index to the shortened_path list
			if not np.array_equal(path[last_false_index], current_state):
				shortened_path.append(path[last_false_index])
		
		shortened_path = np.array(shortened_path)

		return shortened_path


	def plot_obstacles(self, ax):
		"""Plots the obstacles on a given axis `ax`"""
		plt.title("The Global Planning Algorithm")
		plt.xlabel("North (m)")
		plt.ylabel("East (m)")
		for i in range(len(self.obstacle_centers)):
			center = self.obstacle_centers[i]
			halfsize = self.obstacle_halfsizes[i]
			if center[2] + halfsize[2] >= self.goal[2]:
				rect = plt.Rectangle((center[0] - halfsize[0], center[1] - halfsize[1]), 
									 2*halfsize[0], 2*halfsize[1], 
									 color='Gray', alpha=0.5)
				ax.add_patch(rect)

	def animate(self, path):
		fig, ax = plt.subplots()

		# Plot obstacles
		self.plot_obstacles(ax)

		# scatter plot of explored nodes
		scatter = ax.scatter([], [], color='blue', s=1, label="Explored states")

		# lines plot for the edges
		lines = []

		# setup the plot limits
		ax.set_xlim(self.xbounds)
		ax.set_ylim(self.ybounds)

		# final path line
		final_path_line, = ax.plot([], [], color="r", label="Shortened path")
		ax.legend(loc="upper center", ncol=2)
		
		# initialize previous states
		self.prev_states = 0

		# function to draw each frame of the animation
		def update(num):
			"""Updates the plot for a given frame `num`"""
			# Only update if there's a new point to add
			if num < len(self.explored):
				# Skip frames by adjusting the increment value
				increment = 80  # Adjust the value to skip more or fewer frames
				if num % increment == 0 and num > self.prev_states:
					# Add new points to scatter plot
					scatter.set_offsets(self.explored[self.prev_states:num + 1, :2])
					self.prev_states = num

			else:
				# Draw the final path
				index = num - len(self.explored)
				if index < len(path):
					scatter.set_offsets(self.explored[:, :2])
					final_path_line.set_data(path[:index + 1, 0], path[:index + 1, 1])

			return [scatter] + lines + [final_path_line]		# function to draw each frame of the animation
	   

		# create animation using the animate() function
		anim = animation.FuncAnimation(fig, update, frames=len(self.explored) + len(path), interval=10, blit=True)
		pdb.set_trace()
		# Set up formatting for the movie files
		plt.rcParams['animation.convert_path'] = "C:\\Program Files\\ImageMagick-7.1.1-Q16-HDRI\\magick.exe"
		Writer = animation.writers['imagemagick']
		writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

		# Save the animation
		#anim.save('animation20.gif', writer=writer)

		plt.show()





# Define the obstacle geometry given.
## O_i = [x_c, y_c, z_c, hx, hy, hz]_i
## O = set of all obstacles in datasheet

# Import obstacles as array.
obstacles = np.genfromtxt('colliders.csv', delimiter=',', skip_header=2)

# Define a safety distance around obstacles. 
## Since the x and y halfsizes are all 5, set the safety distance to 2*5sqrt2
safety = 2 * 5 * np.sqrt(2)

# Define a start state in the local frame.
# Use 2D space for demonstration
s_start = np.array([0, 0, 5])

# Define a goal state in the local frame. 
s_goal =np.array([400,400,5])#np.array([475, -320 , 5])#np.array([588, 456 , 5]) #np.array([588, 456 , 5])#np.array([475, -320 , 5])


rrt = RRT(s_start, obstacles)
path = rrt.run(s_goal)
