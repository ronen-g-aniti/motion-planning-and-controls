import numpy as np
import pdb
"""

States:
	
	Sn = [[Sx1, Sy2, Sz1, Spsi1], ... [Sxn, Syn, Szn, Spsin]]

Obstacles:

	Ob = [[x1, y1, z1, dx1, dy1, dz1], ..., [xn, yn, zn, dxn, dyn, dzn]]
"""


# Making sense of the obstacles
# Convert obstacle datasheet into a n x 6 numpy array called obstacles.
# Name the important obstacle parameters for later use
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


# Helper functions
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

# RRT code
states = np.empty((0, 4))
step_dist = 3
connections = {}
s_start = np.array([0,0,0,0])
s_goal = np.array([300, 0, 10, 0])
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
	print("The path has been shortened")




# Visualization of RRT
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import cm

# Create a 2D scatter plot of states with height as color
fig, ax = plt.subplots()


# Plot obstacles as rectangles
for obstacle in obstacles:
	x, y, z, dx, dy, dz = obstacle
	rect_height = z + dz
	rect = Rectangle((x - dx, y - dy), 2 * dx, 2 * dy, color="red", alpha=0.3)
	ax.add_patch(rect)
	#ax.text(x - dx, y - dy, f"{rect_height:0.0f}", fontsize=10)
plt.title("The Obstacle Space")
plt.xlabel("North")
plt.ylabel("East")
ax.scatter(states[:, 0], states[:, 1], color="blue", s=1) # Turn on or off states


path = np.array(path)
plt.plot(path[:, 0], path[:, 1], linewidth=1, color="black")
plt.scatter(shortened_path[:, 0], shortened_path[:, 1], s=5, color="green", marker="s")
plt.plot(shortened_path[:, 0], shortened_path[:, 1])
plt.xlim(x_min, x_max)
plt.ylim(y_min, y_max)
plt.title(str(shortened_path[-1][:3]))
plt.show()

pdb.set_trace()

# Explaining the RRT
"""
Grow a tree from the start state S_start to the goal state S_goal
1. Sample a new state S_new at random that's inside O, the box bounded by all obstacles
2. Select its nearest neighbor state S_neighbor from the existing states S
3. Determine the direction vector U_hat between S_neighbor and S_new
4. Determine S_near, the state M units from S_neighbor in the direction U_hat
5. Check for a collision between S_neighbor and S_new
6. If there's not a collision, update D, the disctionary describing the connectivity of the state space, 
   with a mapping of S_near : S_neighbor, and append s_near to S, the array of states.
7. Repeat steps 1 - 6 until S_near is within N units from S_goal
"""

"""
# Visualize the obstacle space in 3d using a voxel map and a resolution of 1.
# Index 0 in the first dimension maps to x_center_min (north minimum center of obstacle position)
# Index 0 in the second dimension maps to y_center_min (east minimum center of obstacle position)
# Index 0 in the third dimension maps to z_center_mini (down center minimum of obstacle positions)


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# Initialize 3D grid
x_size, y_size, z_size = np.ceil((np.max(obstacles[:, :3] + obstacles[:, 3:], axis=0) - np.min(obstacles[:, :3] - obstacles[:, 3:], axis=0))).astype(int)
voxmap = np.zeros((x_size, y_size, z_size), dtype=bool)

# For each obstacle
for center, halfsize in zip(obstacle_centers, obstacle_halfsizes):
	# Calculate the range of indices that the obstacle covers
	x_range = range(max(0, int(center[0] - halfsize[0])), min(x_size, int(center[0] + halfsize[0]) + 1))
	y_range = range(max(0, int(center[1] - halfsize[1])), min(y_size, int(center[1] + halfsize[1]) + 1))
	z_range = range(max(0, int(center[2] - halfsize[2])), min(z_size, int(center[2] + halfsize[2]) + 1))
	
	# Mark the voxels in this range as True
	voxmap[np.ix_(x_range, y_range, z_range)] = True

# Now you have a voxel map with the obstacles set to True

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x, y, z = voxmap.nonzero()
ax.scatter(x, y, z, zdir='z', c='blue')

plt.show()
"""
