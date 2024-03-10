import numpy as np
import pdb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Array with rows containing the elements x, y, z, dx, dy, dz (x,y,z) are center positions, (dx,dy,dz) are halfsizes
obstacles = np.array([[30,35,35,10,10,35], [60,75,45,10,10,45]])

# Waypoints are (x,y,z) coordinates
path = np.array([[10,40,5],[80,0,55]])

def potential_field(path, obstacles):
	start_pos = path[0, :]
	target_index = 1
	target_pos = path[target_index, :]
	current_pos = start_pos
	goal_pos = path[-1, :]
	collision_distance = 10. * np.sqrt(2)
	detection_distance = 4. * collision_distance
	step = 0.2
	repulsive_gain_max = 10.
	attractive_gain = 10.

	trajectory = []
	attractive_directions = []
	repulsive_directions = []
	attractive_forces = []
	repulsive_forces = []
	result_forces = []
	result_directions = []
	
	


	while np.linalg.norm(current_pos - goal_pos) >= 0.1:
		print(np.linalg.norm(current_pos - goal_pos))
		distances_to_all_obstacles = np.linalg.norm(obstacles[:,:3] - current_pos, axis=1)
		nearest_obstacle_index = np.argmin(distances_to_all_obstacles)
		nearest_obstacle_pos = obstacles[nearest_obstacle_index]
		nearest_obstacle_virtual_pos = np.array([nearest_obstacle_pos[0], nearest_obstacle_pos[1], current_pos[2]])
		ground_distance_to_obstacle_virtual_pos = np.linalg.norm(nearest_obstacle_virtual_pos - current_pos)
		direction_from_obstacle_to_current = (current_pos - nearest_obstacle_virtual_pos) / ground_distance_to_obstacle_virtual_pos 
		if np.linalg.norm(current_pos - target_pos) < 0.1:
			target_index += 1
			if target_index == len(path):
				break
		target_pos = path[target_index, :]
		direction_from_current_to_target = (target_pos - current_pos) / np.linalg.norm(target_pos - current_pos) 
		attractive_force = attractive_gain * direction_from_current_to_target

		# Repulsive gain
		if ground_distance_to_obstacle_virtual_pos <= detection_distance:
			repulsive_gain = repulsive_gain_max * (1 - ((ground_distance_to_obstacle_virtual_pos - collision_distance) / (detection_distance - collision_distance)))
		else:
			repulsive_gain = 0.0


		repulsive_force = repulsive_gain * direction_from_obstacle_to_current
		result_force = attractive_force + repulsive_force
		result_direction = result_force / np.linalg.norm(result_force)

		trajectory.append(current_pos)
		attractive_directions.append(direction_from_current_to_target)
		repulsive_directions.append(direction_from_obstacle_to_current)
		attractive_forces.append(attractive_force)
		repulsive_forces.append(repulsive_force)
		result_forces.append(result_force)
		result_directions.append(result_direction)



		current_pos = current_pos + step * result_direction
	data = [trajectory, attractive_directions, repulsive_forces, attractive_forces, result_forces, result_directions]
	data = np.array(data)

	return data
# Create the figure and 3D subplot

# Create the figure and 3D subplot
data = potential_field(path, obstacles)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the obstacles as boxes
for obstacle in obstacles:
	x, y, z, dx, dy, dz = obstacle
	ax.bar3d(x - dx, y - dy, z - dz, 2 * dx, 2 * dy, 2 * dz, color='red', alpha=1)

# Plot the waypoints as points
ax.scatter(path[:, 0], path[:, 1], path[:, 2], c='blue', marker='o', alpha=1)

# Plot the trajectory
ax.plot(data[0, :, 0], data[0, :, 1], data[0, :, 2], c='lime', alpha=1)
start_label = 'Current Position'
end_label = 'Goal Position'
ax.text(path[0, 0]-2, path[0, 1]-2, path[0, 2]+2, start_label, color='black', fontsize=12)
ax.text(path[-1, 0]-2, path[-1, 1]-2, path[-1, 2]-2, end_label, color='black', fontsize=12)

# Reshape attractive forces to match the shape of trajectory
attractive_forces_reshaped = data[3].reshape(data[0].shape[0], -1)

# Reshape repulsive forces to match the shape of trajectory
repulsive_forces_reshaped = data[2].reshape(data[0].shape[0], -1)

# Reshape result forces to match the shape of trajectory
result_forces_reshaped = data[4].reshape(data[0].shape[0], -1)

# Plot the vectors for every 100th position
for i in range(0, data[0].shape[0], 100):
	ax.quiver(data[0, i, 0], data[0, i, 1], data[0, i, 2],
			  attractive_forces_reshaped[i, 0], attractive_forces_reshaped[i, 1], attractive_forces_reshaped[i, 2],
			  color='blue', label='_nolegend_')

for i in range(0, data[0].shape[0], 100):
	ax.quiver(data[0, i, 0], data[0, i, 1], data[0, i, 2],
			  repulsive_forces_reshaped[i, 0], repulsive_forces_reshaped[i, 1], repulsive_forces_reshaped[i, 2],
			  color='red', label='_nolegend_')

#for i in range(0, data[0].shape[0], 100):
	#ax.quiver(data[0, i, 0], data[0, i, 1], data[0, i, 2],
	#		  result_forces_reshaped[i, 0], result_forces_reshaped[i, 1], result_forces_reshaped[i, 2],
	#		  color='green', label='_nolegend_')

# Set labels and title
ax.set_xlabel('North (m)')
ax.set_ylabel('East (m)')
ax.set_zlabel('Altitude (m)')
ax.set_title('The Obstacle Avoidance Capability')

# Set the limits of the plot
min_coord = np.min(np.concatenate([obstacles[:, :3], path], axis=0)) - 10
max_coord = np.max(np.concatenate([obstacles[:, :3], path], axis=0)) + 10
ax.set_xlim(0., max_coord)
ax.set_ylim(0., max_coord)
ax.set_zlim(0., max_coord)

# Create proxy artists for legend labels
attractive_proxy = plt.Line2D([0], [0], linestyle='-', color='blue')
repulsive_proxy = plt.Line2D([0], [0], linestyle='-', color='red')
result_proxy = plt.Line2D([0], [0], linestyle='-', color='green')

# Show the legend
#ax.legend([attractive_proxy, repulsive_proxy, result_proxy],
#		  ['Attractive Force', 'Repulsive Force', 'Resultant Force'],
#		  loc='upper center',bbox_to_anchor=(0.5, -0.1))

# Show the plot
plt.show()
pdb.set_trace()