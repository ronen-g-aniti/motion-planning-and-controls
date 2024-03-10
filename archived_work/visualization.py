# Visualization of RRT
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import cm
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
import numpy as np
import pdb

from matplotlib import rc

rc('font', family='Verdana')
# Load the data from the file
data = np.load('arrays.npz')
time_history = data['time_history']
pos = data['position_history']
obstacles = data['obstacles']
orig_waypoints = data['orig_waypoints']
data2 = np.load('state_space.npz')
shortened_path = data2['shortened_path']
states = data2['states']
connections = data2['connections']
start_waypoint = data2['start_waypoint']
goal_waypoint = data2['goal_waypoint']
leader_drone = np.load('leader_drone.npz')
leader_path = leader_drone['leader_path']
leader_data = leader_drone['leader_data'] # time, follower pos x, follower pos y, follower pos z, follower pos psi
leader_follow_points = leader_drone['leader_follow_points'] # Neast coarse points to follower position

# Create a 2D scatter plot of states with height as color
fig, ax = plt.subplots()
# Create colormap and normalization object
cmap = plt.cm.viridis
norm = Normalize(vmin=np.min(obstacles[:, 2] + obstacles[:, 5]), 
				 vmax=np.max(obstacles[:, 2] + obstacles[:, 5]))


# Plot obstacles as rectangles
for obstacle in obstacles:
	x, y, z, dx, dy, dz = obstacle
	rect_height = z + dz
	rect = Rectangle((x - dx, y - dy), 2 * dx, 2 * dy, color=cmap(norm(rect_height)), alpha=0.3) #color="red", 
	ax.add_patch(rect)
	#ax.text(x - dx, y - dy, f"{rect_height:0.0f}", fontsize=10)
# Create colorbar
sm = ScalarMappable(norm=norm, cmap=cmap)
sm.set_array([])
fig.colorbar(sm, ax=ax, orientation='vertical', label='Height (m)')


plt.xlim(np.amin(obstacles[:,0]-obstacles[:,3]), np.amax(obstacles[:,0]+obstacles[:,3]))
plt.ylim(np.amin(obstacles[:,1]-obstacles[:,4]), np.amax(obstacles[:,1]+obstacles[:,4]))
plt.title("The Obstacle Space")
plt.xlabel("North (m)")
plt.ylabel("East (m)")
plt.show()

pdb.set_trace()
plt.scatter(leader_path[::10,0], leader_path[::10,1], s=1)
for state in leader_path[::10]:
	ax.text(state[0], state[1], f"{state[2]:0.0f}", fontsize=10)
plt.show()
pdb.set_trace()
skip_count = 1
#plt.scatter(pos[::skip_count,0], pos[::skip_count,1], s=1, color="red", label="Actual path")

# Sampled states
plt.scatter(states[:, 0], states[:, 1], s=1, color=(0, 0, 1), label="Sampled states")

# Shortened path
plt.scatter(shortened_path[:, 0], shortened_path[:, 1], marker='*', s=5, color=(1, 0, 0), label="Shortened path", alpha=1)
plt.plot(shortened_path[:, 0], shortened_path[:, 1], linestyle='dashed')

# Start and goal states
plt.scatter(start_waypoint[0], start_waypoint[1], color=(0, 1, 1), label="Start state")
plt.scatter(goal_waypoint[0], goal_waypoint[1], color=(0, 1, 1), label="Goal state")

plt.legend()
#plt.scatter(orig_waypoints[:], s=2, color="blue")
#ax.scatter(states[:, 0], states[:, 1], color="blue", s=1) # Turn on or off states


#path = np.array(path)
#plt.plot(path[:, 0], path[:, 1], linewidth=1, color="black")
#plt.scatter(shortened_path[:, 0], shortened_path[:, 1], s=5, color="green", marker="s")
#plt.plot(shortened_path[:, 0], shortened_path[:, 1])
plt.show()


