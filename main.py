import environment_data as ed
import lattice
import numpy as np
import pdb
import search
from rrt_r2 import RRT
from shortcut import shortcut
from trajectory_generation import Trajectory, TrajectoryPlanner
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dynamics import Drone3D
from prm import PRM

ed_obj = ed.EnvironmentData("colliders.csv", 5.0)
ed_obj.summary()



#ed_obj.visualize()
center = np.array([0,0,100])
halfsizes = np.array([60,60,100])
lattice_obj = lattice.CubicLattice(ed_obj, center, halfsizes, resolution=25.0, connectivity="full")
#lattice_obj.visualize(ed_obj)

#lon, lat, alt
start_gps = [-122.39745, 37.79248, 0]
goal_gps = [-122.39645,  37.79278, 200]

# PRM
roadmap = PRM(ed_obj,DENSITY=2e-5, NEIGHBORS=5)
path = search.astar(roadmap, start_gps, goal_gps, search.euclidean_distance)
roadmap.visualize([-100, 100, -100, 100, 0, 200], path=path)
pdb.set_trace()
# Lattice examples
print("Lattice start")
path = search.bfs(lattice_obj, start_gps, goal_gps)
lattice_obj.visualize(ed_obj, path)
path = search.dfs(lattice_obj, start_gps, goal_gps)
lattice_obj.visualize(ed_obj, path)
path = search.ucs(lattice_obj, start_gps, goal_gps)
lattice_obj.visualize(ed_obj, path)
path = search.astar(lattice_obj, start_gps, goal_gps, search.euclidean_distance)
lattice_obj.visualize(ed_obj, path=path)
path_as_points = [lattice_obj.free_space_points[index] for index in path]
path_short = shortcut(ed_obj, path, lattice_obj.free_space_points)
lattice_obj.visualize(ed_obj, path=path_short)


#lon, lat, alt
start_gps = [-122.39741, 37.7911, 200]
goal_gps = [-122.39645,  37.7931, 50]

# RRT Example
print("RRT Start")
rrt = RRT(ed_obj, start_gps, goal_gps)
path = rrt.run()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
X,Y,Z,A,B,C = zip(*rrt._states)
ax.quiver(X,Y,Z, A,B,C, linewidth=1, length=5)
px, py, pz, pa, pb, pc = zip(*path)
ax.plot(px, py, pz, 'g-')
ax.scatter(px, py, pz, color='lime')
ax.scatter(*rrt._start_pos, color="green")
ax.scatter(*rrt._goal_pos, color="red")
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('RRT Path Planning')

# Determine the bounding box of RRT states
rrt_states = np.array(rrt._states)
min_bound = np.min(rrt_states[:, :3], axis=0) - 5  # Extra margin
max_bound = np.max(rrt_states[:, :3], axis=0) + 5  # Extra margin

# Function to check if boxes intersect
def boxes_intersect(center, halfsize, min_bound, max_bound):
	return all((center - halfsize) < max_bound) and all((center + halfsize) > min_bound)

# Visualize the relevant part of the environment
for center, halfsize in zip(ed_obj.centers, ed_obj.halfsizes):
	if boxes_intersect(center, halfsize, min_bound, max_bound):
		corner = center - halfsize
		full_size = 2 * halfsize
		ax.bar3d(corner[0], corner[1], 0, full_size[0], full_size[1], full_size[2], color='r', alpha=0.5)


# Example usage
waypoint_sequence = [p[:3] for p in path]
tp = TrajectoryPlanner(waypoint_sequence)
tp.allocate_time(DESIRED_AVERAGE_SPEED=1.0) # Numerical stability error
trajectory = tp.compute_complete_trajectory()
trajectory.plot_3d_trajectory()
trajectory.plot_velocity()
trajectory.plot_acceleration()
trajectory.plot_jerk()
trajectory.plot_snap()

plt.show()


def omega_profile(t, base_omega=670, deviation_factor=0.0001):
    """
    Omega profile for 3D motion with slight lateral deviations over a period of 5 seconds.
    """
    # Base motor speed
    omega_base = base_omega

    # Deviation for lateral movements
    omega_deviation = base_omega * deviation_factor

    if t < 1:  # Ascend
        omega_1 = omega_2 = omega_3 = omega_4 = omega_base
    elif t < 2:  # Slight move forward
        omega_1 = omega_2 = omega_base + omega_deviation
        omega_3 = omega_4 = omega_base - omega_deviation
    elif t < 3:  # Slight move right
        omega_1 = omega_3 = omega_base + omega_deviation
        omega_2 = omega_4 = omega_base - omega_deviation
    elif t < 4:  # Slight move left
        omega_2 = omega_4 = omega_base + omega_deviation
        omega_1 = omega_3 = omega_base - omega_deviation
    else:  # Descend
        omega_1 = omega_2 = omega_3 = omega_4 = omega_base - omega_deviation

    return omega_1, omega_2, omega_3, omega_4


# Initialize the Drone
drone = Drone3D()

# Simulation parameters
duration = 5  # total simulation time in seconds
num_steps = int(duration / drone.dt)
time = np.linspace(0, duration, num_steps)

# Arrays to store the state
positions = np.zeros((num_steps, 3))
orientations = np.zeros((num_steps, 3))
thrust_vectors = np.zeros((num_steps, 3))

# Run the simulation
for i in range(num_steps):
    omega_1, omega_2, omega_3, omega_4 = omega_profile(time[i])
    drone.advance_state(omega_1, omega_2, omega_3, omega_4)

    # Record position and orientation
    positions[i] = np.array([drone.x, drone.y, drone.z])
    orientations[i] = np.array([drone.phi, drone.theta, drone.psi])
    thrust_vectors[i] = np.dot(drone.R(), np.array([0., 0., 1.])) / np.linalg.norm(np.dot(drone.R(), np.array([0., 0., 1.])))
print(thrust_vectors)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create a 3D plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Set the axis limits
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(0, 20)

# Plot the drone's position
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Drone Position', linewidth=2)

# Set the value of N for plotting every N orientations
N = 50

# Create a quiver plot to represent thrust orientation with smaller arrow tips
arrow_length_ratio = 0.1  # Adjust this ratio to make arrow tips smaller or larger
ax.quiver(positions[::N, 0], positions[::N, 1], positions[::N, 2],
          thrust_vectors[::N, 0], thrust_vectors[::N, 1], thrust_vectors[::N, 2],
          length=0.5, normalize=True, color='red', label='Thrust Orientation',
          arrow_length_ratio=arrow_length_ratio)  # Set the arrow_length_ratio

# Set axis labels and title
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('Drone Position and Selected Thrust Orientations vs. Time')

# Add a legend
ax.legend()

# Show the plot
plt.show()
