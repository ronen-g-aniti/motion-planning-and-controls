import numpy as np
from scipy.spatial import KDTree
from reference_frame import global_to_local
from collision_check import collision_check_vectorized, collision_check_two_points
from search import astar
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb

class PRM:
	def __init__(self, environment_data_object, DENSITY=1e-4, NEIGHBORS=5):
		self.environment = environment_data_object
		self.SAMPLES = DENSITY * self.environment.lengths[0] * self.environment.lengths[1] * self.environment.lengths[2]
		self.NEIGHBORS = NEIGHBORS
		self.graph = {}
		self.points = []

		# To be removed later
		self.free_space_points = self.points

		self.gps_home = self.environment.gps_home

		# Take N valid samples
		self.take_n_samples(self.SAMPLES)

		# Connect nodes whose connecting segments don't intersect obstacles
		self.connect_valid_nodes()


		self.free_space_points_kd_tree = KDTree(self.points)

	def take_n_samples(self, N):
		
		# Sample a point one at a time, check for validity, increment n, stop when n=N		
		n = 0
		while n <= N:
			x = np.random.uniform(self.environment.xbounds[0], self.environment.xbounds[1])
			y = np.random.uniform(self.environment.ybounds[0], self.environment.ybounds[1])
			z = np.random.uniform(self.environment.zbounds[0], self.environment.zbounds[1])
			point = np.array([x, y, z])
			if not collision_check_vectorized(self.environment, point):
				self.points.append(point)
				n += 1

	def connect_valid_nodes(self):
		points_kd_tree = KDTree(self.points)
		for index1, point1 in enumerate(self.points):
			print("index ", index1)
			distances, indices = points_kd_tree.query(point1, k=self.NEIGHBORS + 1)
			for distance, index2 in zip(distances[1:], indices[1:]):
				point2 = self.points[index2]
				if not collision_check_two_points(self.environment, point1, point2, SPACING=1.0):
					if index1 not in self.graph:
						self.graph[index1] = {}
					self.graph[index1][index2] = distance


	def visualize(self, bounds, path=None):
		# Unpack the boundary values
		xmin, xmax, ymin, ymax, zmin, zmax = bounds

		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')

		# Plotting obstacles within bounds
		for center, halfsize in zip(self.environment.centers, self.environment.halfsizes):
			if (xmin <= center[0] <= xmax and ymin <= center[1] <= ymax and zmin <= center[2] <= zmax):
				corner = center - halfsize
				full_size = 2 * halfsize
				ax.bar3d(corner[0], corner[1], 0, full_size[0], full_size[1], full_size[2], color='r', alpha=0.5)


		# Plotting nodes within bounds
		filtered_points = [point for point in self.points if xmin <= point[0] <= xmax and ymin <= point[1] <= ymax and zmin <= point[2] <= zmax]
		if filtered_points:
			nodes = np.array(filtered_points)
			ax.scatter(nodes[:, 0], nodes[:, 1], nodes[:, 2], c='b', marker='o', s=10)

		# Plotting edges for nodes within bounds
		for index1, neighbors in self.graph.items():
			point1 = self.points[index1]
			if xmin <= point1[0] <= xmax and ymin <= point1[1] <= ymax and zmin <= point1[2] <= zmax:
				for index2 in neighbors:
					point2 = self.points[index2]
					if xmin <= point2[0] <= xmax and ymin <= point2[1] <= ymax and zmin <= point2[2] <= zmax:
						ax.plot([point1[0], point2[0]], [point1[1], point2[1]], [point1[2], point2[2]], 'lime')

		if path is not None:
			# Highlighting edges on the path
			for i in range(len(path) - 1):
				point1 = self.points[path[i]]
				point2 = self.points[path[i + 1]]
				ax.plot([point1[0], point2[0]], [point1[1], point2[1]], [point1[2], point2[2]], color=(0,0,1), linewidth=2,alpha=1)

		ax.set_xlim([xmin, xmax])
		ax.set_ylim([ymin, ymax])
		ax.set_zlim([zmin, zmax])
		ax.set_xlabel('X axis')
		ax.set_ylabel('Y axis')
		ax.set_zlabel('Z axis')
		plt.title('PRM Visualization: Nodes, Edges, and Obstacles')
		plt.show()



