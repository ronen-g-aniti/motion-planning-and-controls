import numpy as np
import pdb
from scipy.spatial import KDTree
from collision_check import collision_check_spatial, collision_check_vectorized, collision_check_basic
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class CubicLattice:
    def __init__(self, environment_data_object, center, halfsizes, resolution=15.0, connectivity="full"):
        self._center = center
        self._halfsizes = halfsizes
        self._resolution = resolution
        self._connectivity = connectivity
        self._lower_bounds = center - halfsizes
        self._upper_bounds = center + halfsizes
        self._gps_home = environment_data_object.gps_home

        x_positions = np.arange(self._lower_bounds[0], self._upper_bounds[0] + resolution,
            resolution)
        y_positions = np.arange(self._lower_bounds[1], self._upper_bounds[1] + resolution,
            resolution)
        z_positions = np.arange(self._lower_bounds[2], self._upper_bounds[2] + resolution,
            resolution)
        X, Y, Z = np.meshgrid(x_positions, y_positions, z_positions,
            indexing='ij')

        points = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T

        free_space_mask = np.ones(len(points), dtype=bool)

        for idx, point in enumerate(points):
            if collision_check_vectorized(environment_data_object, point):
                free_space_mask[idx] = False

        self._free_space_points = points[free_space_mask]

        self._free_space_points_kd_tree = KDTree(self._free_space_points)

        self._graph = {}

        if connectivity == "full":
            query_radius = self._resolution * np.sqrt(2)
        elif connectivity == "partial":
            query_radius = self._resolution


        for i, point_i in enumerate(self._free_space_points):
            self._graph[i] = {}
            neighbors = self._free_space_points_kd_tree.query_ball_point(point_i, query_radius)
            for j in neighbors:
                if i != j:  # Avoid connecting a point to itself
                    distance = np.linalg.norm(point_i - self._free_space_points[j])
                    # collision check
                    vector = self._free_space_points[j] - point_i
                    unit_vector = (self._free_space_points[j] - point_i) / distance
                    spacing = 1.0
                    num_points = int(distance/spacing)
                    collision_test_points = np.array([point_i + i * spacing * unit_vector for i in range(num_points + 1)])
                    collision_found = False
                    for test_point in collision_test_points:
                        if collision_check_spatial(environment_data_object, test_point):
                            collision_found = True
                            break
                    if not collision_found:
                        self._graph[i][j] = distance

    @property
    def center(self):
        return self._center

    @property
    def halfsizes(self):
        return self._halfsizes

    @property
    def resolution(self):
        return self._resolution

    @property
    def connectivity(self):
        return self._connectivity

    @property
    def lower_bounds(self):
        return self._lower_bounds

    @property
    def upper_bounds(self):
        return self._upper_bounds

    @property
    def free_space_points(self):
        return self._free_space_points

    @property
    def free_space_points_kd_tree(self):
        return self._free_space_points_kd_tree

    @property
    def graph(self):
        return self._graph

    @property
    def gps_home(self):
        return self._gps_home
    


    def visualize(self, environment_data_object, path=None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plotting obstacles within lattice bounds in the XY plane
        for center, halfsize in zip(environment_data_object.centers, environment_data_object.halfsizes):
            # Check if the obstacle's XY footprint is within the lattice bounds
            if (self._lower_bounds[0] <= center[0] <= self._upper_bounds[0]) and (self._lower_bounds[1] <= center[1] <= self._upper_bounds[1]):
                corner = center - halfsize
                full_size = 2 * halfsize
                ax.bar3d(corner[0], corner[1], 0, full_size[0], full_size[1], full_size[2], color='r', alpha=0.5)

        # Plotting free space points
        ax.scatter(self._free_space_points[:, 0], self._free_space_points[:, 1], self._free_space_points[:, 2], color='g', alpha=0.5, s=10)

        # Plotting edges (connections)
        for point_index, connections in self._graph.items():
            for connection_index in connections.keys():
                point1 = self._free_space_points[point_index]
                point2 = self._free_space_points[connection_index]
                ax.plot([point1[0], point2[0]], [point1[1], point2[1]], [point1[2], point2[2]], color='b', alpha=0.5, linewidth=1)
        
        # Highlighting the path and adding arrows, if provided
        if path is not None:
            
            path_points = self._free_space_points[path]

            # Highlighting nodes on the path
            ax.scatter(path_points[:, 0], path_points[:, 1], path_points[:, 2], color='y', s=50)

            # Plotting arrows for the path
            for i in range(len(path) - 1):
                point1 = self._free_space_points[path[i]]
                point2 = self._free_space_points[path[i + 1]]
                # Calculating the vector (direction and magnitude) from point1 to point2
                direction = point2 - point1
                # Adding an arrow to the plot
                ax.quiver(point1[0], point1[1], point1[2], direction[0], direction[1], direction[2], color='orange', length=np.linalg.norm(direction), normalize=True)

            # Highlighting edges on the path
            for i in range(len(path) - 1):
                point1 = self._free_space_points[path[i]]
                point2 = self._free_space_points[path[i + 1]]
                ax.plot([point1[0], point2[0]], [point1[1], point2[1]], [point1[2], point2[2]], color='lime', linewidth=2)



        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        plt.title('3D Lattice Visualization with Edges')
        plt.show()


