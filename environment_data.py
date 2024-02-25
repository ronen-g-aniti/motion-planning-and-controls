import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree

class EnvironmentData:
    def __init__(self, obstacle_geometry_file, margin_of_safety):
        # Reading obstacle geometry from a file
        obstacle_geometry_as_array = np.genfromtxt(obstacle_geometry_file, delimiter=',', skip_header=2)
        
        # Assuming 'csv_file' is meant to be 'obstacle_geometry_file' for reading geodetic positions
        geodetic_position_as_array = np.genfromtxt(obstacle_geometry_file, delimiter=',', dtype='str', max_rows=1)
        
        # Extracting home longitude and latitude
        self._home_latitude = float(geodetic_position_as_array[0].split()[1])
        self._home_longitude = float(geodetic_position_as_array[1].split()[1])
        self._home_altitude = 0.0
        self._gps_home = np.array([self._home_longitude, self._home_latitude, self._home_altitude])
        
        # Setting margin of safety and calculating centers, half-sizes, and heights
        self._margin_of_safety = margin_of_safety
        self._centers = obstacle_geometry_as_array[:, :3]
        self._halfsizes = obstacle_geometry_as_array[:, 3:] + self._margin_of_safety
        self._heights = self._centers[:, 2] + self._halfsizes[:, 2]
        
        # Calculating bounds and lengths
        xmin = np.min(self._centers[:, 0] - self._halfsizes[:, 0])
        xmax = np.max(self._centers[:, 0] + self._halfsizes[:, 0])
        ymin = np.min(self._centers[:, 1] - self._halfsizes[:, 1])
        ymax = np.max(self._centers[:, 1] + self._halfsizes[:, 1])
        zmin = np.min(self._centers[:, 2] - self._halfsizes[:, 2])
        zmax = np.max(self._centers[:, 2] + self._halfsizes[:, 2])
        self._xbounds = np.array([xmin, xmax])
        self._ybounds = np.array([ymin, ymax])
        self._zbounds = np.array([zmin, zmax])
        self._lengths = np.array([xmax - xmin, ymax - ymin, zmax - zmin])

        # Incorporating spatial data structure for fast nearest neighbor obstacle lookups
        self._ground_centers = KDTree(self._centers[:,:2]) 

    # Property methods to access private attributes
    @property
    def home_latitude(self):
        return self._home_latitude

    @property
    def home_longitude(self):
        return self._home_longitude

    @property
    def gps_home(self):
        return self._gps_home
  
    @property
    def margin_of_safety(self):
        return self._margin_of_safety

    @property
    def centers(self):
        return self._centers

    @property
    def halfsizes(self):
        return self._halfsizes

    @property
    def heights(self):
        return self._heights

    @property
    def xbounds(self):
        return self._xbounds

    @property
    def ybounds(self):
        return self._ybounds

    @property
    def zbounds(self):
        return self._zbounds

    @property
    def lengths(self):
        return self._lengths

    @property
    def ground_centers(self):
        return self._ground_centers

    # Method to print a summary of the environment data
    def summary(self):
        print("Environment Data Summary:")
        print(f"Home Latitude: {self.home_latitude}")
        print(f"Home Longitude: {self.home_longitude}")
        print(f"Margin of Safety: {self.margin_of_safety}")
        print(f"Centers: \n{self.centers}")
        print(f"Half-sizes: \n{self.halfsizes}")
        print(f"Heights: \n{self.heights}")
        print(f"X Bounds: {self.xbounds}")
        print(f"Y Bounds: {self.ybounds}")
        print(f"Z Bounds: {self.zbounds}")
        print(f"Lengths: {self.lengths}")

    # Method to visualize the environment data in 3D
    def visualize(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for center, halfsize in zip(self.centers, self.halfsizes):
            corner = center - halfsize
            full_size = 2 * halfsize
            ax.bar3d(corner[0], corner[1], 0, full_size[0], full_size[1], full_size[2], color='r', alpha=0.5)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        plt.title('3D Obstacle Visualization')
        plt.show()

    def add_obstacles(centers, halfsizes):
        pass