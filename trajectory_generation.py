import numpy as np
import pdb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy.polynomial import Polynomial

"""
Employs a seventh-order polynomial interpolation scheme to approximate a minimum snap trajectory
given an input of N waypoints and a desired average flight speed.

NOTE: It seems that currently, there is a numerical stability issue. Motion is smooth. However, the derivatives of motion don't seem to be as would be expected.
Edit: I have seemed to fix this issue. The way I fixed it was that I evaluated polynomials using numpy.polynomial instead of np.polyval (which is prone to rounding error
and is not advised to be used with "polynomials of high degree", as per the NumPy documentation.)
"""

class Trajectory:
    def __init__(self, x_dict, y_dict, z_dict, points, normalized_time, scale):
        self.x_dict = x_dict
        self.y_dict = y_dict
        self.z_dict = z_dict
        self.normalized_time = normalized_time
        self.scale = scale
        self.points = points

    def evaluate_position(self, time):
        """
        Evaluate the x, y, and z positions at a given time.
        """
        segment_index = np.searchsorted(self.normalized_time, time, side='right') - 1
        if segment_index == len(self.normalized_time)-1:
            segment_index -= 1
        x_pos = self.x_dict[segment_index]['position'](time)
        y_pos = self.y_dict[segment_index]['position'](time)
        z_pos = self.z_dict[segment_index]['position'](time)
        
        return [x_pos, y_pos, z_pos]

    def plot_3d_trajectory(self):
        """
        Plot the trajectory in 3D space.
        """
        normalized_times = np.linspace(self.normalized_time[0], self.normalized_time[-1], 1000)  # Adjust 1000 for resolution
        positions = [self.evaluate_position(t) for t in normalized_times]

        x_positions, y_positions, z_positions = zip(*positions)

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_positions, y_positions, z_positions)
        ax.scatter([p[0] for p in self.points], [p[1] for p in self.points], [p[2] for p in self.points])
        for i in range(len(self.normalized_time)):
            ax.text(self.points[i][0], self.points[i][1], self.points[i][2], f'{self.normalized_time[i]:.2f}', color='red')

        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Z Position')
        ax.set_title('3D Trajectory')

        plt.show()


    def compute_velocity(self, time):
        """
        Compute velocity at a given time.
        """
        # Compute the derivative of the position polynomial to get velocity.
        segment_index = np.searchsorted(self.normalized_time, time, side='right') - 1
        if segment_index == len(self.normalized_time) - 1:
            segment_index -= 1
        x_velocity = self.x_dict[segment_index]['position'].deriv()(time)
        y_velocity = self.y_dict[segment_index]['position'].deriv()(time)
        z_velocity = self.z_dict[segment_index]['position'].deriv()(time)

        return [x_velocity, y_velocity, z_velocity]

    def compute_acceleration(self, time):
        """
        Compute acceleration at a given time.
        """
        # Compute the derivative of the velocity polynomial to get acceleration.
        segment_index = np.searchsorted(self.normalized_time, time, side='right') - 1
        if segment_index == len(self.normalized_time) - 1:
            segment_index -= 1
        x_acceleration = self.x_dict[segment_index]['position'].deriv().deriv()(time)
        y_acceleration = self.y_dict[segment_index]['position'].deriv().deriv()(time)
        z_acceleration = self.z_dict[segment_index]['position'].deriv().deriv()(time)

        return [x_acceleration, y_acceleration, z_acceleration]

    def compute_jerk(self, time):
        """
        Compute jerk at a given time.
        """
        # Compute the derivative of the acceleration polynomial to get jerk.
        segment_index = np.searchsorted(self.normalized_time, time, side='right') - 1
        if segment_index == len(self.normalized_time) - 1:
            segment_index -= 1
        x_jerk = self.x_dict[segment_index]['position'].deriv().deriv().deriv()(time)
        y_jerk = self.x_dict[segment_index]['position'].deriv().deriv().deriv()(time)
        z_jerk = self.x_dict[segment_index]['position'].deriv().deriv().deriv()(time)

        return [x_jerk, y_jerk, z_jerk]

    def compute_snap(self, time):
        """
        Compute snap at a given time.
        """
        # Compute the derivative of the acceleration polynomial to get snap.
        segment_index = np.searchsorted(self.normalized_time, time, side='right') - 1
        if segment_index == len(self.normalized_time) - 1:
            segment_index -= 1
        x_snap = self.x_dict[segment_index]['position'].deriv().deriv().deriv().deriv()(time)
        y_snap = self.x_dict[segment_index]['position'].deriv().deriv().deriv().deriv()(time)
        z_snap = self.x_dict[segment_index]['position'].deriv().deriv().deriv().deriv()(time)

        return [x_snap, y_snap, z_snap]

    def plot_velocity(self):
        """
        Plot velocity vs. time.
        """
        normalized_times = np.linspace(self.normalized_time[0], self.normalized_time[-1], 1000)
        velocities = [self.compute_velocity(t) for t in normalized_times]

        x_velocities, y_velocities, z_velocities = zip(*velocities)

        plt.figure(figsize=(10, 6))
        plt.plot(normalized_times, x_velocities, label='X Velocity')
        plt.plot(normalized_times, y_velocities, label='Y Velocity')
        plt.plot(normalized_times, z_velocities, label='Z Velocity')
        plt.xlabel('Normalized Time')
        plt.ylabel('Velocity')
        plt.title('Velocity vs. Normalized Time')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_acceleration(self):
        """
        Plot acceleration vs. time.
        """
        normalized_times = np.linspace(self.normalized_time[0], self.normalized_time[-1], 1000)
        accelerations = [self.compute_acceleration(t) for t in normalized_times]

        x_accelerations, y_accelerations, z_accelerations = zip(*accelerations)

        plt.figure(figsize=(10, 6))
        plt.plot(normalized_times, x_accelerations, label='X Acceleration')
        plt.plot(normalized_times, y_accelerations, label='Y Acceleration')
        plt.plot(normalized_times, z_accelerations, label='Z Acceleration')
        plt.xlabel('Normalized Time')
        plt.ylabel('Acceleration')
        plt.title('Acceleration vs. Normalized Time')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_jerk(self):
        """
        Plot jerk vs. time.
        """
        normalized_times = np.linspace(self.normalized_time[0], self.normalized_time[-1], 1000)
        jerks = [self.compute_jerk(t) for t in normalized_times]

        x_jerks, y_jerks, z_jerks = zip(*jerks)

        plt.figure(figsize=(10, 6))
        plt.plot(normalized_times, x_jerks, label='X Jerk')
        plt.plot(normalized_times, y_jerks, label='Y Jerk')
        plt.plot(normalized_times, z_jerks, label='Z Jerk')
        plt.xlabel('Normalized Time')
        plt.ylabel('Jerk')
        plt.title('Jerk vs. Normalized Time')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_snap(self):
        """
        Plot snap vs. time.
        """
        normalized_times = np.linspace(self.normalized_time[0], self.normalized_time[-1], 1000)
        snaps = [self.compute_snap(t) for t in normalized_times]

        x_snaps, y_snaps, z_snaps = zip(*snaps)

        plt.figure(figsize=(10, 6))
        plt.plot(normalized_times, x_snaps, label='X snap')
        plt.plot(normalized_times, y_snaps, label='Y snap')
        plt.plot(normalized_times, z_snaps, label='Z snap')
        plt.xlabel('Normalized Time')
        plt.ylabel('snap')
        plt.title('snap vs. Normalized Time')
        plt.legend()
        plt.grid(True)
        plt.show()


class TrajectoryPlanner:
    def __init__(self, waypoint_sequence):
        self._waypoints = waypoint_sequence
        self._number_of_waypoints = len(waypoint_sequence)
        self._segment_count = len(waypoint_sequence) - 1
        self._number_of_intermediate_waypoints = len(waypoint_sequence) - 2
        self._number_intermediate_coeffs= 8 * self._number_of_intermediate_waypoints
        self._number_of_unknowns = 8 * self._segment_count
        self._xs = [waypoint[0] for waypoint in self._waypoints]
        self._ys = [waypoint[1] for waypoint in self._waypoints]
        self._zs = [waypoint[2] for waypoint in self._waypoints]


    def allocate_time(self, DESIRED_AVERAGE_SPEED):
        # Return a 1D array of distances between subsequent waypoints. The first entry of this array
        # should be the Euclidean distance between waypoint_sequence[0] and waypoint_sequence[1].
        waypoints_np = np.array(self._waypoints)
        distances = np.linalg.norm(waypoints_np[1:] - waypoints_np[:-1], axis=1)
        time_deltas = distances / DESIRED_AVERAGE_SPEED
        timestamps = np.concatenate(([0], np.cumsum(time_deltas)))
        #normalized_timestamps = timestamps / max(timestamps)
        #normalized_timestamps = 2 * (timestamps - min(timestamps)) / (max(timestamps) - min(timestamps)) - 1
        normalized_timestamps = timestamps
        self._timestamps = timestamps.tolist()
        self._normalized_timestamps = normalized_timestamps.tolist()


    def compute_complete_trajectory(self):
        x_polynomials = self.compute_trajectory(self._xs)
        y_polynomials = self.compute_trajectory(self._ys)
        z_polynomials = self.compute_trajectory(self._zs)
        print("An XYZ trajectory has been computed")
        return Trajectory(x_polynomials, y_polynomials, z_polynomials, self._waypoints, self._normalized_timestamps, max(self._timestamps))

    def compute_trajectory(self, xs):
        # A system of equations of the form Mc=b described with matrix-vector notation
        # `c` is the coefficient vector
        # `b` is a vector of constraints
        # `M` is the matrix obtained by collecting the set of seventh order polynomial equations describing
        #  the motion 

        # For readability...
        t = self._normalized_timestamps
        nic = self._number_intermediate_coeffs
        niw = self._number_of_intermediate_waypoints
        nos = self._segment_count
        n = self._number_of_waypoints
        nou = self._number_of_unknowns

        M = np.zeros((nou, nou))
        b = np.zeros((nou, ))

        # The initial and final conditions are that the drone is at rest (0 value for derivatives 1 to 3)
        # This therefore yields 6 initial conditions (2x3)
        # Here're those independent conditions regarding the first waypoint:
        M[0, :8] = np.array([7*t[0]**6, 6*t[0]**5, 5*t[0]**4, 4*t[0]**3, 3*t[0]**2, 2*t[0], 1, 0]); b[0] = 0
        M[1, :8] = np.array([42*t[0]**5, 30*t[0]**4, 20*t[0]**3, 12*t[0]**2, 6*t[0], 2, 0, 0]); b[1] = 0
        M[2, :8] = np.array([210*t[0]**4, 120*t[0]**3, 60*t[0]**2, 24*t[0], 6, 0, 0, 0]); b[2] = 0

        
        # Here're those independent conditions regarding the final waypoint:
        M[3, -8:] = np.array([7*t[-1]**6, 6*t[-1]**5, 5*t[-1]**4, 4*t[-1]**3, 3*t[-1]**2, 2*t[-1], 1, 0]); b[3] = 0
        M[4, -8:] = np.array([42*t[-1]**5, 30*t[-1]**4, 20*t[-1]**3, 12*t[-1]**2, 6*t[-1], 2, 0, 0]); b[4] = 0
        M[5, -8:] = np.array([210*t[-1]**4, 120*t[-1]**3, 60*t[-1]**2, 24*t[-1], 6, 0, 0, 0]); b[5] = 0

        # The position at each waypoint is prescribed, so this adds N independent conditions to the system of equations
        # The total count of equations is thus N + 6
        for i in range(n-1):
            M[6+i, i*8:8*(i+1)] = np.array([1*t[i]**7, 1*t[i]**6, 1*t[i]**5, 1*t[i]**4, 1*t[i]**3, 1*t[i]**2, 1*t[i], 1]); b[6+i] = xs[i]

        M[6+n-1, (n-2)*8:8*(n-1)] = np.array([1*t[-1]**7, 1*t[-1]**6, 1*t[-1]**5, 1*t[-1]**4, 1*t[-1]**3, 1*t[-1]**2, 1*t[-1], 1]); b[6+n-1] = xs[-1]
        print(6+n-1)
        # Enforce continuity at each of the N - 2 intermediate points for derivatives 0, 1, 2, 3, 4, 5, 6
        # This yields 7x(N - 2) more equations
        # The total count of equations is thus N + 6 + 7(N-2), which equals 8N - 8 (or 8(N-1), where N-1 is the number
        # of segments of the piecewise polynomial), which is what is required.
        for j in range(1, niw+1): # For each intermediate waypoint, place 7 equations into the system:

            # derivative 0
            M[6+n+(j-1)*7+0, (j-1)*8:(j+1)*8] = np.array([1*t[j]**7, 1*t[j]**6, 1*t[j]**5, 1*t[j]**4, 1*t[j]**3, 1*t[j]**2, 1*t[j], 1,
                                                    -1*t[j]**7, -1*t[j]**6, -1*t[j]**5, -1*t[j]**4, -1*t[j]**3, -1*t[j]**2, -1*t[j], -1])
            b[6+n+(j-1)*7+0] = 0

            # derivative 1
            M[6+n+(j-1)*7+1, (j-1)*8:(j+1)*8] = np.array([7*t[j]**6, 6*t[j]**5, 5*t[j]**4, 4*t[j]**3, 3*t[j]**2, 2*t[j]**1, 1, 0,
                                                    -7*t[j]**6, -6*t[j]**5, -5*t[j]**4, -4*t[j]**3, -3*t[j]**2, -2*t[j]**1, -1, 0])
            b[6+n+(j-1)*7+1] = 0

            # derivative 2
            M[6+n+(j-1)*7+2, (j-1)*8:(j+1)*8] = np.array([42*t[j]**5, 30*t[j]**4, 20*t[j]**3, 12*t[j]**2, 6*t[j], 2, 0, 0, 
                                                   -42*t[j]**5, -30*t[j]**4, -20*t[j]**3, -12*t[j]**2, -6*t[j], -2, 0, 0])
            b[6+n+(j-1)*7+2] = 0

            # derivative 3
            M[6+n+(j-1)*7+3, (j-1)*8:(j+1)*8] = np.array([210*t[j]**4, 120*t[j]**3, 60*t[j]**2, 24*t[j], 6, 0, 0, 0, 
                                                    -210*t[j]**4, -120*t[j]**3, -60*t[j]**2, -24*t[j], -6, 0, 0, 0])
            b[6+n+(j-1)*7+3] = 0

            # derivative 4
            M[6+n+(j-1)*7+4, (j-1)*8:(j+1)*8] = np.array([840*t[j]**3, 360*t[j]**2, 120*t[j], 24, 0, 0, 0, 0,
                                                    -840*t[j]**3, -360*t[j]**2, -120*t[j], -24, 0, 0, 0, 0])
            b[6+n+(j-1)*7+4] = 0

            # derivative 5
            M[6+n+(j-1)*7+5, (j-1)*8:(j+1)*8] = np.array([2520*t[j]**2, 720*t[j], 120, 0, 0, 0, 0, 0,
                                                    -2520*t[j]**2, -720*t[j], -120, 0, 0, 0, 0, 0])
            b[6+n+(j-1)*7+5] = 0

            # derivative 6
            M[6+n+(j-1)*7+6, (j-1)*8:(j+1)*8] = np.array([5040*t[j], 720, 0, 0, 0, 0, 0, 0,
                                                    -5040*t[j], -720, 0, 0, 0, 0, 0, 0])
            b[6+n+(j-1)*7+6] = 0

        coefficients = np.linalg.solve(M, b)
        coeffs_dict = self.arrange_coeffs(coefficients, nos)

        return coeffs_dict

    def arrange_coeffs(self, coeffs, num_segments):
        coeffs_dict = {}
        # i is the segment index number where 0 references the first segment in the trajectory
        for i in range(num_segments):
            coeffs_dict[i] = {}
            position_coeffs = coeffs[i*8:(i+1)*8][::-1]
            coeffs_dict[i]['position'] = Polynomial(position_coeffs)

        return coeffs_dict