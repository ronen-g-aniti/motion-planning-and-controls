import numpy as np
from trajectory_generation import Trajectory, TrajectoryPlanner
import pdb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

## There is currently an issue with computing trajectories with the cubic splines planning class ##

class TrajectoryPlannerSplines:
    def __init__(self, waypoint_sequence):
        self._waypoints = waypoint_sequence
        self._number_of_waypoints = len(waypoint_sequence)
        self._segment_count = len(waypoint_sequence) - 1
        self._number_of_intermediate_waypoints = len(waypoint_sequence) - 2
        self._number_intermediate_coeffs= 4 * self._number_of_intermediate_waypoints
        self._number_of_unknowns = 4 * self._segment_count
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
        normalized_timestamps = timestamps / max(timestamps)
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


        # The initial and final conditions are that the drone is at rest (0 value for derivatives 1 to 2)
        # This therefore yields 6 initial conditions (2x3)
        # Here're those independent conditions regarding the first waypoint:
        M[0, :4] = np.array([3*t[0]**2, 2*t[0], 1, 0]); b[0] = 0
        M[1, :4] = np.array([6*t[0], 2, 0, 0]); b[1] = 0
        
        # Here're those independent conditions regarding the final waypoint:
        M[2, -4:] = np.array([3*t[-1]**2, 2*t[-1], 1, 0]); b[2] = 0
        M[3, -4:] = np.array([6*t[-1], 2, 0, 0]); b[3] = 0

        # The position at each waypoint is prescribed, so this adds N independent conditions to the system of equations
        # The total count of equations is thus N + 6
        for i in range(n-1):
            M[4+i, i*4:4*(i+1)] = np.array([1*t[i]**3, 1*t[i]**2, 1*t[i], 1]); b[4+i] = xs[i]
        M[4+n-1, (n-2)*4:4*(n-1)] = np.array([1*t[-1]**3, 1*t[-1]**2, 1*t[-1], 1]); b[4+n-1] = xs[-1]

        pdb.set_trace()
        for i in range(niw): # For each intermediate waypoint, place 3 equations into the system:
            print(i)
            j = i + 1 # j references the original waypoint sequence: The 0th intermediate waypoint is actuall the first waypoint
                      # of the original waypoint sequence

            # derivative 0
            M[4+n+i*3+0, i*4:(i+2)*4] = np.array([1*t[j]**3, 1*t[j]**2, 1*t[j], 1,
                                                    -1*t[j]**3, -1*t[j]**2, -1*t[j], -1])
            b[4+n+i*3+0] = 0

            # derivative 1
            M[4+n+i*3+1, i*4:(i+2)*4] = np.array([3*t[j]**2, 2*t[j], 1, 0,
                                                    -3*t[j]**2, -2*t[j], -1, 0])
            b[4+n+i*3+1] = 0

            # derivative 2
            M[4+n+i*3+2, i*4:(i+2)*4] = np.array([6*t[j], 2, 0, 0, 
                                                   -6*t[j], -2, 0, 0])
            b[4+n+i*3+2] = 0

        
        print("Actual rank",np.linalg.matrix_rank(M))
        print("Required rank", nou)
        # With the M matrix constructed and the b vector also constructed, it's time to solve. The assumption is that M is an 
        # invertable matrix.
        print(np.linalg.cond(M))

        coefficients = np.linalg.solve(M, b)
        coeffs_dict = self.arrange_coeffs(coefficients, nos)

        return coeffs_dict

    def arrange_coeffs(self, coeffs, num_segments):
        coeffs_dict = {}
        # i is the segment index number where 0 references the first segment in the trajectory
        for i in range(num_segments):
            coeffs_dict[i] = {}
            position_coeffs = coeffs[i*4:(i+1)*4]
            coeffs_dict[i]['position'] = position_coeffs
            coeffs_dict[i]['velocity'] = np.polyder(position_coeffs, 1)
            coeffs_dict[i]['acceleration'] = np.polyder(position_coeffs, 2)
            coeffs_dict[i]['jerk'] = np.polyder(position_coeffs, 3)
            coeffs_dict[i]['snap'] = np.polyder(position_coeffs, 4)

        return coeffs_dict