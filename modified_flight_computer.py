import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from rrt_r2 import RRT
from environment_data import EnvironmentData
import matplotlib.pyplot as plt

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        #self.landing_transition()
                        print("Drone at target position")

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):

        self.flight_state = States.PLANNING

        # Extracts the home latitude and longitude from the colliders map file
        with open('colliders.csv', 'r') as file:
            first_line = file.readline().strip()
            parts = first_line.split(',')
            lat0 = float(parts[0].split()[1])
            lon0 = float(parts[1].split()[1])
        global_home = (lon0, lat0, 0.0)
        
        # Set home position to (lon0, lat0, 0)
        self.set_home_position(*global_home)
        
        # Generate EnvironmentData object from mapping data (CSV of bounding boxes)
        environment_data = EnvironmentData('colliders.csv', 5.0)

        # Set a target gps position
        goal_gps = (-122.39645,  37.79278, 200)

        # Extract current global position
        current_gps = self.global_position

        # Convert the global position to local coordinates
        current_local = global_to_local(current_gps, global_home)
        
        # Print global planning info to the console
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Initialize and run RRT global planning object
        rrt = RRT(environment_data, current_gps, goal_gps, GOAL_BIAS=0.75, MAX_STEER_ANGLE_RATE=np.pi/24, TIME_STEP=0.1, 
          TIME_INTERVAL=5.0, SPEED=2.0, MAX_ITERATIONS=10000, GOAL_TOLERANCE=1.0)

        # Compute the path of "states" (position, orientation) tuples,
        # where position is represented as (x,y,z) and orientation (phi, theta, psi) Euler angles
        print("Planning a coarse path...")
        path_of_states = rrt.run()

        # Post process the path: Since the drone can only accept (x,y,z, HEADING) as position command,
        # an extra step is necessary to transform the orientation vector components at each state of the path 
        # to a yaw command -- From first principles, the heading angle is the angle between the positive
        # x-direction and the vector formed by projecting the orientation vector (ex, ey, ez) onto the
        # ground plane (XY plane). To illustrate, state[4] is ey, and state[3] is ex.
        yaw_commands = [np.arctan2(state[4], state[3]) for state in path_of_states]

        # Convert path to waypoints
        waypoints = [[state[0], state[1], state[2], yaw_commands[i]] for i, state in enumerate(path_of_states)]

        # Ignore the first waypoint since the RRT algorithm counts the current position as the first 
        # waypoint
        self.waypoints = waypoints[1:]

        # Set the takeoff altitude
        self.target_position[2] = self.waypoints[0][2]



    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)
    drone.start()