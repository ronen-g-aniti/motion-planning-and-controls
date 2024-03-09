"""
This file contains a GridMap class that extends the Map class to provide a 2D grid-based representation
of the drone's environment, along with functions to build the grid, search the grid for a path, and 
visualize the grid and path.
Ronen Aniti
3/30/2023
"""

from map_class import Map
import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue
from typing import List, Tuple, Union, Dict, Any

class GridMap(Map):
    """
    A 2d grid representation of the drone's environment
    """
    def __init__(self, filename: str, title: str, safety: float, global_home: Tuple[float, float, float], 
                 current_local_position: Tuple[float, float, float], goal_local_position: Tuple[float, float, float]):
        super().__init__(filename, title, safety, global_home, current_local_position, goal_local_position)
        self.grid = self.compute_grid()
        

    def compute_grid(self):
        """
        Build a grid to represent the drone's environment
        """
        
        # First, initialize a grid of zeros
        grid = np.zeros((self.map_size[0], self.map_size[1]), dtype='float64')
        
        # Second, build a grid representation of the drone's environment
        obstacle_boundaries = [0, 0, 0, 0]
        ## Iterate through the map data file to do this
        for i in range(self.data.shape[0]):
            north, east, down, d_north, d_east, d_down = self.data[i, :]
            if (down + d_down) + self.safety > self.goal_altitude:
                obstacle_boundaries = [
                    int(north - self.ned_boundaries[0] - d_north - self.safety),
                    int(north - self.ned_boundaries[0] + d_north + self.safety),
                    int(east - self.ned_boundaries[2] - d_east - self.safety),
                    int(east - self.ned_boundaries[2] + d_east + self.safety)
                ]
                grid[obstacle_boundaries[0] : obstacle_boundaries[1]+1, obstacle_boundaries[2] : obstacle_boundaries[3]+1] = 1.0

        # Third, return the grid map
        return grid
    
    def search_grid(self):
        """
        Generates a position command sequence that will bring the drone from start to goal
        """
        
        current_local_position = self.current_local_position
        current_local_position[2] = self.goal_altitude # The first waypoint will be at the target altitude
        current_north, current_east = self.current_grid_location
        goal_north, goal_east = self.goal_grid_location

        # First, raise an excpetion if the start or end grid cells are occupied
        if self.grid[current_north, current_east] == 1:
            raise Exception('Invalid start node')
        if self.grid[goal_north, goal_east] == 1:
            raise Exception('Invalid goal node')


        # Second, begin an A* search routine
        frontier = PriorityQueue()
        visited = set()
        frontier.put((0.0, self.current_grid_location))
        visited.add(self.current_grid_location)
        travel_cost = 0.0
        grid_sequence = []
        found = False
        pathinfo = {}

        ## Repeat this subroutine until a path from start to goal is found
        while not frontier.empty():
            
            _, gridcell = frontier.get()

            if gridcell == self.goal_grid_location:
                found = True
                print('Found a path.')
                break

            free_neighbors = self.explore_free_neighbors(gridcell)
            for free_neighbor in free_neighbors:
                candidate_cell = free_neighbor[0]
                candidate_north, candidate_east = candidate_cell
                heuristic_cost = np.sqrt((candidate_north - self.goal_grid_location[0])**2 + (candidate_east - self.goal_grid_location[1])**2)
                action_cost = free_neighbor[1] 
                incremental_cost = action_cost + heuristic_cost
                candidate_total_cost = travel_cost + incremental_cost
                if candidate_cell not in visited:
                    frontier.put((candidate_total_cost, candidate_cell))
                    pathinfo[candidate_cell] = (gridcell, action_cost)
                    visited.add(candidate_cell)
        
        # Third, once the goal gridcell has been found, generate a sequence of gricells from star to goal
        if found:
            subgoal = self.goal_grid_location
            origin, action_cost = pathinfo[subgoal]
            grid_sequence.append(subgoal)
            while origin != self.current_grid_location:
                subgoal = origin
                origin, action_cost = pathinfo[subgoal]
                grid_sequence.append(subgoal)
            grid_sequence.append(origin)
            grid_sequence = grid_sequence[::-1]

            
            
            ## Remove collinear gridcells from the waypoint sequence
            grid_sequence = self.remove_collinear(grid_sequence)

            ## Plot the updated grid sequence
            #self.plot_path(grid_sequence)

            ## Convert the grid sequence into a waypoint sequence
            waypoint_commands = [self.grid_to_waypoint(grid_sequence[i], grid_sequence[i-1]) for i in range(1, len(grid_sequence))]
            
            ## Return the waypoint sequence
            return waypoint_commands

        else:
            raise Exception('Failed to find a path.')

    
    def remove_collinear(self, grid_sequence: List[Tuple[int,int]]) -> List[Tuple[int,int]]:
        """
        Removes collinear gridcells
        """
        i = 0 
        while i+2 < len(grid_sequence):
            x1 = grid_sequence[i][0]
            y1 = grid_sequence[i][1]
            x2 = grid_sequence[i+1][0]
            y2 = grid_sequence[i+1][1]
            x3 = grid_sequence[i+2][0]
            y3 = grid_sequence[i+2][1]

            collinear = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2) == 0

            if collinear:
                del(grid_sequence[i+1])
            else:
                i += 1
        return grid_sequence


    def grid_to_waypoint(self, gridcell: Tuple[int, int], previous_gridcell: Union[None, Tuple[int, int]] = None) -> List[float]:
        """
        Converts a grid cell into a waypoint command
        """
        if previous_gridcell is not None:
            heading = np.arctan2(gridcell[1] - previous_gridcell[1], gridcell[0] - previous_gridcell[0])
        else:
            heading = 0.0

        waypoint = np.array([gridcell[0] + self.ned_boundaries[0],
                             gridcell[1] + self.ned_boundaries[2],
                             self.goal_altitude,
                             heading
                            ])

        return list(waypoint)




    def explore_free_neighbors(self, gridcell: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        """
        Returns a gridcell's free neighbors along with the cost of getting to each
        """
        
        # First, define the action set
        actions = {
            'LEFT': (0, -1, 1),
            'RIGHT': (0, 1, 1),
            'UP': (-1, 0, 1),
            'DOWN': (1, 0, 1),
            'NORTHEAST': (-1, 1, np.sqrt(2)),
            'NORTHWEST': (-1, -1, np.sqrt(2)),
            'SOUTHEAST': (1, 1, np.sqrt(2)),
            'SOUTHWEST': (1, -1, np.sqrt(2))
        }
        
        grid_north, grid_east = gridcell

        free_neighbors = []
        for value in actions.values():
            north_delta, east_delta, action_cost = value
            candidate_cell_north = grid_north + north_delta
            candidate_cell_east = grid_east + east_delta
            candidate_cell = (candidate_cell_north, candidate_cell_east)
            if candidate_cell_north >= 0 and candidate_cell_north < self.map_size[0] and candidate_cell_east >= 0 and candidate_cell_east < self.map_size[1]:
                if self.grid[candidate_cell_north, candidate_cell_east] == 0.0:
                    free_neighbors.append((candidate_cell, action_cost))


        return free_neighbors

    def plot_grid(self) -> None:
        plt.imshow(self.grid, origin='lower', cmap='Greys')
        plt.title(self.title)
        plt.xlabel('Eastings (m)')
        plt.ylabel('Northings (m)')
        plt.show()

    def plot_path(self, grid_sequence: List[Tuple[int,int]]) -> None:
        """
        Plots the path from start to goal
        """

        grid_sequence_north = []
        grid_sequence_east = []
        for grid_cell in grid_sequence:
            grid_sequence_north.append(grid_cell[0])
            grid_sequence_east.append(grid_cell[1])
        
        plt.imshow(self.grid, origin='lower', cmap='Greys')
        plt.plot(grid_sequence_east, grid_sequence_north, color='blue', linestyle='-', marker='o')
        plt.title(self.title)
        plt.xlabel('Eastings (m)')
        plt.ylabel('Northings (m)')
        plt.show()
