from reference_frame import global_to_local
import numpy as np
import queue
import pdb
import heapq

def find_nearest(free_space_lattice_object, query_pos):
	_, indices = free_space_lattice_object.free_space_points_kd_tree.query([query_pos], k=1)
	return indices[0]

def bfs(free_space_lattice_object, start_gps, goal_gps):

	start_pos = global_to_local(start_gps, free_space_lattice_object.gps_home)
	start_index = find_nearest(free_space_lattice_object, start_pos)
	goal_pos = global_to_local(goal_gps, free_space_lattice_object.gps_home)
	goal_index = find_nearest(free_space_lattice_object, goal_pos)

	goal_found = False
	branch = {}
	branch[start_index] = None
	visited = set()

	fifo_queue = queue.Queue()
	fifo_queue.put(start_index)

	while not fifo_queue.empty():
		current_index = fifo_queue.get()
		visited.add(current_index)
		if current_index == goal_index:
			print("Goal found")
			goal_found = True
			break
		for neighbor_index, _ in valid_neighbors(free_space_lattice_object, current_index):
			if neighbor_index not in visited:
				fifo_queue.put(neighbor_index)
				branch[neighbor_index] = current_index
	
	if goal_found:
		path = []
		while current_index != start_index:
			path.append(current_index)
			current_index = branch[current_index]
		path.append(start_index)
		path = path[::-1]

	return path

def valid_neighbors(free_space_lattice_object, current_index):
    neighbors = free_space_lattice_object.graph[current_index]
    neighbor_pairs = [(neighbor_index, distance) for neighbor_index, distance in neighbors.items()]
    return neighbor_pairs

def dfs(free_space_lattice_object, start_gps, goal_gps):

	start_pos = global_to_local(start_gps, free_space_lattice_object.gps_home)
	start_index = find_nearest(free_space_lattice_object, start_pos)
	goal_pos = global_to_local(goal_gps, free_space_lattice_object.gps_home)
	goal_index = find_nearest(free_space_lattice_object, goal_pos)

	goal_found = False
	branch = {}
	branch[start_index] = None
	visited = set()

	lifo_queue = []
	lifo_queue.append(start_index)

	while lifo_queue:
		current_index = lifo_queue.pop()
		visited.add(current_index)
		if current_index == goal_index:
			print("Goal found")
			goal_found = True
			break
		for neighbor_index, _ in valid_neighbors(free_space_lattice_object, current_index):
			if neighbor_index not in visited:
				lifo_queue.append(neighbor_index)
				branch[neighbor_index] = current_index
	
	if goal_found:
		path = []
		while current_index != start_index:
			path.append(current_index)
			current_index = branch[current_index]
		path.append(start_index)
		path = path[::-1]

	return path

def euclidean_distance(lattice_object, lattice_index_1, lattice_index_2):
	return np.linalg.norm(lattice_object.free_space_points[lattice_index_1] - lattice_object.free_space_points[lattice_index_2])

def astar(free_space_lattice_object, start_gps, goal_gps, h):

	start_pos = global_to_local(start_gps, free_space_lattice_object.gps_home)
	start_index = find_nearest(free_space_lattice_object, start_pos)
	goal_pos = global_to_local(goal_gps, free_space_lattice_object.gps_home)
	goal_index = find_nearest(free_space_lattice_object, goal_pos)

	goal_found = False
	branch = {}
	branch[start_index] = [None, 0.0] 
	visited = set()

	priority_queue = []
	heapq.heappush(priority_queue, (0.0, start_index))

	counter = 0
	while priority_queue:
		counter += 1
		print("A STAR COUNTER ", counter)
		_, current_index = heapq.heappop(priority_queue)
		if current_index in visited:
			continue
		visited.add(current_index)
		if current_index == goal_index:
			print("Goal found")
			goal_found = True
			break
		for neighbor_index, distance in valid_neighbors(free_space_lattice_object, current_index):
			if neighbor_index in visited:
				continue
			g_score = branch[current_index][1]
			g_score_tentative = g_score + distance
			h_score_tentative = h(free_space_lattice_object, neighbor_index, goal_index)
			f_score_tentative = g_score_tentative + h_score_tentative
			if neighbor_index not in branch.keys() or ((neighbor_index in branch.keys()) and g_score_tentative <= branch[neighbor_index][1]):
				heapq.heappush(priority_queue, (f_score_tentative, neighbor_index))
				branch[neighbor_index] = [current_index, g_score_tentative]
	
	if goal_found:
		cost = branch[goal_index][1]
		path = []
		while current_index != start_index:
			path.append(current_index)
			current_index = branch[current_index][0]
		path.append(start_index)
		path = path[::-1]

	return path

def ucs(free_space_lattice_object, start_gps, goal_gps):

	start_pos = global_to_local(start_gps, free_space_lattice_object.gps_home)
	start_index = find_nearest(free_space_lattice_object, start_pos)
	goal_pos = global_to_local(goal_gps, free_space_lattice_object.gps_home)
	goal_index = find_nearest(free_space_lattice_object, goal_pos)

	goal_found = False
	branch = {}
	branch[start_index] = [None, 0.0] 
	visited = set()

	priority_queue = []
	heapq.heappush(priority_queue, (0.0, start_index))

	counter = 0
	while priority_queue:
		counter += 1
		print("UCS COUNTER ", counter)		
		_, current_index = heapq.heappop(priority_queue)
		if current_index in visited:
			continue
		visited.add(current_index)
		if current_index == goal_index:
			print("Goal found")
			goal_found = True
			break

		for neighbor_index, distance in valid_neighbors(free_space_lattice_object, current_index):
			if neighbor_index in visited:
				continue
			g_score = branch[current_index][1]
			g_score_tentative = g_score + distance
			if neighbor_index not in branch.keys() or ((neighbor_index in branch.keys()) and g_score_tentative <= branch[neighbor_index][1]):
				heapq.heappush(priority_queue, (g_score_tentative, neighbor_index))
				branch[neighbor_index] = [current_index, g_score_tentative]


	if goal_found:
		cost = branch[goal_index][1]
		path = []
		while current_index != start_index:
			path.append(current_index)
			current_index = branch[current_index][0]
		path.append(start_index)
		path = path[::-1]

	return path
