import numpy as np
from collision_check import collision_check_vectorized, collision_check_two_points
import pdb

def shortcut(environment_data_object, input_path, points):

	E = len(input_path) - 1
	S = 0

	e = E
	s = S

	P = [E]

	while s < e:
		shortcut_found = False
		while s < e:
			if collision_check_two_points(environment_data_object, points[input_path[s]],points[input_path[e]]):
				s += 1
			else:
				P.append(s)
				shortcut_found = True
				break
		if shortcut_found:
			e = P[-1]
			s = 0
		else:
			e -= 1
			P.append(e)
			s = 0


	P = P[::-1]
	shorter_path = [input_path[p] for p in P]

	
	

	return shorter_path

