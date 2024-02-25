import numpy as np
import timeit
import environment_data as ed
import lattice
from collision_check import collision_check_spatial, collision_check_vectorized, collision_check_basic

# Assuming the EnvironmentData class is defined in the environment_data module
environment_data_object = ed.EnvironmentData("colliders.csv", 5.0)

# Calculate lower and upper bounds for testing
center = np.array([0, 0, 50])
halfsizes = np.array([100, 100, 50])
lower_bounds = center - halfsizes
upper_bounds = center + halfsizes

def random_point(lower_bounds, upper_bounds):
    return np.random.uniform(lower_bounds, upper_bounds, size=lower_bounds.shape)

def time_function(func, environment_data_object, lower_bounds, upper_bounds, number=1000):
    total_time = timeit.timeit(lambda: func(environment_data_object, random_point(lower_bounds, upper_bounds)), number=number)
    return total_time / number  # Average time per call

# Timing with random points
average_time_basic = time_function(collision_check_basic, environment_data_object, lower_bounds, upper_bounds)
average_time_vectorized = time_function(collision_check_vectorized, environment_data_object, lower_bounds, upper_bounds)
average_time_spatial = time_function(collision_check_spatial, environment_data_object, lower_bounds, upper_bounds)

print(f"Average Time for Basic Collision Check: {average_time_basic} seconds")
print(f"Average Time for Vectorized Collision Check: {average_time_vectorized} seconds")
print(f"Average Time for Spatial Collision Check: {average_time_spatial} seconds")


