import numpy as np

def collision_check_basic(environment_data_object, point):
    for idx, obstacle_center in enumerate(environment_data_object.centers):
        if abs(point[0] - obstacle_center[0]) <= environment_data_object.halfsizes[idx, 0]:
            if abs(point[1] - obstacle_center[1]) <= environment_data_object.halfsizes[idx, 1]:
                if abs(point[1] - obstacle_center[2]) <= environment_data_object.halfsizes[idx, 2]:
                    return True
    return False

def collision_check_vectorized(environment_data_object, point):
    broadcasted_point = np.tile(point, (len(environment_data_object.centers),1))
    deltas = np.abs(broadcasted_point - environment_data_object.centers)
    collision_conditions = (deltas <= environment_data_object.halfsizes)
    return np.any(np.all(collision_conditions, axis=1))

def collision_check_spatial(environment_data_object, point):
    _, indices = environment_data_object.ground_centers.query([point[:2]], k=4)
    for i in indices[0]:
        if environment_data_object.heights[i] >= point[2]:
            if abs(environment_data_object.centers[i][0] - point[0]) <= environment_data_object.halfsizes[i][0]:
                if abs(environment_data_object.centers[i][1] - point[1]) <= environment_data_object.halfsizes[i][1]:
                    return True
    return False

def inside_environment(environment_data_object, point):
    if point[0] < environment_data_object.xbounds[0] or point[0] > environment_data_object.xbounds[1]:
        return False
    if point[1] < environment_data_object.ybounds[0] or point[1] > environment_data_object.ybounds[1]:
        return False
    if point[2] < environment_data_object.zbounds[0] or point[2] > environment_data_object.zbounds[1]:
        return False
    return True

def collision_check_two_points(environment_data_object, point1, point2, SPACING=1.0):
    delta = point2 - point1
    distance = np.linalg.norm(delta)
    direction = delta / distance
    number_of_points = int(distance / SPACING)
    test_points = np.array([point1 + i * SPACING * direction for i in range(number_of_points + 1)])
    segment_evaluation = np.array([collision_check_vectorized(environment_data_object, test_point) for test_point in test_points])
    return np.any(segment_evaluation)
