import numpy as np
import math
# from scipy import spatial

def middle_val(val, min_val, max_val):
    if val < min_val:
        return min_val

    elif max_val < val:
        return max_val

    else:
        return val

def ACC_accel(length_l, al, vl, pl, vf, pf, max_accel):
    k1 = 0.025
    k2 = 0.41
    k3 = 19.3
    k4 = 0.56

    ta = k3 * al + k1 * (math.dist(pl, pf) - ACC_l(vf, length_l)) + k2 * (vl - vf) / k4

    if ta < -max_accel:
        return -max_accel

    elif max_accel < ta:
        return max_accel

    else:
        return ta

def ACC_l(c_v, length_l):
    l_safe = 5
    h = 1

    return h * c_v + l_safe + length_l

def liner_coefficients_by_2_points(p1, p2):
    # ex, y = ax + b, return a, b
    a = (p1[1] - p2[1]) / (p1[0] - p2[0])
    b = p1[1] - a * p1[0]

    return a, b

def cross_point(a1, b1, a2, b2):
    x = -(b1 - b2) / (a1 - a2)

    return [x, a1 * x + b1]

def distance_from_point(a, b, p):
    # ex. y = ax + b -> 0 = y -ax -b

    return math.fabs(p[1] -a * p[0] - b) / math.sqrt(1 * 1 + -a * -a)

def distance_between_points(p1, p2):
    return np.linalg.norm(np.array(p1), pn.array(p2))


def parse_xmlobj_of_element_tree(elem):
    result = {}

    for node in elem:
        if node.tag not in result.keys():
            result[node.tag] = []

        child_result = parse_xmlobj_of_element_tree(node)
        result[node.tag].append(dict(node.attrib, **dict(child_result)))

    return result

def rad_by_points(vec1, vec2):
    v1 = np.array(vec1)
    v2 = np.array(vec2)

    return np.arccos( np.dot(v1, v2) / np.linalg.norm(v1) * np.linalg.norm(v2) )


def cos_sim(v1, v2):
    # # print("--- in cos sim ---")
    # # print(v1)
    # # print(v2)
    # # print(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    # # print("--- end ---")
    return np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))

def divide_waypoints_by_point(waypoints, point):
    min_sim = float("inf")
    index = -1

    # # print(position)
    # # print(waypoints)
    for i in range(0, len(waypoints) - 1):
        origin_vec = np.array(waypoints[i + 1]) - np.array(waypoints[i])
        comp_vec = np.array(waypoints[i + 1]) - np.array(point)

        k = np.dot(origin_vec, comp_vec) / np.dot(origin_vec, origin_vec)
        print(f"p1: {waypoints[i]}, p2: {waypoints[i + 1]}, p: {point}, k: {k}")
        if i == 0 :
            if 1 <= k:
                return [], waypoints

        if 0 <= k <= 1:
            return waypoints[0:i+1], waypoints[i+1:]

    return waypoints, []
