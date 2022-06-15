import numpy as np
import math

def point2func(t_l, x_l, y_l, dim=10):
    print("----- t -----")
    print(t_l)
    print("----- x -----")
    print(x_l)
    print("----- y -----")
    print(y_l)

    min_size = min([len(t_l), len(x_l), len(y_l)])

    if dim is None:
        dim = min_size

    point_matrix = np.array([x_l, y_l]).T
    print("----- point_matrix -----")
    print(point_matrix)

    origin_time_matrix = []
    for t in t_l:
        tmp = []

        for i in range(0, dim):
            tmp.append(math.pow(t, i))

        origin_time_matrix.append(tmp)

    time_matrix = np.array(origin_time_matrix)
    print("----- time_matrix -----")
    print(time_matrix)

    ans = np.dot(np.linalg.pinv(time_matrix), point_matrix)

    re_ans = np.dot(time_matrix, ans)
    print(" ----- ans -----")
    print(re_ans.T)
    print(" ----- grand truth -----")
    print(point_matrix.T)

    d_x = re_ans.T[0] - point_matrix.T[0]
    d_y = re_ans.T[1] - point_matrix.T[1]

    print(" ----- diff -----")
    print(d_x)
    print(d_y)
    diff_x_max = max([math.fabs(d) for d in d_x])
    diff_y_max = max([math.fabs(d) for d in d_y])
    return ans, diff_x_max, diff_y_max
