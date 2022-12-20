import numpy as np
import math

def poly(coefficients, x):
    return sum([coefficients[i] * math.pow(x, i) for i in range(0, len(coefficients))])

def mydist(a, b):
    return np.linalg.norm(np.array(a)-np.array(b))

def point2func(t_l, x_l, y_l, dim=10):
    # print("----- t -----")
    # print(t_l)
    # print("----- x -----")
    # print(x_l)
    # print("----- y -----")
    # print(y_l)

    min_size = min([len(t_l), len(x_l), len(y_l)])

    if dim is None:
        dim = min_size

    point_matrix = np.array([x_l, y_l]).T
    # print("----- point_matrix -----")
    # print(point_matrix)

    origin_time_matrix = []
    for t in t_l:
        tmp = []

        for i in range(0, dim):
            tmp.append(math.pow(t, i))

        origin_time_matrix.append(tmp)

    time_matrix = np.array(origin_time_matrix)
    # # print("----- time_matrix -----")
    # # print(time_matrix)

    ans = np.dot(np.linalg.pinv(time_matrix), point_matrix)

    re_ans = np.dot(time_matrix, ans)
    # print(" ----- ans -----")
    # print(re_ans.T)
    # print(" ----- grand truth -----")
    # print(point_matrix.T)

    d_x = re_ans.T[0] - point_matrix.T[0]
    d_y = re_ans.T[1] - point_matrix.T[1]

    # print(" ----- diff -----")
    # print(d_x)
    # print(d_y)
    diff_x_max = max([math.fabs(d) for d in d_x])
    diff_y_max = max([math.fabs(d) for d in d_y])
    diff_max = max([math.sqrt(d_x[i] * d_x[i] + d_y[i] * d_y[i]) for i in range(0, len(d_x))])
    return ans, diff_x_max, diff_y_max, diff_max, list(re_ans.T[0]), list(re_ans.T[1])

def point2func(t_l, x_l, dim=10):
    # print("----- t -----")
    # print(t_l)
    # print("----- x -----")
    # print(x_l)
    # print("----- y -----")
    # print(y_l)

    min_size = min([len(t_l), len(x_l)])

    if dim is None:
        dim = min_size

    point_matrix = np.array([x_l]).T
    # print("----- point_matrix -----")
    # print(point_matrix)

    origin_time_matrix = [[math.pow(t, i) for i in range(0, dim)] for t in t_l]
    # for t in t_l:
    #     tmp = []
    #
    #     for i in range(0, dim):
    #         tmp.append(math.pow(t, i))
    #
    #     origin_time_matrix.append(tmp)

    time_matrix = np.array(origin_time_matrix)
    # # print("----- time_matrix -----")
    # # print(time_matrix)

    ans = np.dot(np.linalg.pinv(time_matrix), point_matrix)

    re_ans = np.dot(time_matrix, ans)
    # print(" ----- ans -----")
    # print(re_ans.T)
    # print(" ----- grand truth -----")
    # print(point_matrix.T)

    d_x = re_ans.T[0] - point_matrix.T[0]

    # print(" ----- diff -----")
    # print(d_x)
    # print(d_y)
    diff_x_max = max([math.fabs(d) for d in d_x])
    # diff_max = max([math.sqrt(d_x[i] * d_x[i] + d_y[i] * d_y[i]) for i in range(0, len(d_x))])
    return ans, diff_x_max, list(re_ans.T[0])
