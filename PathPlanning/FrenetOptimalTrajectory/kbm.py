from math import cos, sin, tan
from math import atan as arctan
from numpy import linalg
import numpy as np
import random
import argparse
import math
import json
import time

import matplotlib.pyplot as plt

from func import point2func

def multi_container(t, x, max_dim, error_th):
    efficiency = -10
    result = {
        "efficient_point": 0,
        "efficient_dim":  0,
        "coefficients": [],
        "ans": []
    }

    for dim in range(1, max_dim + 1):
        for point in range(1, len(t) + 1):
            # print(t)
            coef, diff_x_max, ans_x = point2func(t[:point], x[:point], dim)

            # print(f"middle: {point}, {len(t)}, {point == len(t)}")

            if error_th < diff_x_max:
                # print(f"{efficiency}, {(point - 1) / float(dim)}")
                if efficiency < (point - 1) / float(dim):
                    # print(f"{point}, {len(t[(point):])}")
                    t_coef, t_diff_x_max, t_ans_x = point2func(t[:point - 1], x[:point - 1], dim)
                    # print(f"over error: {dim}, {point - 1}, {t_diff_x_max}")

                    efficiency = (point - 1) / float(dim)
                    result["efficient_point"] = point - 1
                    result["efficient_dim"] = dim
                    result["coefficients"] = t_coef
                    result["ans"] = t_ans_x
                    result["diff_max"] = t_diff_x_max
                    result["error_th"] = error_th
                    result["type"] = "less error"

                    assert(len(t_ans_x) == (point - 1))
                    break

                else:
                    break

            elif point == len(t):
                # print(f"{efficiency}, {(point) / float(dim)}")
                if efficiency < (point) / float(dim):
                    # print(f"{point}, {len(t[(point):])}")
                    # coef, diff_x_max, ans_x = point2func(t[:point], x[:point], dim)
                    # print(f"last: {dim}, {point}, {diff_x_max}")
                    efficiency = point / float(dim)
                    result["efficient_point"] = point
                    result["efficient_dim"] = dim
                    result["coefficients"] = coef
                    result["ans"] = ans_x
                    result["diff_max"] = diff_x_max
                    result["error_th"] = error_th
                    result["type"] = "last"

                    assert(len(ans_x) == (point))
                    break

                else:
                    break


            elif diff_x_max <= error_th:
                # print(f"less error: {dim}, {point}, {diff_x_max}")
                continue

            else:
                raise Exception("unexpected state.")

    if result["efficient_point"] <= 0:
        # print(result)
        # print(x)
        # print(t)
        # print("!!!!")
        # print(efficiency)
        return []

    if result["efficient_point"] == len(x):
        # print("----")
        return [result]

    else:
        point = result["efficient_point"]
        # print(f"last: {point}, {len(t[(point):])}")
        return [result] + multi_container(t[(point):], x[(point):], max_dim, error_th)

def clip(v, v_min, v_max):
    if v < v_min:
        return v_min

    elif v_max < v:
        return v_max

    else:
        return v

def kbm(x_c, y_c, v_c, theta_c, acc, stear, d_t, veh_length):
    # cite from: https://shuffleai.blog/blog/Simple_Understanding_of_Kinematic_Bicycle_Model.html

    x_next = x_c + v_c * cos(theta_c) * d_t
    y_next = y_c + v_c * sin(theta_c) * d_t
    v_next = clip(v_c + acc * d_t, 0, float('inf'))
    theta_next = theta_c + v_c * (tan(stear) / veh_length) * d_t

    # print(f"x: {x_next}, y: {y_next}, v: {v_next}, theta: {math.degrees(theta_next)}")
    return x_next, y_next, v_next, theta_next

class Car:
    def __init__(self, x_init, y_init, v_init, theta_init, s_init, d_t=0.1, veh_length=5, v_max=200.0/3600.0, s_max = 45, sv_max = 0.1):
        self.x_log = [x_init]
        self.y_log = [y_init]
        self.v_log = [v_init]
        self.t_log = [0]
        self.s_log = [s_init]
        self.theta_log = [theta_init]

        self.d_t = d_t
        self.veh_length = veh_length
        self.vmax = v_max
        self.s_max = s_max
        self.sv_max = sv_max


    def path(self):
        return self.x_log, self.y_log, self.t_log

    def path_by_delta_t(self, d_t):
        x_r = [self.x_log[0]]
        y_r = [self.y_log[0]]
        t_r = [self.t_log[0]]

        t_pivot = self.t_log[0]
        for i in range(0, len(self.t_log)):
            if d_t <= self.t_log[i] - t_pivot:
                t_pivot = self.t_log[i]
                x_r.append(self.x_log[i])
                y_r.append(self.y_log[i])
                t_r.append(self.t_log[i])

        return x_r, y_r, t_r


    def path_by_point_num(self, point_num, skip_num):
        return (self.x_log[0::skip_num])[0:point_num], (self.y_log[0::skip_num])[0:point_num], (self.t_log[0::skip_num])[0:point_num]

    def path_filterd_by_ETSI(self):
        x_r = [self.x_log[0]]
        y_r = [self.y_log[0]]
        t_r = [self.t_log[0]]
        v_r = [self.v_log[0]]
        theta_r = [self.theta_log[0]]

        for i in range(0, len(self.t_log)):
            dt = self.t_log[i] - t_r[-1]
            dv = self.v_log[i] - v_r[-1]
            dh = self.theta_log[i] - theta_r[-1]
            # print(self.x_log[i])
            # print(self.y_log[i])
            # print(np.array([float(self.x_log[i]), float(self.y_log[i])]))
            # print(np.array([float(x_r[-1]), float(y_r[-1])]))
            # print(np.array(float(self.x_log[i]), float(self.y_log[i])))
            # print(np.array(float(x_r[-1]), float(y_r[-1])))
            dd = math.dist([float(self.x_log[i]), float(self.y_log[i])], [float(x_r[-1]), float(y_r[-1])])

            if 0.1 <= self.t_log[i] - t_r[-1]:
                if 1.0 <= dt or 0.5 <= dv or 4 <= dh or 4 <= dd:
                    x_r.append(self.x_log[i])
                    y_r.append(self.y_log[i])
                    t_r.append(self.t_log[i])
                    v_r.append(self.v_log[i])

        return x_r, y_r, t_r

    def latest_time(self):
        return self.t_log[-1]

    def generate_path_by_delta_t(self, d_t, acc, stear):
        x_next, y_next, v_next, theta_next = kbm(
            self.x_log[-1],
            self.y_log[-1],
            self.v_log[-1],
            self.theta_log[-1],
            acc,
            stear,
            d_t,
            self.veh_length,
        )

        self.x_log.append(x_next)
        self.y_log.append(y_next)
        self.v_log.append(clip(v_next, 0.0, self.vmax))
        self.theta_log.append(theta_next)
        self.t_log.append(self.latest_time() + d_t)
        self.s_log.append(stear)


    def generate_path_by_rotate(self, acc, rotate_radian):
        theta_init = self.theta_log[-1]
        target_theta = self.theta_log[-1] + rotate_radian
        origin_stear = arctan((target_theta - self.theta_log[-1]) * (self.veh_length / (self.v_log[-1] * self.d_t)))

        # print(f"origin_stear: {origin_stear}, sv_max: {self.sv_max}")
        if -self.sv_max * self.d_t <= origin_stear and origin_stear <= self.sv_max * self.d_t:
            self.generate_path_by_delta_t(self.d_t, acc, origin_stear)
            self.generate_path_by_delta_t(self.d_t, acc, 0)

            return 0

        else:
            _, _, _, theta_next_high = kbm(
                self.x_log[-1],
                self.y_log[-1],
                self.v_log[-1],
                self.theta_log[-1],
                acc,
                self.s_log[-1] + self.sv_max * self.d_t,
                self.d_t,
                self.veh_length,
            )

            _, _, _, theta_next_midle = kbm(
                self.x_log[-1],
                self.y_log[-1],
                self.v_log[-1],
                self.theta_log[-1],
                acc,
                self.s_log[-1],
                self.d_t,
                self.veh_length,
            )

            _, _, _, theta_next_low = kbm(
                self.x_log[-1],
                self.y_log[-1],
                self.v_log[-1],
                self.theta_log[-1],
                acc,
                self.s_log[-1] - self.sv_max * self.d_t,
                self.d_t,
                self.veh_length,
            )

        # print(f"target_theta: {target_theta}, stear: {self.s_log[-1]}, theta: {self.theta_log[-1]}, rotate_radian: {rotate_radian}")
        # print(f"theta_next_high: {theta_next_high}, theta_next_midle: {theta_next_midle}, theta_next_low: {theta_next_low}")

        n = 1
        if self.s_log[-1] != 0:
            n = math.fabs(self.s_log[-1] / (self.d_t * self.d_t))

        if theta_next_high <= self.theta_log[-1] + rotate_radian / n:
            self.generate_path_by_delta_t(self.d_t, acc, clip(self.s_log[-1] + self.sv_max * self.d_t, -self.s_max, self.s_max))

        elif theta_next_midle <= self.theta_log[-1] + rotate_radian / n:
            self.generate_path_by_delta_t(self.d_t, acc, clip(self.s_log[-1], -self.s_max, self.s_max))

        else:
            self.generate_path_by_delta_t(self.d_t, acc, clip(self.s_log[-1] - self.sv_max * self.d_t, -self.s_max, self.s_max))

        return self.generate_path_by_rotate(acc, rotate_radian - (self.theta_log[-1] - self.theta_log[-2]))


    def generate_path_until_time(self, T, acc, stear):
        while self.latest_time() <= T:
            self.generate_path_by_delta_t(self.d_t, acc, stear)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Kinematic Model')
    parser.add_argument('-vi', '--v_init', type=float, default=0.0)
    parser.add_argument('-ai', '--a_init', type=float, default=0.0)
    parser.add_argument('-si', '--s_init', type=float, default=0.0)
    parser.add_argument('-ti', '--t_init', type=float, default=0.0)
    parser.add_argument('-vmax', '--v_max', type=float, default=int(200 / 3600.0 * 1000.0))
    parser.add_argument('-smax', '--stear_max', type=float, default=45.0)
    # parser.add_argument('-accmax', '--acc_max', type=float, default=2.6)
    # parser.add_argument('-dccmax', '--dcc_max', type=float, default=4.5)
    parser.add_argument('-eth', '--error_th', type=float, default=0.2)
    parser.add_argument('--end', type=float, default=10.0)
    parser.add_argument('--dt', type=float, default=0.1)
    parser.add_argument('--d_dt', type=float, default=0.1)
    parser.add_argument('--np', type=int, default=40)
    parser.add_argument('-aui', '--a_update_interval', type=float, default=100)
    parser.add_argument('-sui', '--stear_update_interval', type=float, default=100)
    parser.add_argument('-rui', '--rotate_update_interval', type=float, default=100)
    parser.add_argument('-rv', '--rotate_value', type=float, default=0)
    parser.add_argument('-rt', '--rotate_type', action='store_true')
    parser.add_argument('-dim', type=int, default=20)
    parser.add_argument('-maxdim', type=int, default=10)


    args = parser.parse_args()    # 4. 引数を解析

    car = Car(
        x_init=0,
        y_init=0,
        v_init=args.v_init,
        s_init=math.radians(args.s_init),
        theta_init=math.radians(args.t_init),
        d_t=args.dt,
        v_max = args.v_max,
        s_max = math.radians(args.stear_max)
    )

    a = args.a_init
    stear = math.radians(args.s_init)
    a_update_time = car.latest_time() + args.a_update_interval
    stear_update_time = car.latest_time() + args.stear_update_interval
    rotate_update_time = car.latest_time() + args.rotate_update_interval
    rotate_type = 1
    while car.latest_time() <= args.np * args.dt:
        # ----- generate acc randomly -----
        if a_update_time <= car.latest_time() and args.a_update_interval != 0:
            a_update_time = car.latest_time() + args.a_update_interval
            a = random.uniform(-args.dcc_max, args.acc_max)

        # ----- generate stear randomly -----
        if stear_update_time <= car.latest_time() and args.stear_update_interval != 0:
            stear_update_time = car.latest_time() + args.stear_update_interval
            stear = math.radians(random.uniform(-args.stear_max, args.stear_max))

        # ----- generate stear randomly -----
        if rotate_update_time <= car.latest_time() and args.rotate_update_interval != 0 and args.rotate_value != 0:
            rotate_radian = math.radians(rotate_type * args.rotate_value)
            car.generate_path_by_rotate(0, rotate_radian)
            # rotate_update_time = car.latest_time() + args.rotate_update_interval
            rotate_update_time = max([car.latest_time(), rotate_update_time + args.rotate_update_interval])

            if args.rotate_type == True:
                rotate_type = -1 * rotate_type

            # print(f"time: {car.latest_time()}, type: rotate, next_time: {rotate_update_time}")

        else:
            car.generate_path_by_delta_t(args.dt, a, stear)

            # print(f"time: {car.latest_time()}, type: strait, next_time: {rotate_update_time}")



    x, y, t = car.path_by_point_num(args.np, int(args.d_dt / args.dt))
    # print(x)
    # print(y)
    # print(t)

    # _, diff_x_max, diff_y_max, diff_max, ans_x, ans_y = point2func(t, x, y, args.dim)
    _, diff_x_max, ans_x = point2func(t, x, args.dim)
    _, diff_y_max, ans_y = point2func(t, y, args.dim)

    d_x = np.array(x) - np.array(ans_x)
    d_y = np.array(y) - np.array(ans_y)
    diff_max = max([math.sqrt(d_x[i] * d_x[i] + d_y[i] * d_y[i]) for i in range(0, len(d_x))])

    # print('{:.20f}, {:.20f}, {:}'.format(diff_x_max, diff_y_max, args.__dict__))
    # file_name = "_".join([f"{k}_{v}" for k, v in args.__dict__.items()])
    # print('{:.20f}, {:}'.format(diff_max, file_name))

    # fig = plt.figure()
    # plt.scatter(x, y, c="black", label="grand truth")
    # plt.scatter(ans_x, ans_y, c="orange", label="estimate")
    # plt.legend()
    # plt.xlabel("x (m)")
    # plt.ylabel("y (m)")
    # fig.savefig(f"path_img/{file_name}.png")
    #
    # fig = plt.figure()
    # plt.scatter(t, x, c="black", label="grand truth")
    # plt.scatter(t, ans_x, c="orange", label="estimate")
    # plt.legend()
    # plt.xlabel("t (sec)")
    # plt.ylabel("x (m)")
    # fig.savefig(f"path_img/{file_name}_x.png")
    #
    # fig = plt.figure()
    # plt.scatter(t, y, c="black", label="grand truth")
    # plt.scatter(t, ans_y, c="orange", label="estimate")
    # plt.legend()
    # plt.xlabel("t (sec)")
    # plt.ylabel("y (m)")
    # fig.savefig(f"path_img/{file_name}_y.png")

    file_name = "_".join([f"{k}_{v}" for k, v in args.__dict__.items()])
    start_time = time.perf_counter()
    res_x = multi_container(t[:], x[:], args.maxdim, math.sqrt(args.error_th/3))
    res_y = multi_container(t[:], y[:], args.maxdim, math.sqrt(args.error_th/3))
    res_z = multi_container(t[:], [0] * len(t), args.maxdim, math.sqrt(args.error_th/3))
    take_time = time.perf_counter() - start_time

    # print(res_x)
    # print(res_y)
    ans_x = sum([d["ans"] for d in res_x], [])
    start_point_x = np.array([0] + [len(d["ans"]) for d in res_x])
    start_point_x = list(np.cumsum(start_point_x))
    # print(start_point_x)
    start_time_x = [t[d] for d in start_point_x[:-1]]
    start_point_x = [ans_x[d] for d in start_point_x[:-1]]
    # print(start_time_x)
    # print(start_point_x)

    ans_y = sum([d["ans"] for d in res_y], [])
    start_point_y = np.array([0] + [len(d["ans"]) for d in res_y])
    start_point_y = list(np.cumsum(start_point_y))
    # print(start_point_y)
    start_time_y = [t[d] for d in start_point_y[:-1]]
    start_point_y = [ans_y[d] for d in start_point_y[:-1]]
    # print(start_time_y)
    # print(start_point_y)

    # print(len(x))
    # print(len(y))
    # print(len(ans_x))
    # print(len(ans_y))
    # print([d["efficient_dim"] for d in res_x])
    # print([d["efficient_dim"] for d in res_y])
    colors = ["orange", "green"]

    fig = plt.figure()
    plt.scatter(x, y, c="black", label="grand truth")
    plt.scatter(ans_x, ans_y, c="orange", label="estimate")
    plt.legend()
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    fig.savefig(f"path_img/{file_name}.pdf")

    fig = plt.figure()
    plt.scatter(t, x, c="black", label="grand truth")
    plt.scatter(t, ans_x, c="orange", label="estimate")
    plt.scatter(start_time_x, start_point_x, c="green", label="start point of polynomial")
    plt.legend()
    plt.xlabel("t (sec)")
    plt.ylabel("x (m)")
    fig.savefig(f"path_img/{file_name}_x.pdf")

    fig = plt.figure()
    plt.scatter(t, y, c="black", label="grand truth")
    plt.scatter(t, ans_y, c="orange", label="estimate")
    plt.scatter(start_time_y, start_point_y, c="green", label="start point of polynomial")
    plt.legend()
    plt.xlabel("t (sec)")
    plt.ylabel("y (m)")
    fig.savefig(f"path_img/{file_name}_y.pdf")

    etsi_x, etsi_y, etsi_t = car.path_filterd_by_ETSI()

    dim_dict = {
        "x": [d["efficient_dim"] for d in res_x],
        "y": [d["efficient_dim"] for d in res_y]
    }
    x_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_x])
    y_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_y])
    z_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_z])
    # z_size = 2 + 2 + 4
    size = 14 + x_size + y_size + z_size
    # print(t)
    print('{:}, {:}, {:}, {:}'.format(size, 9 * len(etsi_x), take_time, file_name))
