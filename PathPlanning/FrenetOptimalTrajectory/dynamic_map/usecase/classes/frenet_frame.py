import sys
import os
import pprint

import copy

from .util import *

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../../")
from frenet_optimal_trajectory import (
    frenet_optimal_planning,
    generate_target_course,
    QuarticPolynomial,
    FrenetPath
)

from .poly import Poly

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "../..//../../../QuinticPolynomialsPlanner/")
from quintic_polynomials_planner import QuinticPolynomial

class FrenetOptimalTrajectry:
    # cite from: https://github.com/AtsushiSakai/PythonRobotics

    def __init__(self, waypoints, current_speed, current_accel, target_times=None, max_tt=5, min_tt=4, d_tt=0.1):
        # # print(waypoints)
        self.waypoints = waypoints
        self.waypoints_x = [d[0] for d in waypoints]
        self.waypoints_y = [d[1] for d in waypoints]
        # print(f"waypoints_y: {self.waypoints_y}")
        self.csp = self.__csp(self.waypoints_x, self.waypoints_y)
        # self.road_width_list = road_width_list

        # if target_times is not None:
        #     self.target_times = target_times
        # else:
        #     self.target_times = [None for _ in self.csp.s]
        # assert(len(self.csp.s) == len(self.target_times))

        # self.max_tt = max_tt
        # self.min_tt = min_tt
        # self.d_tt = d_tt


        self.c_d = 0.0  # current lateral position [m]
        self.c_d_d = 0.0  # current lateral speed [m/s]
        self.c_d_dd = 0.0  # current lateral acceleration [m/s]
        self.c_s = 0.0  # current course position
        self.c_s_d = current_speed
        self.c_s_dd = current_accel

        # cost weights
        self.K_J = 0.1
        self.K_T = 0.1
        self.K_D = 1.0
        self.K_LAT = 1.0
        self.K_LON = 1.0

    def diff_s_from_position(self, position):
        s, d = self.sd_from_position(position)

        return s - self.s, d

    def sd_from_position(self, position):
        d_min = float('inf')
        s = -1

        for i in range(0, len(self.waypoints_x) - 1):
            s1, s2 = self.csp.s[i], self.csp.s[i+1]
            p1, p2 = [self.waypoints_x[i], self.waypoints_y[i]], [self.waypoints_x[i+1], self.waypoints_y[i+1]]
            a1, b1 = liner_coefficients_by_2_points(p1, p2)
            a2, b2 = 1, -position[0] + position[1]

            # print(self.waypoints)
            # print(f"{sys._getframe().f_code.co_name}, a1: {a1}, b1: {b1}, a2: {a2}, b2: {b2}, s1: {s1}, s2: {s2}")
            cross_p = cross_point(a1, b1, a2, b2)

            x = cross_p[0]
            y = cross_p[1]

            d = math.fabs(x - position[0])
            w = ((x - p1[0]) / (p2[0] - p1[0]))
            s_tmp = (1 - w) * s1 + w * s2
            # assert( math.fabs(x - position[0]) == math.fabs(y - position[1]) )

            if rad_by_points(self.waypoints[i], self.waypoints[i+1]) < 0:
                d = -d

            # print(f"{sys._getframe().f_code.co_name}, d: {d}, s_tmp: {s_tmp}, cross_point: {cross_p}, position: {position}")
            if math.fabs(d) < d_min and s1 <= s_tmp <= s2:
                d_min = math.fabs(d)
                s = s_tmp

                assert(s1 <= s)

        return s, d_min

    def frenet_frame_paths(self, csp, s0, c_speed, c_d, c_d_d, c_d_dd):
        fplist = self.calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
        fplist = self.calc_global_paths(fplist, csp)

        return fplist


    def calc_frenet_paths(self, c_speed, c_d, c_d_d, c_d_dd, s0):
        pass

    def check_paths(self, fplist, ob, max_speed, max_accel, max_curv):
        ok_ind = []

        for i, _ in enumerate(fplist):
            if any([v > (max_speed + 10.0/3.6) for v in fplist[i].s_d]):  # Max speed check
                continue
            elif any([abs(a) > max_accel for a in
                      fplist[i].s_dd]):  # Max accel check
                continue
            elif any([abs(c) > max_curv for c in
                      fplist[i].c]):  # Max curvature check
                continue
            # elif not self.check_collision(fplist[i], ob, shapes):
            #     continue

            ok_ind.append(i)

        return [fplist[i] for i in ok_ind]

    def check_collision(self, fp, obstacles, shapes):
        for pos in shapes:
            pass


    def calc_global_paths(self, fplist):
        csp = self.csp

        for fp in fplist:
            # print(f"\n ----- {sys._getframe().f_code.co_name}")
            # print(f"{sys._getframe().f_code.co_name}, fp.s: {fp.s}")
            # print(f"{sys._getframe().f_code.co_name}, fp.s_d: {fp.s_d}")
            # print(f"{sys._getframe().f_code.co_name}, fp.s_dd: {fp.s_dd}")
            for i in range(len(fp.s)):
                ix, iy = csp.calc_position(fp.s[i])
                if ix is None or math.isnan(ix):
                    print(f"{sys._getframe().f_code.co_name}, i: {i}, s: {fp.s[i]}")
                    break
                i_yaw = csp.calc_yaw(fp.s[i])
                di = fp.d[i]
                fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
                fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
                fp.x.append(fx)
                fp.y.append(fy)

            # calc yaw and ds
            # print(fp.x)
            if len(fp.x) <= 1:
                continue

            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(math.atan2(dy, dx))
                fp.ds.append(math.hypot(dx, dy))

            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])

            # calc curvature
            for i in range(len(fp.yaw) - 1):
                if fp.ds[i] == 0:
                    fp.c.append(0)
                else:
                    fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

            # print(fp.__dict__)
        return fplist


    def frenet_path(self, tt, td, ts, tv, ta):
        print(f"{sys._getframe().f_code.co_name}, tt: {tt}, td: {td}, ts: {ts}, tv: {tv}, ta: {ta}")
        fp = FrenetPath()
        fp.t = [t for t in np.arange(0.0, tt, 0.1)]
        lat_qp = QuinticPolynomial(self.c_d, self.c_d_d, self.c_d_dd, td, 0.0, 0.0, tt)
        fp.d = [lat_qp.calc_point(t) for t in fp.t]
        fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
        fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
        fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

        tfp = copy.deepcopy(fp)
        lon_qp = None
        if ts is None:
            lon_qp = QuarticPolynomial(self.c_s, self.c_s_d, self.c_s_dd, tv, ta, tt)
        else:
            lon_qp = QuinticPolynomial(self.c_s, self.c_s_d, self.c_s_dd, ts, tv, ta, tt)
        tfp.s = [lon_qp.calc_point(t) for t in fp.t]
        tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
        tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
        tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

        return self.tfp_with_cost(tfp, tt, tv)

    def tfp_with_cost(self, tfp, tt, tv):
        Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
        Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

        # square of diff from target speed
        ds = (tv - tfp.s_d[-1]) ** 2

        tfp.cd = self.K_J * Jp + self.K_T * tt + self.K_D * tfp.d[-1] ** 2
        tfp.cv = self.K_J * Js + self.K_T * tt + self.K_D * ds
        tfp.cf = self.K_LAT * tfp.cd + self.K_LON * tfp.cv

        return tfp

    def ACC(self, tt, tp, tv, ta, obstacles):
        ts, td = self.sd_from_position(t_p)

        path = self.frenet_optimal_planning(tt, td, ts, tv, ta, obstacles)

    def path(self, tt=None, ts=None, td=None, tv=None, ta=None, tt_limits=[0, 5, 10], td_limits=[0, 0], ta_limits=[-2, 2], obstacles=[]):
        tt_list = [tt] if tt is not None else np.linspace(tt_limits[0], tt_limits[1], int(tt_limits[3] - tt_limits[2]) + 1)
        ts_list = [ts]
        td_list = [td] if td is not None else np.linspace(td_limits[0], td_limits[1], int(td_limits[1] - td_limits[0]))
        ta_list = [ta] if ta is not None else np.linspace(ta_limits[0], ta_limits[1], 3)

        paths = []
        for tt in tt_list:
            for td in td_list:
                for ts in ts_list:
                    for ta in ta_list:
                        paths.append(self.frenet_path(tt, td, ts, tv, ta))

        paths = self.calc_global_paths(paths, self.csp)
        paths = self.check_paths(paths, obstacles)

        if 0 < len(paths):
            # find minimum cost path
            min_cost = float("inf")
            best_path = None
            for fp in fplist:
                if min_cost >= fp.cf:
                    min_cost = fp.cf
                    best_path = fp

            self.c_d = best_path.d[1]
            self.c_d_d = best_path.d_d[1]
            self.c_d_dd = best_path.d_dd[1]
            self.c_s = best_path.s[1]
            self.c_s_d = best_path.s_d_d[1]
            self.c_s_dd = best_path.s_d_dd[1]

            return best_path

        else:
            return None

    def __csp(self, waypoints_x, waypoints_y):
        tx, ty, _, _, csp = generate_target_course(waypoints_x, waypoints_y)

        return csp


class AccFrenetOptimalTrajectry(FrenetOptimalTrajectry):
    def tfp_with_cost(tfp, tt):
        pass


class FreeFrenetOptimalTrajectry(FrenetOptimalTrajectry):
    def tfp_with_cost(tfp, tt):
        pass
