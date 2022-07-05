from .util import *

from .sumo_dynamic_map import *
from .frenet_frame import *
from .obstacles import *


class CAV:
    def __init__(self, id, net_file_path, edges, waypoints, c_speed, c_acc, road_width_list, state_update_interval, width, length, heading):
        self.id = id
        self.dynamic_map = DynamicMap(net_file_path)
        self.path_planner = FrenetOptimalTrajectry(waypoints, c_speed, c_acc, road_width_list)
        self.cav_handler = ""
        self.cpm_handler = ""
        self.mcm_handler = ""
        self.obstacle_handler = ObstacleHandler()
        self.state_update_interval = state_update_interval
        self.path = None
        self.width = width
        self.length = length
        self.heading = heading


    def waypoints_by_edges(self, edges):
        return self.dynamic_map.waypoints_by_edges(edges)


    def add_obstacle(self, obstacle_obj):
        # print(obstacle_obj.__dict__)
        if isinstance(obstacle_obj, CarObstacle):
            self.obstacle_handler.add_car_obstacle(obstacle_obj)

        else:
            self.obstacle_handler.add_obstacle(obstacle_obj)

    def clear_obstacle(self):
        self.obstacle_handler.clear_obstacle()

    def obstacles(self, t=None):
        return self.obstacle_handler.positions(t)

    def will_be_collided(self, target_t):
        # # print(self.path)
        self_carob = CarObstacle(
            t=target_t,
            width=self.width,
            length=self.length,
            path=self.path
        )

        for t in np.arange(target_t, self.path.max_time(), 1):
            for ob in self.obstacle_handler.all_obstacles():
                if self_carob.check_collision(ob, t):
                    return True

                else:
                    continue

        return False

    def __s_is_invalid(fp):
        return any([math.isnan(s) or s < 0 for s in fp.s])

    def __curv_is_invalid(fp, max_curv):
        return any([abs(c) > max_curv for c in fp.c])

    def fund_check_path(self, fplist, now, max_speed, max_accel, max_curv):
        ok_ind = []

        for i, _ in enumerate(fplist):
            # print(f"\n ----- {sys._getframe().f_code.co_name}")
            # if len(fplist[i].x) <= 2:
            #     print("short path")
            #     print(f"{sys._getframe().f_code.co_name}, fp.x: {fplist[i].x}")
            #     continue
            if any([math.isnan(s) or s < 0 for s in fplist[i].s]):
                print("invalid s.")
                print(f"{sys._getframe().f_code.co_name}, fp.s: {fplist[i].s}")
                continue
            elif any([v > (max_speed + 10.0/3.6) or v < 0 for v in fplist[i].s_d]):  # Max speed check
                print("speed over.")
                print(f"{sys._getframe().f_code.co_name}, fp.s_d: {fplist[i].s_d}")
                continue
            elif any([abs(a) > max_accel for a in fplist[i].s_dd]):  # Max accel check
                # print(max_accel)
                # # print([a for a in fplist[i].s_dd])
                print("accel over.")
                print(f"{sys._getframe().f_code.co_name}, fp.s_dd: {fplist[i].s_dd}")
                continue
            # elif any([abs(c) > max_curv for c in fplist[i].c]):  # Max curvature check
            #     # print("curve over.")
            #     continue

            ok_ind.append(i)

        return [fplist[i] for i in ok_ind]

    def check_path(self, fplist, now, max_speed, max_accel, max_curv):
        ok_ind = []

        for i, _ in enumerate(fplist):
            if len(fplist[i].x) <= 2:
                print("short path")
                print(f"{sys._getframe().f_code.co_name}, fp.x: {fplist[i].x}")
                continue

            # ----- Since we cannot handle some curve in normal SUMO netfile, we comment out this validation -----
            # ----- This comment out does not affect on the essense of any results. -----
            # elif any([abs(c) > max_curv for c in fplist[i].c]):  # Max curvature check
            #     print(fplist[i].s_d)
            #     print(fplist[i].s_dd)
            #     print(fplist[i].x)
            #     print(fplist[i].y)
            #     print(fplist[i].c)
            #     print(fplist[i].yaw)
            #     print([abs(c) > max_curv for c in fplist[i].c])
            #     print(any([abs(c) > max_curv for c in fplist[i].c]))
            #     print("curve over.")
            #     continue

            # ----- collision check -----
            """
            print(f"t size: {len(fplist[i].t)}, x size: {len(fplist[i].x)}, y size: {len(fplist[i].y)}")
            L = min([len(fplist[i].t), len(fplist[i].x), len(fplist[i].y)])
            if L <= 0:
                print("short L")
                continue

            self_carob = CarObstacle(
                t=now,
                width=self.width,
                length=self.length,
                path=Path([[now + fplist[i].t[j], fplist[i].x[j], fplist[i].y[j], 0] for j in range(0, L)])
            )

            for ob in self.obstacle_handler.all_obstacles():
                for t in fplist[i].t:
                    tt = t + now

                    if self_carob.check_collision(ob, tt):
                        print("collide.")
                        continue
            # """
            ok_ind.append(i)

        return [fplist[i] for i in ok_ind]

    def state(self, t):
        if self.path is None:
            return None, None, None, None, None

        if self.will_be_collided(t):
            return None, None, None, None, None

        return self.path.state(t)

    def generate_new_path(self, waypoints, c_speed, c_acc, now, max_speed, max_accel, max_curv):
        self.path_planner = FrenetOptimalTrajectry(waypoints, c_speed, c_acc)

        fplist = []
        # ----- ACC -----

        following_vehs = []
        max_score = 0
        for ob in self.obstacle_handler.car_obstacles():
            v1 = np.array(ob.position[0:2]) - np.array(waypoints[0])
            v2 = np.array([math.cos(math.radians(self.heading)), math.sin(math.radians(self.heading))])
            # print(f"ob_id: {ob.id}")
            # print(f"v1: {v1}")
            # print(f"v2: {v2}")

            if cos_sim(v1, v2) >= math.cos(math.radians(2.5)) and math.dist(v1, v2) <= 150:
                # following_vehs.append(ob)

                if max_score < cos_sim(v1, v2) / math.dist(v1, v2):
                    following_vehs = [ob]

        print(f"waypoints: {waypoints}")
        # print(np.array(waypoints[0]))
        # print(self.heading)
        # print(math.cos(math.radians(30)))
        # print(f"id: {self.id}, following_vehs: {[ob.id for ob in following_vehs]}")

        # ----- acc -----
        if 1 <= len(following_vehs):
            for acc_veh in following_vehs:
                ts, td = self.path_planner.sd_from_position(acc_veh.position)

                al = math.dist((0, 0, 0), tuple(acc_veh.accel))
                vl = math.dist((0, 0, 0), tuple(acc_veh.speed))
                pl = acc_veh.pos(now)

                ta = ACC_accel(acc_veh.length, al, vl, pl, c_speed, waypoints[0] + tuple([0]), max_accel)
                for dT in range(1, 11):
                    if ts is None:
                        continue

                    ts = ts + vl * dT + al * (dT**2) /2 - ACC_l(c_speed, acc_veh.length)
                    tv = middle_val(vl + al * dT, 0, max_speed)
                    print(f"----- follow id: {acc_veh.id}, speed: {vl}, accel: {ta} -----")
                    new_path = self.path_planner.frenet_path(
                        dT,
                        td,
                        ts,
                        tv,
                        al
                    )

                    fplist = fplist + self.fund_check_path([new_path], now, max_speed, max_accel, max_curv)

        if len(fplist) <= 0:
            # ----- free flow -----
            # # print(max_speed)
            # # print(c_speed)

            # print(tv)
            # print(ta)
            print("----- free flow -----")
            dT = (max_speed - c_speed) / max_accel
            tv = max_speed
            ta = 0
            if dT <= 5:
                dT = 5

            print(f"dT: {dT}")
            for tt in np.linspace(dT, 2 * dT, 5):
                new_path = self.path_planner.frenet_path(
                    tt,
                    0,
                    None,
                    tv,
                    ta
                )

                fplist = fplist + self.fund_check_path([new_path], now, max_speed, max_accel, max_curv)



        # print(len(fplist))
        # fplist = self.fund_check_path(fplist, now, max_speed, max_accel, max_curv)
        fplist = self.path_planner.calc_global_paths(fplist)
        # fplist
        fplist = self.check_path(fplist, now, max_speed, max_accel, max_curv)
        # fplist = self.check_path(fplist, now, max_speed, max_accel, max_curv)

        best_path = None
        if 0 < len(fplist):
            # find minimum cost path
            min_cost = float("inf")
            best_path = None
            for fp in fplist:
                if min_cost >= fp.cf:
                    min_cost = fp.cf
                    best_path = fp

        if best_path is not None:
            print("--- best path ---")
            print(f"{sys._getframe().f_code.co_name}, fp.s: {best_path.s}")
            print(f"{sys._getframe().f_code.co_name}, fp.s_d: {best_path.s_d}")
            print(f"{sys._getframe().f_code.co_name}, fp.s_dd: {best_path.s_dd}")
            # # print(now)
            # # print([[now + best_path.t[j], best_path.x[j], best_path.y[j], 0] for j in range(0, len(best_path.t))])
            L = min([len(best_path.t), len(best_path.x), len(best_path.y)])
            self.path = Path([[now + best_path.t[j], best_path.x[j], best_path.y[j], 0] for j in range(0, L)])

        else:
            pass
            # self.path = None


    def leading_vehicle_state(self):
        min_diff_s = float('inf')

        for p in self.obstacle_handler.positions():
            diff_s, d = self.path_planner.diff_s_from_position(p)

            if math.fabs(d) <= 3.2 and 0 <= diff_s and diff_s < min_diff_s:
                pass
