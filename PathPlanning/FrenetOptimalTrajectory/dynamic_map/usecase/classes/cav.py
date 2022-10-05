from .util import *

from .sumo_dynamic_map import *
from .frenet_frame import *
from .obstacles import *


class CAV:
    def __init__(self, id, net_file_path, edges, c_speed, c_acc, road_width_list, state_update_interval, width, length, heading, default_speed_mode, default_lane_mode, default_tau):
        self.id = id
        self.dynamic_map = DynamicMap(net_file_path)
        self.path_planner = None
        self.cav_handler = ""
        self.cpm_handler = ""
        self.mcm_handler = ""
        self.obstacle_handler = ObstacleHandler()
        self.state_update_interval = state_update_interval
        self.path = None
        self.fp = None
        self.width = width
        self.length = length
        self.heading = heading
        self.default_speed_mode = default_speed_mode
        self.default_lane_mode = default_lane_mode
        self.default_tau = default_tau

    # def waypoints_by_edges(self, edges):
    #     return self.dynamic_map.waypoints_by_edges(edges)


    def add_obstacle(self, obstacle_obj):
        # # print(obstacle_obj.__dict__)
        if isinstance(obstacle_obj, CarObstacle):
            self.obstacle_handler.add_car_obstacle(obstacle_obj)

        else:
            self.obstacle_handler.add_obstacle(obstacle_obj)

    def clear_obstacle(self):
        self.obstacle_handler.clear_obstacle()

    def obstacles(self, t=None):
        return self.obstacle_handler.positions(t)

    def will_be_collided(self, target_t):
        # return False

        # # # print(self.path)
        self_carob = CarObstacle(
            id=self.id,
            t=target_t,
            width=self.width,
            length=self.length,
            path=self.path
        )

        for ob in self.obstacle_handler.all_obstacles():
            print(f"id: {self.id}, ob_id: {ob.id}, target_t: {target_t}, max_time: {self.path.max_time()}")
            # if any([self_carob.check_collision(ob, t) for t in np.linspace(target_t, self.path.max_time(), 10)]):
            if any([self_carob.check_collision(ob, target_t + t) for t in np.arange(1, 10, 1)]):
                print("collide.")
                return True

        return False

    # def __s_is_invalid(fp):
    #     return any([math.isnan(s) or s < 0 for s in fp.s])
    #
    # def __curv_is_invalid(fp, max_curv):
    #     return any([abs(c) > max_curv for c in fp.c])
    def __s_is_invalid(self, fp):
        for s in fp.s:
            if math.isnan(s) or s < 0:
                return True

        return False

    def __s_d_is_invalid(self, fp, max_speed):
        for v in fp.s_d:
            if v > (max_speed + 10.0/3.6) or v < 0:
                return True

        return False

    def __s_dd_is_invalid(self, fp, max_accel):
        for a in fp.s_dd:
            if abs(a) > max_accel:
                return True

        return False

    def __will_be_collied(self, fp, now):
        # return False

        L = min([len(fp.t), len(fp.x), len(fp.y)])
        if L <= 0:
            # print("short L")
            return True

        self_carob = CarObstacle(
            id=self.id,
            t=now,
            width=self.width,
            length=self.length,
            path=Path([[now + fp.t[j], fp.x[j], fp.y[j], 0] for j in range(0, L)])
        )

        # print("-- collision check --")
        for ob in self.obstacle_handler.all_obstacles():
            print(f"ob_id: {ob.id}")
            # for t in fp.t[::10]:
            # for t in fp.t:
            for t in np.arange(1, 10, 1):
                if self_carob.check_collision(ob, t + now):
                    # print("collide")
                    return True

        return False

    def fund_check_path(self, fplist, now, max_speed, max_accel, max_curv):
        ok_ind = []

        for i, _ in enumerate(fplist):
            # # print(f"\n ----- {sys._getframe().f_code.co_name}")
            # if len(fplist[i].x) <= 2:
            #     # print("short path")
            #     # print(f"{sys._getframe().f_code.co_name}, fp.x: {fplist[i].x}")
            #     continue
            if self.__s_is_invalid(fplist[i]):
                print("invalid s.")
                print(f"{sys._getframe().f_code.co_name}, fp.s: {fplist[i].s}")
                continue
            elif self.__s_d_is_invalid(fplist[i], max_speed):  # Max speed check
                print("speed over.")
                print(f"{sys._getframe().f_code.co_name}, fp.s_d: {fplist[i].s_d}")
                continue
            elif self.__s_dd_is_invalid(fplist[i], max_accel):
                # print(max_accel)
                # # print([a for a in fplist[i].s_dd])
                print("accel over.")
                print(f"{sys._getframe().f_code.co_name}, fp.s_dd: {fplist[i].s_dd}")
                continue
            # elif any([abs(c) > max_curv for c in fplist[i].c]):  # Max curvature check
            #     # # print("curve over.")
            #     continue

            ok_ind.append(i)

        return [fplist[i] for i in ok_ind]

    def check_path(self, fplist, now, max_speed, max_accel, max_curv):
        ok_ind = []

        for i, _ in enumerate(fplist):
            if len(fplist[i].x) <= 2:
                # print("short path")
                # print(f"{sys._getframe().f_code.co_name}, fp.x: {fplist[i].x}")
                continue

            # ----- Since we cannot handle some curve in normal SUMO netfile, we comment out this validation -----
            # ----- This comment out does not affect on the essense of any results. -----
            # elif any([abs(c) > max_curv for c in fplist[i].c]):  # Max curvature check
            #     # print(fplist[i].s_d)
            #     # print(fplist[i].s_dd)
            #     # print(fplist[i].x)
            #     # print(fplist[i].y)
            #     # print(fplist[i].c)
            #     # print(fplist[i].yaw)
            #     # print([abs(c) > max_curv for c in fplist[i].c])
            #     # print(any([abs(c) > max_curv for c in fplist[i].c]))
            #     # print("curve over.")
            #     continue

            # ----- collision check -----
            # """
            # print(f"t size: {len(fplist[i].t)}, x size: {len(fplist[i].x)}, y size: {len(fplist[i].y)}")
            if self.__will_be_collied(fplist[i], now):
                continue

            # """
            ok_ind.append(i)

        return [fplist[i] for i in ok_ind]

    def state(self, t):
        if self.path is None:
            print(f"id: {self.id}, path is None")
            return None, None, None, None, None

        # if self.will_be_collided(t):
        #     return None, None, None, None, None

        return self.path.state(t)

    def __acc_path(self, waypoints, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv):
        fplist = []

        following_vehs = []
        max_score = 0
        for ob in self.obstacle_handler.car_obstacles():
            v1 = np.array(ob.position[0:2]) - np.array(waypoints[0])
            v2 = np.array([math.cos(math.radians(self.heading)), math.sin(math.radians(self.heading))])
            # # print(f"ob_id: {ob.id}")
            # # print(f"v1: {v1}")
            # # print(f"v2: {v2}")

            if cos_sim(v1, v2) >= math.cos(math.radians(2.5)) and math.dist(v1, v2) <= 150:
                # following_vehs.append(ob)

                if max_score < cos_sim(v1, v2) / math.dist(v1, v2):
                    following_vehs = [ob]

        for acc_veh in following_vehs:
            ts, td = self.path_planner.sd_from_position(acc_veh.position)

            al = math.dist((0, 0, 0), tuple(acc_veh.accel))
            vl = math.dist((0, 0, 0), tuple(acc_veh.speed))
            pl = acc_veh.pos(now)

            # ta = ACC_accel(acc_veh.length, al, vl, pl, c_speed, waypoints[0] + tuple([0]), max_accel)

            for dT in np.arange(10, 11, 1):
                if ts is None:
                    continue

                for new_ts in np.linspace(ts, ts + vl * dT + al * (dT**2) /2 - ACC_l(c_speed, acc_veh.length), 10):
                    # print(f"----- follow id: {acc_veh.id}, ts: {ts}, speed: {vl}, accel: {al} -----")
                    # new_ts = ts + vl * dT + al * (dT**2) /2 - ACC_l(c_speed, acc_veh.length)
                    # new_ts = ts + vl * dT + al * (dT**2) /2 - 100
                    # tv = middle_val(vl + al * dT, 0, max_speed)
                    # print(f"----- follow id: {acc_veh.id}, ts: {ts}, new_ts: {new_ts}, speed: {vl}, accel: {al} -----")
                    #
                    # validate_v = (new_ts - ts) / dT
                    # validate_a = (vl - )
                    new_path = self.path_planner.frenet_path(
                        dT,
                        td,
                        new_ts,
                        vl,
                        al
                    )

                    fplist = fplist + self.fund_check_path([new_path], now, max_speed, max_accel, max_curv)


        return fplist


    def __free_flow_path(self, waypoints, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv):
        fplist = []

        # # print(road_width_list)
        # print("----- free flow -----")
        dT = max([(max_speed - c_speed) / max_accel, 10])
        tv = max_speed
        ta = 0

        # print(f"dT: {dT}")
        for td in np.linspace(road_width_list[1], road_width_list[0], max([1, int(road_width_list[0] - road_width_list[1])])):
            for tt in np.linspace(dT, 10, int(10 / dT)):
                print(f"----- id: {self.id}, free flow: tt: {tt}, speed: {tv}, accel: {ta}, dT: {dT} -----")
                new_path = self.path_planner.frenet_path(
                    tt,
                    td,
                    None,
                    tv,
                    ta
                )

                fplist = fplist + self.fund_check_path([new_path], now, max_speed, max_accel, max_curv)

        return fplist

    def __possible_path(self, waypoints, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv):
        fplist = []

        # # print(road_width_list)
        # print("----- possible path -----")
        # # print(f"dT: {dT}")
        ta = 0
        tt = 10
        for td in np.linspace(road_width_list[1], road_width_list[0], max([1, int(road_width_list[0] - road_width_list[1])])):
            # for ta in [-max_accel, -max_accel / 2, 0, max_accel / 2, max_accel]:
            for tv in np.linspace(0, max_speed, 5):
                ta = (tv - c_speed) / tt
                # print(f"----- possible path: tt: {tt}, speed: {tv}, accel: {ta} -----")
                new_path = self.path_planner.frenet_path(
                    tt,
                    td,
                    None,
                    tv,
                    ta
                )

                fplist = fplist + self.fund_check_path([new_path], now, max_speed, max_accel, max_curv)

        return fplist


    def __stop(self, waypoints, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv):
        fplist = []

        # # print(road_width_list)
        # print("----- possible path -----")
        # # print(f"dT: {dT}")
        ta = 0
        td = 0
        tv = 0
        for tt in np.linspace(10, 1, 10):
            # print(f"----- stop: tt: {tt}, speed: {tv}, accel: {ta} -----")
            new_path = self.path_planner.frenet_path(
                tt,
                td,
                None,
                tv,
                ta
            )

            fplist = fplist + self.fund_check_path([new_path], now, max_speed, max_accel, max_curv)

        return fplist

    def __set_path(self, fplist, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv):
        for fp in self.path_planner.calc_global_paths(sorted(fplist, key=lambda x: x.cf, reverse=True)):
            print(fp.__dict__)
            if 1 <= len(self.check_path([fp], now, max_speed, max_accel, max_curv)):
                best_path = fp

                L = min([len(best_path.t), len(best_path.x), len(best_path.y)])
                print("----- found best path -----")
                print([[now + best_path.t[j], best_path.x[j], best_path.y[j], 0] for j in range(0, L)])
                self.path = Path([[now + best_path.t[j], best_path.x[j], best_path.y[j], 0] for j in range(0, L)])

                return True

            else:
                continue

        print("----- unfound best path -----")
        return False

    def generate_new_path(self, waypoints, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv):
        self.path_planner = FrenetOptimalTrajectry(waypoints, c_speed, 0)
        # self.path_planner = FrenetOptimalTrajectry(waypoints, c_speed, c_acc)
        self.waypoints = waypoints
        print(f"id: {self.id}, waypoints: {waypoints}")

        # """
        # ----- free flow -----
        print("--- free flow")
        fplist = self.__free_flow_path(waypoints, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv)
        if self.__set_path(fplist, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv):
            return True

        # ----- ACC -----
        print("---- acc")
        fplist = self.__acc_path(waypoints, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv)
        if self.__set_path(fplist, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv):
            return True

        # ----- possible path -----
        # print("--- possible path")
        # fplist = self.__possible_path(waypoints, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv)
        # if self.__set_path(fplist, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv):
        #     return True

        # ----- stop -----
        print("--- stop")
        fplist = self.__stop(waypoints, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv)
        if self.__set_path(fplist, c_speed, c_acc, now, road_width_list, max_speed, max_accel, max_curv):
            return True

        # raise Exception(f"No path, id: {self.id}, waypoints: {waypoints}")
        self.path = None
        return False
        # """



    def leading_vehicle_state(self):
        min_diff_s = float('inf')

        for p in self.obstacle_handler.positions():
            diff_s, d = self.path_planner.diff_s_from_position(p)

            if math.fabs(d) <= 3.2 and 0 <= diff_s and diff_s < min_diff_s:
                pass
