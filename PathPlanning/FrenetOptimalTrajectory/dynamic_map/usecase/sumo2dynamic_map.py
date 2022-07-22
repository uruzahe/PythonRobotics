import argparse
import os, sys
import numpy as np
import math
import warnings
import pprint

from functools import lru_cache
from xml.etree import ElementTree
from collections import OrderedDict


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../")
from frenet_optimal_trajectory import (
    frenet_optimal_planning,
    generate_target_course,
    QuarticPolynomial
)

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../../QuinticPolynomialsPlanner/")
from quintic_polynomials_planner import QuinticPolynomial
import cubic_spline_planner

from classes.util import *
from classes.sumo_dynamic_map import *
from classes.cav import *
from classes.mylogger import *

def optimal_waypoints(dynamic_map, edges, current_route_index, position):
    optimal_cost = float('inf')
    optimal_waypoints = None

    waypoints_list = dynamic_map.waypoints_list(edges[current_route_index:])
    for wp in waypoints_list:
        # print(wp)
        _, left_shapes = divide_waypoints_by_point( wp, position )
        waypoints = [tuple(position)] + left_shapes
        waypoints = [waypoints[i] for i in range(0, len(waypoints) - 1) if math.dist(waypoints[i], waypoints[i + 1]) != 0] + [waypoints[-1]]

        rads = [ rad_by_points( np.array(waypoints[i + 1]) - np.array(waypoints[i]), np.array([1, 0]) ) for i in range(0, len(waypoints) - 1) ]
        cost = np.sum(np.diff(np.array(rads)))

        if cost < optimal_cost:
            optimal_waypoints = waypoints

    assert(optimal_waypoints is not None)
    return optimal_waypoints

if __name__ == "__main__":
    mylogger = MyLogger(
        logger_name='sumo-log',
        logger_file_path='./sumo-log.txt',
        format='%(asctime)s - %(levelname)s - %(message)s'
    )

    parser = argparse.ArgumentParser(description='Dynamic Map.')
    # parser.add_argument('-nfp', '--net_file_path')
    parser.add_argument('-cfp', '--sumo_cfg_file_path', required=True)
    parser.add_argument('-st', '--start_time', type=float, default=0)
    parser.add_argument('-et', '--end_time', type=float, default=24 * 60 * 60)
    parser.add_argument('-maxs','--max_speed', type=int, default=60.0 * 1000.0 / 3600.0)
    parser.add_argument('-gui', action='store_true', default=False)
    parser.add_argument('--step', type=float, default=0.1)
    parser.add_argument('--seed', type=float, default=1)
    parser.add_argument('-ies', '--inflow_edges', nargs='*')
    parser.add_argument('-irs', '--inflow_rates', nargs='*')
    parser.add_argument('-camrs', '--cam_rates', nargs='*')
    parser.add_argument('-cpmrs', '--cpm_rates', nargs='*')
    parser.add_argument('-mcmrs', '--mcm_rates', nargs='*')
    parser.add_argument('-cavrs', '--cav_rates', nargs='*')
    parser.add_argument('-senrs', '--sensor_rates', nargs='*')
    parser.add_argument('--data_dir', default="./data/")
    args = parser.parse_args()

    sumocfg_elems = parse_xmlobj_of_element_tree(ElementTree.parse(args.sumo_cfg_file_path).getroot())
    net_file_path = ""
    for d in sumocfg_elems["input"]:
        for k, v in d.items():
            if k == "net-file":
                net_file_path = v[0]["value"]
    assert net_file_path != ""

    dynamic_map = DynamicMap(net_file_path)
    print("\n----- all data -----\n")
    pprint.pprint(dynamic_map.data)
    print("\n----- edge2edge -----\n")
    pprint.pprint(dynamic_map.edge2edge)
    print("\n----- edge2lanes -----\n")
    pprint.pprint(dynamic_map.edge2lanes)
    sumoBinary = "sumo"
    if args.gui:
        sumoBinary = "sumo-gui"
    traci.start(f"{sumoBinary} -c {args.sumo_cfg_file_path} --step-length {args.step} --seed {args.seed}".split(" "))

    id2cavs = {}
    veh_ids = []
    while True:
        traci.simulationStep()
        NOW = float(traci.simulation.getTime())
        print(f"{sys._getframe().f_code.co_name}, ----- {NOW} -----\n")

        del_ids = traci.simulation.getArrivedIDList()
        add_ids = traci.simulation.getDepartedIDList()

        for id in traci.simulation.getArrivedIDList():
            if id in id2cavs:
                id2cavs.pop(id)

        veh_ids = list(set(veh_ids) | set(add_ids) - set(del_ids))

        for id in veh_ids:
            print(f"--- id: {id} ---")
            try:
                position = list(traci.vehicle.getPosition(id))
                current_route_index = traci.vehicle.getRouteIndex(id)
                edges = traci.vehicle.getRoute(id)
                angle = -traci.vehicle.getAngle(id) + 90
                speed = traci.vehicle.getSpeed(id)
                accel = traci.vehicle.getAcceleration(id)
                max_speed = traci.vehicle.getMaxSpeed(id)

            except traci.exceptions.TraCIException:
                veh_ids.remove(id)
                continue

            # ----- add new cav -----
            if id not in id2cavs:
                id2cavs[id] = CAV(
                    id,
                    net_file_path,
                    edges,
                    traci.vehicle.getSpeed(id),
                    traci.vehicle.getAcceleration(id),
                    [dynamic_map.road_width(edges[e_i], list(traci.vehicle.getPosition(id))) for e_i in range(0, len(edges))],
                    traci.simulation.getDeltaT(),
                    width=traci.vehicle.getWidth(id),
                    length=traci.vehicle.getLength(id),
                    heading=angle,
                    default_speed_mode=traci.vehicle.getSpeedMode(id),
                    default_lane_mode=traci.vehicle.getLaneChangeMode(id)
                )
                # ----- tips: initial setting -----
                traci.vehicle.setSpeedMode(id, 0)
                traci.vehicle.setLaneChangeMode(id, 0)

            cav = id2cavs[id]
            cav.clear_obstacle()
            # ----- for obstacles -----
            for ob_id, _ in id2cavs.items():
                ob_pos = traci.vehicle.getPosition(ob_id)
                pos_vec = np.array(list(ob_pos)) - np.array(position)
                pos_dist = math.dist(ob_pos, position)
                angle_vec = np.array([math.cos(math.radians(angle)), math.sin(math.radians(angle))])
                if cav.id is not ob_id and -1/2 <= cos_sim(pos_vec, angle_vec) and pos_dist <= 150:
                    ob_angle = -traci.vehicle.getAngle(ob_id) + 90
                    ob_speed = traci.vehicle.getSpeed(ob_id)
                    ob_accel = traci.vehicle.getAcceleration(ob_id)
                    mylogger.debug(f"time: {NOW}, id: {id}, ob_id: {ob_id}, ob_angle: {ob_angle}, ob_speed: {ob_speed}, ob_accel: {ob_accel}")
                    cav.add_obstacle(CarObstacle(
                        id=ob_id,
                        t = NOW,
                        heading=ob_angle,
                        width=traci.vehicle.getWidth(ob_id),
                        length=traci.vehicle.getLength(ob_id),
                        position=list(traci.vehicle.getPosition(ob_id)) + [0],
                        speed=[ob_speed * math.cos(math.radians(ob_angle)), ob_speed * math.sin(math.radians(ob_angle)), 0],
                        accel=[ob_accel * math.cos(math.radians(ob_angle)), ob_accel * math.sin(math.radians(ob_angle)), 0],
                    ))

            # mylogger.debug(f"time: {NOW}, id: {id}, neighber: {traci.vehicle.getNeighbors(id, 7)}, follower: "),
            # neighbers = traci.vehicle.getNeighbors(id, 7)
            # leader = traci.vehicle.getLeader(id, 150)
            # mylogger.debug(f"time: {NOW}, id: {id}, neighber: {neighbers}, leader: {leader}"),
            # obstacle_cars = [d[0] for d in traci.vehicle.getNeighbors(id, 7)]
            # if leader is not None:
            #     obstacle_cars.append(leader[0])
            # mylogger.debug(f"time: {NOW}, id: {id}, neighber: {obstacle_cars}"),
            # for ob_id in obstacle_cars:
            #     # pos_vec = np.array(list(traci.vehicle.getPosition(ob_id))) - np.array(position)
            #     # angle_vec = np.array([math.cos(math.radians(angle)), math.sin(math.radians(angle))])
            #     ob_angle = -traci.vehicle.getAngle(ob_id) + 90
            #     ob_speed = traci.vehicle.getSpeed(ob_id)
            #     ob_accel = traci.vehicle.getAcceleration(ob_id)
            #
            #     cav.add_obstacle(CarObstacle(
            #         id=ob_id,
            #         t = NOW,
            #         heading=ob_angle,
            #         width=traci.vehicle.getWidth(ob_id),
            #         length=traci.vehicle.getLength(ob_id),
            #         position=list(traci.vehicle.getPosition(ob_id)) + [0],
            #         speed=[ob_speed * math.cos(math.radians(ob_angle)), ob_speed * math.sin(math.radians(ob_angle)), 0],
            #         accel=[ob_accel * math.cos(math.radians(ob_angle)), ob_accel * math.sin(math.radians(ob_angle)), 0],
            #     ))

            # """
            # ----- update vehicle state -----
            angle, p, v, a, progress = cav.state(NOW + 0.1)
            max_speed = max([
                speed,
                min([
                    max_speed,
                    dynamic_map.max_speed(traci.vehicle.getRoute(id)[traci.vehicle.getRouteIndex(id)], traci.vehicle.getLaneID(id), max_speed
                )])
            ])
            max_accel = max([math.fabs(accel), traci.vehicle.getAccel(id)])
            if p is None or 0.5 <= progress:
                mylogger.debug(f"time: {NOW}, id: {id}, waypoints: {optimal_waypoints(dynamic_map, edges, current_route_index, position)}")
                cav.generate_new_path(
                    optimal_waypoints(dynamic_map, edges, current_route_index, position),
                    speed,
                    accel,
                    NOW,
                    dynamic_map.road_width(edges[current_route_index], position),
                    max_speed,
                    max_accel,
                    2
                )
                angle, p, v, a, progress = cav.state(NOW + 0.1)

            # # print(traci.vehicle.getPosition(id))

            print(f"{sys._getframe().f_code.co_name}, id: {id}, angle: {angle}, pos: {p}, speed: {v}, accel: {a}, progress: {progress}, lane: {traci.vehicle.getLaneID(id)}")
            print(f"{sys._getframe().f_code.co_name}, current_pos: {traci.vehicle.getPosition(id)}")

            # # print(cav.path.all_paths())
            try:
                if p is not None:
                    # print(f"dist: {math.dist(position, cav.waypoints[-1])}")
                    if 20 <= math.dist(position, cav.waypoints[-1]):
                        # keep_route = 1
                        mylogger.debug(f"time: {NOW}, id: {id}, could change: {traci.vehicle.couldChangeLane(id, -1)}, {traci.vehicle.couldChangeLane(id, 1)}, state: {traci.vehicle.getLaneChangeStatePretty(id, -1)}, {traci.vehicle.getLaneChangeStatePretty(id, 1)}, {traci.vehicle.wantsAndCouldChangeLane(id, -1, state=None)}, {traci.vehicle.wantsAndCouldChangeLane(id, 1, state=None)}")
                        lane_change_right_state = traci.vehicle.getLaneChangeStatePretty(id, -1)[1]
                        if 'right' in lane_change_right_state and 'stay' not in lane_change_right_state:
                            traci.vehicle.changeLaneRelative(id, -1, 1)
                            # keep_route = 2

                        lane_change_left_state = traci.vehicle.getLaneChangeStatePretty(id, 1)[1]
                        if 'left' in lane_change_left_state and 'stay' not in lane_change_left_state:
                            traci.vehicle.changeLaneRelative(id, 1, 1)
                            # keep_route = 2
                        # ----- position change -----
                        traci.vehicle.moveToXY(
                            id,
                            traci.vehicle.getRoute(id)[traci.vehicle.getRouteIndex(id)],
                            # int(traci.vehicle.getLaneID(id).split("_")[1]),
                            -1,
                            p[0],
                            p[1],
                            # matchThreshold=1,
                            # keepRoute=4
                            # math.degrees(angle) + 90
                        )
                        traci.vehicle.setSpeed(id, math.dist(v, [0,0,0]))

                        # ----- lane change -----
                        # signal = traci.vehicle.getSignals(id)
                        #
                        # lane_change_right_state = traci.vehicle.getLaneChangeStatePretty(id, -1)[0]
                        # if 'right' in lane_change_right_state and 'stay' not in lane_change_right_state:
                        #     traci.vehicle.changeLaneRelative(id, -1, 1)
                        #
                        # lane_change_left_state = traci.vehicle.getLaneChangeStatePretty(id, 1)[0]
                        # if 'left' in lane_change_right_state and 'stay' not in lane_change_left_state:
                        #     traci.vehicle.changeLaneRelative(id, 1, 1)
                        #
                        # if signal == 0: # Right blinker
                        #     mylogger.debug(f"time: {NOW}, id: {id}, signal: {signal}, could change: {traci.vehicle.couldChangeLane(id, -1)}, {traci.vehicle.couldChangeLane(id, 1)}, state: {traci.vehicle.getLaneChangeStatePretty(id, -1)}, {traci.vehicle.getLaneChangeStatePretty(id, 1)}, {traci.vehicle.wantsAndCouldChangeLane(id, -1, state=None)}, {traci.vehicle.wantsAndCouldChangeLane(id, 1, state=None)}")
                        #     if traci.vehicle.couldChangeLane(id, -1):    # int = -1 is for the right lane change state
                        #         traci.vehicle.changeLaneRelative(id, -1, 0)
                        #
                        # elif signal == 1: # Left blinker
                        #     mylogger.debug(f"time: {NOW}, id: {id}, signal: {signal}, could change: {traci.vehicle.couldChangeLane(id, -1)}, {traci.vehicle.couldChangeLane(id, 1)}, state: {traci.vehicle.getLaneChangeStatePretty(id, -1)}, {traci.vehicle.getLaneChangeStatePretty(id, 1)}, {traci.vehicle.wantsAndCouldChangeLane(id, -1, state=None)}, {traci.vehicle.wantsAndCouldChangeLane(id, 1, state=None)}")
                        #     if traci.vehicle.couldChangeLane(id, 1):    # int = 1 is for the left lane change state
                        #         traci.vehicle.changeLaneRelative(id, 1, 0)
                        #
                        # else:
                        #     mylogger.debug(f"time: {NOW}, id: {id}, signal: {signal}, could change: {traci.vehicle.couldChangeLane(id, -1)}, {traci.vehicle.couldChangeLane(id, 1)}, state: {traci.vehicle.getLaneChangeStatePretty(id, -1)}, {traci.vehicle.getLaneChangeStatePretty(id, 1)}, {traci.vehicle.wantsAndCouldChangeLane(id, -1, state=None)}, {traci.vehicle.wantsAndCouldChangeLane(id, 1, state=None)}")


                    else:
                        # traci.vehicle.setSpeed(id, math.dist(v, [0,0,0]))
                        # ----- tips: initial setting -----
                        traci.vehicle.setSpeedMode(id, cav.default_speed_mode)
                        traci.vehicle.setLaneChangeMode(id, cav.default_lane_mode)
                        traci.vehicle.setSpeed(id, -1)
                        id2cavs.pop(id)
                        veh_ids.remove(id)
                else:
                    if math.dist(position, cav.waypoints[-1]) <= 20:
                        traci.vehicle.setSpeed(id, -1)
                        id2cavs.pop(id)
                        veh_ids.remove(id)

                    else:
                        # warnings.warn(f"No path: id: {id}, pos: {position}, force to stop")
                        mylogger.warning(f"No path: id: {id}, pos: {position}, force to stop")
                        traci.vehicle.setSpeed(id, 0)
                        # raise Exception(f"No path: id: {id}, pos: {position}")
                        # traci.vehicle.setSpeed(id, -1)

            except traci.exceptions.TraCIException:
                pass
            # """
    traci.close()
