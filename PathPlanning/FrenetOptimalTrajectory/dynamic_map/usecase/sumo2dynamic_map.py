import argparse
import os, sys
import numpy as np
import math
import warnings
import pprint
import csv
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

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
from classes.message import *

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
    parser.add_argument('--communication_range', type=float, default=500)
    parser.add_argument('--sensor_range', type=float, default=50)
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
    cams = {}
    #cpm = CPM()
    cpms = {}
    mcms = {}

    cam_count = 0
    cpm_count = 0
    mcm_count = 0
    total_count = 0

    cam_size = 0.0
    cpm_size = 0.0
    mcm_size = 0.0
    total_size = 0.0

    send_dict = {}
    recv_dict = {}
    dpt_time = {}

    cbr = 0.0

    past = [0, 0, 0, 0, 0]

    min_dist_0 = 300.0
    min_dist_1 = 300.0

    step = 0

    with open('./result.csv', 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['vehicle', 'CAM', 'CPM', 'MCM', 'message', 'CBR'])

    with open('./id0_dist.csv', 'w') as fd0:
        writer = csv.writer(fd0)
        writer.writerow(['time', 'distance0'])

    with open('./id1_dist.csv', 'w') as fd1:
        writer = csv.writer(fd1)
        writer.writerow(['time', 'distance1'])

    with open('./id0_accel.csv', 'w') as fa0:
        writer = csv.writer(fa0)
        writer.writerow(['time', 'accel0'])

    with open('./id1_accel.csv', 'w') as fa1:
        writer = csv.writer(fa1)
        writer.writerow(['time', 'accel1'])
    
    with open('./send.csv', 'w') as s:
        writer = csv.writer(s)
        writer.writerow(['id', 'send-byte'])

    with open('./recv.csv', 'w') as r:
        writer = csv.writer(r)
        writer.writerow(['id', 'recv-byte'])

    with open("./pos0.csv", 'w') as p0:
        writer = csv.writer(p0)
        writer.writerow(['time', 'position0'])

    with open("./pos1.csv", 'w') as p1:
        writer = csv.writer(p1)
        writer.writerow(['time', 'position1'])
        

    try:
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

            if len(del_ids) > 0:    
                for sender_id in id2cavs:
                    for receiver_id in del_ids:
                        if sender_id == receiver_id:
                            continue
                        else:
                            if cams.get(sender_id).get(receiver_id) is not None:
                                cams[sender_id].pop(receiver_id)
                            if cpms.get(sender_id).get(receiver_id) is not None:
                                cpms[sender_id].pop(receiver_id)
                            if mcms.get(sender_id).get(receiver_id) is not None:
                                mcms[sender_id].pop(receiver_id)

                for sender_id in del_ids:
                    cams.pop(sender_id)
                    cpms.pop(sender_id)
                    mcms.pop(sender_id)
                

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
                        default_lane_mode=traci.vehicle.getLaneChangeMode(id),
                        default_tau=traci.vehicle.getTau(id)
                    )
                    # ----- tips: initial setting -----
                    traci.vehicle.setSpeedMode(id, 0)
                    traci.vehicle.setLaneChangeMode(id, 0)
                    traci.vehicle.setTau(id, 0)

                    dpt_time[id] = NOW

                cav = id2cavs[id]

                # ----- CAM and CPM generation ----
                if cav.id not in cams:
                    cams[cav.id] = {}
                    cpms[cav.id] = {}
                    send_dict[cav.id] = 0.0
                    recv_dict[cav.id] = 0.0

                for receiver_id, _ in id2cavs.items():
                    if receiver_id != cav.id and math.dist(traci.vehicle.getPosition(receiver_id), position) <= args.communication_range:
                        cams[cav.id][receiver_id] = CAM(cav.id, NOW, position, speed, accel, angle, cav.width, cav.length) 
                        cpms[cav.id][receiver_id] = CPM(NOW)
                        send_dict[cav.id] += 35.0
                        send_dict[cav.id] += 156.0
                        for surrounding_id, _ in id2cavs.items():
                            if cpms[cav.id].__sizeof__() >= 1500:
                                break   
                            if cav.id != surrounding_id and math.dist(traci.vehicle.getPosition(surrounding_id), position) <= args.sensor_range:
                                ob = CarObstacle(
                                    NOW,
                                    id2cavs[surrounding_id].width,
                                    id2cavs[surrounding_id].length,
                                    id2cavs[surrounding_id].heading,
                                    surrounding_id,
                                    list(traci.vehicle.getPosition(surrounding_id)) + [0],
                                    id2cavs[surrounding_id].speed,
                                    id2cavs[surrounding_id].accel,
                                    None
                                )
                                cpms[cav.id][receiver_id].add_obstacles(ob, t=NOW)
                                send_dict[cav.id] += 35.0
                            else:
                                continue
                    else:
                        continue
                #if cav.id not in cpm.positions:
                    #for surrounding_id, _ in id2cavs.items():
                        #if  cav.id != surrounding_id and math.dist(traci.vehicle.getPosition(surrounding_id), position)<= 50:
                            #ob = CarObstacle(
                                #NOW,
                                #id2cavs[surrounding_id].width,
                                #id2cavs[surrounding_id].length,
                                #id2cavs[surrounding_id].heading,
                                #surrounding_id,
                                #traci.vehicle.getPosition(surrounding_id),
                                #id2cavs[surrounding_id].speed,
                                #id2cavs[surrounding_id].accel,
                                #None
                            #)
                            #cpm[cav.id].add_obstacles(ob, t=NOW)
                        #else: 
                            #continue

                #for receiver_id, _ in id2cavs.items():
                    #if receiver_id != cav.id:
                        #cpms[cav.id][receiver_id] = cpm[cav.id]
                    #else: 
                        #continue

                cav.clear_obstacle()
                # ----- for obstacles -----

                #---- default code ----
                #for ob_id, _ in id2cavs.items():
                #    ob_pos = traci.vehicle.getPosition(ob_id)
                #    pos_vec = np.array(list(ob_pos)) - np.array(position)
                #    pos_dist = math.dist(ob_pos, position)
                #    angle_vec = np.array([math.cos(math.radians(angle)), math.sin(math.radians(angle))])
                #    if cav.id is not ob_id and -1/2 <= cos_sim(pos_vec, angle_vec) and pos_dist <= 150:
                #        ob_angle = -traci.vehicle.getAngle(ob_id) + 90
                #        ob_speed = traci.vehicle.getSpeed(ob_id)
                #        ob_accel = traci.vehicle.getAcceleration(ob_id)
                #        mylogger.debug(f"time: {NOW}, id: {id}, ob_id: {ob_id}, ob_angle: {ob_angle}, ob_speed: {ob_speed}, ob_accel: {ob_accel}")
                #        cav.add_obstacle(CarObstacle(
                #            id=ob_id,
                #            t = NOW,
                #            heading=ob_angle,
                #            width=traci.vehicle.getWidth(ob_id),
                #            length=traci.vehicle.getLength(ob_id),
                #            position=list(traci.vehicle.getPosition(ob_id)) + [0],
                #            speed=[ob_speed * math.cos(math.radians(ob_angle)), ob_speed * math.sin(math.radians(ob_angle)), 0],
                #            accel=[ob_accel * math.cos(math.radians(ob_angle)), ob_accel * math.sin(math.radians(ob_angle)), 0],
                #        ))

                #---- Collision detection using CAM and MCM ---- 
                for ob_id, _ in id2cavs.items():
                    if mcms.get(ob_id) is None or mcms.get(ob_id).get(cav.id) is None:
                        continue
                    elif cams.get(ob_id) is None or cams.get(ob_id).get(cav.id) is None:
                        continue
                    else:
                        ob_pos = cams[ob_id][cav.id].position
                        pos_vec = np.array(ob_pos) - np.array(position)
                        pos_dist = math.dist(ob_pos, position)
                        angle_vec = np.array([math.cos(math.radians(angle)), math.sin(math.radians(angle))])
                        if ob_id != cav.id and -1/2 <= cos_sim(pos_vec, angle_vec) and pos_dist <= 150:
                            if mcms[ob_id][cav.id].fpath is not None:
                                cav.add_obstacle(CarObstacle(
                                    id = ob_id, 
                                    t = NOW - args.step,
                                    heading = cams[ob_id][cav.id].angle,
                                    width = cams[ob_id][cav.id].width,
                                    length  = cams[ob_id][cav.id].length,
                                    position = None,
                                    speed = None,
                                    accel = None,
                                    path = mcms[ob_id][cav.id].fpath
                                ))
                            else:
                                ob_angle = cams[ob_id][cav.id].angle
                                ob_speed = cams[ob_id][cav.id].speed
                                ob_accel = cams[ob_id][cav.id].accel
                                cav.add_obstacle(CarObstacle(
                                    id = ob_id,
                                    t = NOW - args.step,
                                    heading = ob_angle,
                                    width = cams[ob_id][cav.id].width,
                                    length = cams[ob_id][cav.id].length,
                                    position = ob_pos + [0],
                                    speed = [ob_speed * math.cos(math.radians(ob_angle)), ob_speed * math.sin(math.radians(ob_angle)), 0],
                                    accel = [ob_accel * math.cos(math.radians(ob_angle)), ob_accel * math.sin(math.radians(ob_angle)), 0],
                                ))

                #---- Collisiton detection using only CAM ----
                #for ob_id, _ in id2cavs.items():
                #    if cams.get(ob_id) is None or cams.get(ob_id).get(cav.id) is None:
                #        continue
                #    else:
                #        ob_pos = cams[ob_id][cav.id].position
                #        pos_vec = np.array(ob_pos) - np.array(position)
                #        pos_dist = math.dist(ob_pos, position)
                #        angle_vec = np.array([math.cos(math.radians(angle)), math.sin(math.radians(angle))])
                #        if ob_id != cav.id and -1/2 <= cos_sim(pos_vec, angle_vec) and pos_dist <= 150:
                #            #print(f"ob_id: {ob_id}")
                #            ob_angle = cams[ob_id][cav.id].angle
                #            #print(f"ob_angle: {ob_angle}, Real-time value: {-traci.vehicle.getAngle(ob_id) + 90}")
                #            ob_speed = cams[ob_id][cav.id].speed
                #            #print(f"ob_speed: {ob_speed}, Real-time value: {traci.vehicle.getSpeed(ob_id)}")
                #            ob_accel = cams[ob_id][cav.id].accel
                #            #print(f"ob_accel: {ob_accel}, Real-time value: {traci.vehicle.getAcceleration(ob_id)}")
                #            cav.add_obstacle(CarObstacle(
                #                id = ob_id,
                #                t = NOW - args.step,
                #                heading = ob_angle,
                #                width = cams[ob_id][cav.id].width,
                #                length = cams[ob_id][cav.id].length,
                #                position = ob_pos + [0],
                #                speed = [ob_speed * math.cos(math.radians(ob_angle)), ob_speed * math.sin(math.radians(ob_angle)), 0],
                #                accel = [ob_accel * math.cos(math.radians(ob_angle)), ob_accel * math.sin(math.radians(ob_angle)), 0],
                #            ))

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

                # ----- MCM generation ----
                #if step % 5 == 0:
                if cav.id not in mcms:
                    mcms[cav.id] = {}

                for receiver_id, _ in id2cavs.items():
                    if receiver_id != cav.id and math.dist(traci.vehicle.getPosition(receiver_id), position) <= args.communication_range:
                        mcms[cav.id][receiver_id] = MCM(NOW, edges, cav.path)
                        send_dict[cav.id] += 9.0 * len(mcms[cav.id][receiver_id].fpath.data)
                    else:
                        continue

                if len(id2cavs) > 1:    
                    for sender_id, _ in id2cavs.items():
                        if sender_id == cav.id:
                            continue
                        if cams.get(sender_id).get(cav.id) is not None:
                            if cams[sender_id][cav.id].time == NOW - args.step or cams[sender_id][cav.id].time == NOW:
                                recv_dict[cav.id] += 35.0

                        if cpms.get(sender_id).get(cav.id) is not None:
                            if cpms[sender_id][cav.id].time == NOW - args.step or cpms[sender_id][cav.id].time == NOW:
                                recv_dict[cav.id] += 156.0
                                for surrounding_id, _ in cpms[sender_id][cav.id].positions.items():
                                    if cpms[sender_id][cav.id].positions.get(surrounding_id) is None:
                                        continue
                                    else:
                                        recv_dict[cav.id] += 35.0

                        if mcms.get(sender_id).get(cav.id) is not None:
                            if mcms[sender_id][cav.id].time == NOW - args.step or mcms[sender_id][cav.id].time == NOW:
                                recv_dict[cav.id] += 9.0 * len(cav.path.data)
                                #print(f"len(path): {len(cav.path.data)}")
                            

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
                            traci.vehicle.setTau(id, cav.default_tau)
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

                #def __init__(self):
                    #self.positions = {}
                #def add_obstacle(self, ob, t):
                    #self.positions[ob.id] = ob.positions(t)
            
            with open('./send.csv', 'a') as send:
                writer = csv.writer(send)
                for key, value in send_dict.items():
                    try:
                        writer.writerow([key, value / (NOW - dpt_time[key])])
                    except:
                        writer.writerow([key, value])
                
            with open('./recv.csv', 'a') as recv:
                writer = csv.writer(recv)
                for key, value in recv_dict.items():
                    try:
                        writer.writerow([key, value / (NOW - dpt_time[key])])
                    except:
                        writer.writerow([key, value])

            for sender_id, _ in cams.items():
                for receiver_id, _ in cams.items():
                    if len(veh_ids) == 1:
                        break
                    elif sender_id == receiver_id:
                        continue
                    elif sender_id in veh_ids and receiver_id in veh_ids:
                        if cams.get(sender_id).get(receiver_id) is None:
                            continue
                        else:
                            cam_count += 1
                            cam_size += 35.0
                    else: 
                        continue

            for sender_id, _ in cpms.items():
                for receiver_id, _ in cpms.items():
                    if len(veh_ids) == 1:
                        break
                    elif sender_id == receiver_id:
                        continue
                    elif sender_id in veh_ids and receiver_id in veh_ids: 
                        if cpms.get(sender_id).get(receiver_id) is None:
                            continue
                        else:
                            cpm_size += 156.0
                            for surrounding_id, _ in cpms[sender_id][receiver_id].positions.items():
                                if cpms[sender_id][receiver_id].positions.get(surrounding_id) is None:
                                    continue
                                else:
                                    cpm_size += 35.0
                            cpm_count += 1
                    else: 
                        continue

            for sender_id, _ in mcms.items():
                for receiver_id, _ in mcms.items():
                    if len(veh_ids) == 1:
                        break
                    elif sender_id == receiver_id:
                        continue
                    elif sender_id in veh_ids and receiver_id in veh_ids: 
                        if mcms.get(sender_id).get(receiver_id) is None:
                            continue
                        else:
                            mcm_count += 1
                            mcm_size += 9.0 * len(mcms[sender_id][receiver_id].fpath.data)
                    else: 
                        continue
            
            total_count = cam_count + cpm_count + mcm_count

            total_size = cam_size + cpm_size + mcm_size
            cbr = total_size * 8.0 / 600000.0

            print(f"vehicle: {len(veh_ids)}, CAM: {cam_count}, CPM: {cpm_count}, MCM: {mcm_count}, total: {total_count}")
            print(f"vehicle: {len(veh_ids)}, CAM: {cam_size}, CPM: {cpm_size}, MCM: {mcm_size}, total: {total_size}, CBR: {cbr}")

            #if past is not [0, 0] and [len(veh_ids), cam_count, cpm_count, mcm_count, total_count] != past:
            #    with open('./result.csv', 'a') as f:
            #        writer = csv.writer(f)
            #        writer.writerow([len(veh_ids), cam_count, cpm_count, mcm_count, total_count])
                    
            if past is not [0, 0] and [len(veh_ids), cam_count, cpm_count, mcm_count, total_count] != past:
                with open('./result.csv', 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow([len(veh_ids), cam_size, cpm_size, mcm_size, total_size, cbr])

            past = [len(veh_ids), cam_count, cpm_count, mcm_count, total_count]

            cam_count = 0
            cpm_count = 0
            mcm_count = 0
            total_count = 0

            cam_size = 0.0
            cpm_size = 0.0
            mcm_size = 0.0
            total_size = 0.0

            if '0' in veh_ids:
                for id, _ in id2cavs.items():
                    if id == '0':
                        continue
                    else:
                        dist = math.dist(traci.vehicle.getPosition('0'), traci.vehicle.getPosition(id))
                        if min_dist_0 > dist:
                            min_dist_0 = dist
                
                with open('./id0_dist.csv', 'a') as fd0:
                    writer = csv.writer(fd0)
                    writer.writerow([NOW, min_dist_0])

                with open('./id0_accel.csv', 'a') as fa0:
                    writer = csv.writer(fa0)
                    writer.writerow([NOW, traci.vehicle.getAcceleration('0')])

                if abs(traci.vehicle.getDrivingDistance('0', ':gneJ5_1', 1.0)) < 1000:
                    with open('./pos0.csv', 'a') as p0:
                        writer = csv.writer(p0)
                        writer.writerow([NOW, traci.vehicle.getDrivingDistance('0', ':gneJ5_1', 1.0)])

            if '1' in veh_ids:
                for id, _ in id2cavs.items():
                    if id == '1':
                        continue
                    else:
                        dist = math.dist(traci.vehicle.getPosition('1'), traci.vehicle.getPosition(id))
                        if min_dist_1 > dist:
                            min_dist_1 = dist
                
                with open('./id1_dist.csv', 'a') as fd1:
                    writer = csv.writer(fd1)
                    writer.writerow([NOW, min_dist_1])

                with open('./id1_accel.csv', 'a') as fa1:
                    writer = csv.writer(fa1)
                    writer.writerow([NOW, traci.vehicle.getAcceleration('1')])

                if abs(traci.vehicle.getDrivingDistance('1', ':gneJ5_0', 1.0)) < 1000:
                    with open('./pos1.csv', 'a') as p1:
                        writer = csv.writer(p1)
                        writer.writerow([NOW, traci.vehicle.getDrivingDistance('1', ':gneJ5_0', 1.0)])
            
            min_dist_0 = 500.0
            min_dist_1 = 500.0

            step += 1
        traci.close()

    except (KeyboardInterrupt, traci.exceptions.FatalTraCIError) as e:
        data = pd.read_csv('./result.csv', encoding = 'UTF8')

        data_vehicle = data[data.columns[0]]
        data_cam = data[data.columns[1]]
        data_cpm = data[data.columns[2]]
        data_mcm = data[data.columns[3]]
        data_total = data[data.columns[4]]
        data_cbr = data[data.columns[5]]

        plt.xlabel(data.columns[0])
        plt.ylabel(data.columns[1])

        plt.scatter(data_vehicle, data_cam)
        plt.show()

        plt.xlabel(data.columns[0])
        plt.ylabel(data.columns[2])

        plt.scatter(data_vehicle, data_cpm)
        plt.show()

        plt.xlabel(data.columns[0])
        plt.ylabel(data.columns[3])

        plt.scatter(data_vehicle, data_mcm)
        plt.show()

        plt.xlabel(data.columns[0])
        plt.ylabel(data.columns[4])

        plt.scatter(data_vehicle, data_total)
        plt.show()

        plt.xlabel(data.columns[0])
        plt.ylabel(data.columns[5])

        plt.scatter(data_vehicle, data_cbr)
        plt.show()

        data_dist0 = pd.read_csv('id0_dist.csv', encoding = 'UTF8')

        data_dtime0 = data_dist0[data_dist0.columns[0]]
        data_dvalue0 = data_dist0[data_dist0.columns[1]]

        plt.xlabel(data_dist0.columns[0])
        plt.ylabel(data_dist0.columns[1])

        plt.scatter(data_dtime0, data_dvalue0)
        plt.show()

        data_dist1 = pd.read_csv('id1_dist.csv', encoding = 'UTF8')

        data_dtime1 = data_dist1[data_dist1.columns[0]]
        data_dvalue1 = data_dist1[data_dist1.columns[1]]

        plt.xlabel(data_dist1.columns[0])
        plt.ylabel(data_dist1.columns[1])

        plt.scatter(data_dtime1, data_dvalue1)
        plt.show()

        data_accel0 = pd.read_csv('id0_accel.csv', encoding = 'UTF8')

        data_atime0 = data_accel0[data_accel0.columns[0]]
        data_avalue0 = data_accel0[data_accel0.columns[1]]

        plt.xlabel(data_accel0.columns[0])
        plt.ylabel(data_accel0.columns[1])

        plt.scatter(data_atime0, data_avalue0)
        plt.show()

        data_accel1 = pd.read_csv('id1_accel.csv', encoding = 'UTF8')

        data_atime1 = data_accel1[data_accel1.columns[0]]
        data_avalue1 = data_accel1[data_accel1.columns[1]]

        plt.xlabel(data_accel1.columns[0])
        plt.ylabel(data_accel1.columns[1])

        plt.scatter(data_atime1, data_avalue1)
        plt.show()

        data_send = pd.read_csv('send.csv', encoding = 'UTF8')

        data_sid = data_send[data_send.columns[0]]
        data_sbyte = data_send[data_send.columns[1]]

        plt.xlabel(data_send.columns[0])
        plt.ylabel(data_send.columns[1])

        plt.scatter(data_sid, data_sbyte)
        plt.show()

        data_recv = pd.read_csv('recv.csv', encoding = 'UTF8')

        data_rid = data_recv[data_recv.columns[0]]
        data_rbyte = data_recv[data_recv.columns[1]]

        plt.xlabel(data_recv.columns[0])
        plt.ylabel(data_recv.columns[1])

        plt.scatter(data_rid, data_rbyte)
        plt.show()

        data_pos0 = pd.read_csv('pos0.csv', encoding = 'UTF8')

        data_p0 = data_pos0[data_pos0.columns[1]]
        data_time0 = data_pos0[data_pos0.columns[0]]

        plt.xlabel(data_pos0.columns[0])
        plt.ylabel(data_pos0.columns[1])

        plt.scatter(data_time0, data_p0)
        plt.show()

        data_pos1 = pd.read_csv('pos1.csv', encoding = 'UTF8')

        data_p1 = data_pos1[data_pos1.columns[1]]
        data_time1 = data_pos1[data_pos1.columns[0]]

        plt.xlabel(data_pos1.columns[0])
        plt.ylabel(data_pos1.columns[1])

        plt.scatter(data_time1, data_p1)
        plt.show()

        print('finish')
        print(e)