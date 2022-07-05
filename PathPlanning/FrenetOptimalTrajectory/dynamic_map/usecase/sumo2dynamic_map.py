import argparse
import os, sys
import numpy as np
import math
import warnings

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

if __name__ == "__main__":
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

            except traci.exceptions.TraCIException:
                veh_ids.remove(id)
                continue

            # ----- add new cav -----
            if id not in id2cavs:
                _, current_edge_shapes = divide_waypoints_by_point( dynamic_map.waypoints_by_edges([edges[current_route_index]]), position )
                waypoints = [tuple(position)] + current_edge_shapes + dynamic_map.waypoints_by_edges(edges[(current_route_index + 1):])

                id2cavs[id] = CAV(
                    id,
                    net_file_path,
                    edges,
                    waypoints,
                    traci.vehicle.getSpeed(id),
                    traci.vehicle.getAcceleration(id),
                    [dynamic_map.road_width(edges[e_i], list(traci.vehicle.getPosition(id))) for e_i in range(0, len(edges))],
                    traci.simulation.getDeltaT(),
                    width=traci.vehicle.getWidth(id),
                    length=traci.vehicle.getLength(id),
                    heading=angle
                )

            cav = id2cavs[id]
            cav.clear_obstacle()
            # ----- for obstacles -----
            for ob_id, _ in id2cavs.items():
                if cav.id is not ob_id:
                    ob_angle = -traci.vehicle.getAngle(ob_id) + 90
                    ob_speed = traci.vehicle.getSpeed(ob_id)
                    ob_accel = traci.vehicle.getAcceleration(ob_id)

                    cav.add_obstacle(CarObstacle(
                        id=ob_id,
                        t = NOW,
                        heading=angle,
                        width=traci.vehicle.getWidth(ob_id),
                        length=traci.vehicle.getLength(ob_id),
                        position=list(traci.vehicle.getPosition(ob_id)) + [0],
                        speed=[ob_speed * math.cos(math.radians(ob_angle)), ob_speed * math.sin(math.radians(ob_angle)), 0],
                        accel=[ob_accel * math.cos(math.radians(ob_angle)), ob_accel * math.sin(math.radians(ob_angle)), 0],
                    ))

            # """
            # ----- update vehicle state -----
            angle, p, v, a, progress = cav.state(NOW + 0.1)
            if p is None or 0.5 <= progress:
                _, current_edge_shapes = divide_waypoints_by_point( dynamic_map.waypoints_by_edges([edges[current_route_index]]), position )
                waypoints = [tuple(position)] + current_edge_shapes + dynamic_map.waypoints_by_edges(edges[(current_route_index + 1):])

                cav.generate_new_path(
                    waypoints,
                    speed,
                    accel,
                    NOW,
                    min([
                        traci.vehicle.getMaxSpeed(id),
                        dynamic_map.max_speed(traci.vehicle.getRoute(id)[traci.vehicle.getRouteIndex(id)], traci.vehicle.getLaneID(id)
                    )]),
                    traci.vehicle.getAccel(id),
                    2
                )
                angle, p, v, a, progress = cav.state(NOW + 0.1)

            # # print(traci.vehicle.getPosition(id))

            print(f"{sys._getframe().f_code.co_name}, id: {id}, angle: {angle}, pos: {p}, speed: {v}, accel: {a}, progress: {progress}, lane: {traci.vehicle.getLaneID(id)}")
            print(f"{sys._getframe().f_code.co_name}, current_pos: {traci.vehicle.getPosition(id)}")

            # # print(cav.path.all_paths())
            try:
                if p is not None:
                    if 20 <= math.dist(position, waypoints[-1]):
                        traci.vehicle.moveToXY(
                            id,
                            traci.vehicle.getRoute(id)[traci.vehicle.getRouteIndex(id)],
                            int(traci.vehicle.getLaneID(id).split("_")[1]),
                            p[0],
                            p[1],
                            # math.degrees(angle) + 90
                        )
                        traci.vehicle.setSpeed(id, math.dist(v, [0,0,0]))

                    else:
                        # traci.vehicle.setSpeed(id, math.dist(v, [0,0,0]))
                        traci.vehicle.setSpeed(id, -1)
                        id2cavs.pop(id)

                else:
                    if math.dist(position, waypoints[-1]) <= 20:
                        traci.vehicle.setSpeed(id, -1)
                        id2cavs.pop(id)

                    else:
                        # warnings.warn(f"No path: id: {id}, pos: {position}")
                        raise Exception(f"No path: id: {id}, pos: {position}")
                        # traci.vehicle.setSpeed(id, -1)

            except traci.exceptions.TraCIException:
                pass
            # """
    traci.close()
