import sys, os
import argparse
import csv

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci

def log_file_path(sumo_cfg_file_path):
    return "./" + "".join(sumo_cfg_file_path.split(".")[:-1]) + "_log.csv"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Dynamic Map.')
    parser.add_argument('-cfp', '--sumo_cfg_file_path', required=True)
    parser.add_argument('-st', '--start_time', type=float, default=600)
    parser.add_argument('-et', '--end_time', type=float, default=720)
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

    sumoBinary = "sumo"
    if args.gui:
        sumoBinary = "sumo-gui"
    # traci.start(f"{sumoBinary} -c {args.sumo_cfg_file_path} --step-length {args.step} --seed {args.seed} -b {args.start_time} -e {args.end_time}".split(" "))
    traci.start(f"{sumoBinary} -c {args.sumo_cfg_file_path} --step-length {args.step} --seed {args.seed}".split(" "))

    logs = []
    veh_ids = []
    while traci.simulation.getTime() <= args.end_time:
        traci.simulationStep()
        del_ids = traci.simulation.getArrivedIDList()
        add_ids = traci.simulation.getDepartedIDList()
        veh_ids = list((set(veh_ids) | set(add_ids)) - set(del_ids))

        if traci.simulation.getTime() < args.start_time:
            continue

        for veh_id in veh_ids:
            pos = traci.vehicle.getPosition(veh_id)

            logs.append({
                "time": traci.simulation.getTime(),
                "id": veh_id,
                "pos_x": pos[0],
                "pos_y": pos[1],
                "pos_z": 0,
            })


    with open(log_file_path(args.sumo_cfg_file_path), 'w', encoding='utf-8') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames = logs[0].keys())
        writer.writeheader()
        writer.writerows(logs)

    traci.close()
