import argparse
import json

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Kinematic Model')
    parser.add_argument('-jfp', "--json_log_file_path", required=True)
    parser.add_argument('-odir', "--output_directory", required=True)
    parser.add_argument('--name', default="prop")

    args = parser.parse_args()    # 4. 引数を解析

    id2time2data = {}
    with open(args.json_log_file_path, "r") as f:
        lines = f.readlines()

        for l in lines:
            json_data = json.loads(l)

            if json_data["name"] != args.name:
                continue

            id = json_data["id"]
            t = json_data["time"]
            if id not in id2time2data.keys():
                id2time2data[id] = {}

            id2time2data[id][t] = json_data


    id2mcms = {}
    for id, time2data in id2time2data.items():
        if id not in id2mcms.keys():
            id2mcms[id] = []

        # print(f"id: {id}")
        # print(min([t for t in time2data.keys()]))
        for t, data in time2data.items():
            simple_mcm = {
                "timestamp": t,
                "option": {
                    "type": "MCM",
                    "size": data["size"]
                }
            }
            id2mcms[id].append(json.dumps(simple_mcm))

    for id, mcms in id2mcms.items():
        with open(f"{args.output_directory}/{id}_mcm.json", "w") as f:
            f.writelines("\n".join(mcms))

    # print(id2mcms)
