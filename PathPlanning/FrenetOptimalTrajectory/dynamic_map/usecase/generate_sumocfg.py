import argparse

def sumocfg(net_file_path=None, route_file_path=None):
    input_text_list = []

    if net_file_path is not None:
        input_text_list.append(f"<net-file value=\"{net_file_path}\"/>")

    if route_file_path is not None:
        input_text_list.append(f"<route-files value=\"{route_file_path}\"/>")


    input_text = "\n".join(input_text_list)
    return f"""
<configuration>
    <input>
        {input_text}
    </input>
    <time>
        <begin value="0"/>
        <end value="10000"/>
    </time>
</configuration>
"""

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='sumocfg generater.')
    parser.add_argument('-nfp', '--net_file_path', default=None)
    parser.add_argument('-rfp', '--route_file_path', default=None)
    parser.add_argument('-cfp', '--sumo_cfg_file_path', default=None)
    args = parser.parse_args()

    sumo_cfg_file_path = ""
    if args.sumo_cfg_file_path is not None:
        sumo_cfg_file_path = args.sumo_cfg_file_path

    else:
        if args.net_file_path is not None:
            sumo_cfg_file_path = args.net_file_path.split(".")[0] + ".sumocfg"

        elif args.route_file_path is not None:
            sumo_cfg_file_path = args.route_file_path.split(".")[0] + ".sumocfg"

        else:
            raise Exception("aaa")

    with open(sumo_cfg_file_path, "w") as f:
        f.write(sumocfg(args.net_file_path, args.route_file_path))
