from math import cos, sin, tan
from math import atan as arctan
from numpy import linalg
import numpy as np
import random
import argparse
import math
import json
import time
import struct
import gzip
from bitstring import BitArray

import matplotlib.pyplot as plt
from multiprocessing import Pool

from func import point2func, poly
from compression import ShannonFennonCompression, HuffmanCompression
from google_polyline import PolylineCodec

from kbm import Car, MCM
from compress_handler import CompressHandler

def update_result(all_result, d):
    all_result.update(d)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Kinematic Model')
    parser.add_argument('-vi', '--v_init', type=float, default=0.0)
    parser.add_argument('-ai', '--a_init', type=float, default=0.0)
    parser.add_argument('-si', '--s_init', type=float, default=0.0)
    parser.add_argument('-ti', '--t_init', type=float, default=0.0)
    parser.add_argument('-vmax', '--v_max', type=float, default=int(200 / 3600.0 * 1000.0))
    parser.add_argument('-smax', '--stear_max', type=float, default=45.0)
    parser.add_argument('-eth', '--error_th', type=float, default=0.1)
    parser.add_argument('--end', type=float, default=10.0)
    parser.add_argument('--dt', type=float, default=0.1)
    parser.add_argument('--d_dt', type=float, default=0.1)
    parser.add_argument('--np', type=int, default=40)
    parser.add_argument('-aui', '--a_update_interval', type=float, default=100)
    parser.add_argument('-sui', '--stear_update_interval', type=float, default=100)
    parser.add_argument('-rui', '--rotate_update_interval', type=float, default=100)
    parser.add_argument('-rv', '--rotate_value', type=float, default=0)
    parser.add_argument('-rt', '--rotate_type', action='store_true')
    parser.add_argument('-maxdim', type=int, default=10)
    parser.add_argument('-mindim', type=int, default=1)
    parser.add_argument('--result_path', default="./result_prev.json")

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
    force_stop_time = args.np * args.dt

    while car.latest_time() + args.dt <= force_stop_time:
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
            car.generate_path_by_rotate(a, rotate_radian, force_stop_time)
            rotate_update_time = max([car.latest_time(), rotate_update_time + args.rotate_update_interval])

            if args.rotate_type == True:
                rotate_type = -1 * rotate_type

        else:
            car.generate_path_by_delta_t(args.dt, a, stear)

    etsi_x, etsi_y, etsi_t, min_dt = car.path_filterd_by_ETSI()
    x, y, t = car.path_by_point_num(args.np, int(args.d_dt / args.dt))
    z = [0] * len(x)

    mcm = MCM(x, y, z, t)
    file_name = "_".join([f"{k}_{v}" for k, v in args.__dict__.items()])

    compress_handler = CompressHandler(
        maxdim = args.maxdim,
        mindim = args.mindim,
        acceptance_error = args.error_th,
        d_dt = args.dt,
        symbol_length = int(math.log2(4096)),
    )

    all_result = args.__dict__
    all_result.update({"normal_size": mcm.size()})

    print(f"\n\n\n ----- 0.1 sampling (normal size: {mcm.size()}) ----- \n\n\n")

    print("\n----- Proposed Method -----")
    result, comp_time,  size    = compress_handler.compress("proposed", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("proposed", result)
    errors = [math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
    name = "prop"
    all_result.update({
        f"{name}_size": size,
        f"{name}_comp_time": comp_time,
        f"{name}_decomp_time": decomp_time,
        f"{name}_max_error": max(errors),
        f"{name}_ave_error": sum(errors) / len(errors),
    })

    print("\n----- GZip Compression -----")
    result, comp_time,  size   = compress_handler.compress("gzip", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("gzip", result)
    errors = [math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
    name = "gzip"
    all_result.update({
        f"{name}_size": size,
        f"{name}_comp_time": comp_time,
        f"{name}_decomp_time": decomp_time,
        f"{name}_max_error": max(errors),
        f"{name}_ave_error": sum(errors) / len(errors),
    })

    # print("\n----- Shannon Fennon Compression -----")
    # result, comp_time,  size   = compress_handler.compress("shannon", t, x, y, z)
    # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("shannon", result)
    # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
    # name = "shannon_fennon"
    # all_result.update({
    #     f"{name}_size": size,
    #     f"{name}_comp_time": comp_time,
    #     f"{name}_decomp_time": decomp_time,
    # })

    print("\n----- Huffman Compression -----")
    result, comp_time,  size   = compress_handler.compress("huffman", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("huffman", result)
    errors = [math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
    name = "huffman"
    all_result.update({
        f"{name}_size": size,
        f"{name}_comp_time": comp_time,
        f"{name}_decomp_time": decomp_time,
        f"{name}_max_error": max(errors),
        f"{name}_ave_error": sum(errors) / len(errors),
    })

    # print("\n----- D3 Google Polyline -----")
    # result, comp_time, size   = compress_handler.compress("d3google", t, x, y, z)
    # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("d3google", result)
    # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- D4 Google Polyline -----")
    result, comp_time, size   = compress_handler.compress("d4google", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("d4google", result)
    errors = [math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
    name = "google"
    all_result.update({
        f"{name}_size": size,
        f"{name}_comp_time": comp_time,
        f"{name}_decomp_time": decomp_time,
        f"{name}_max_error": max(errors),
        f"{name}_ave_error": sum(errors) / len(errors),
    })

    print(f"\n\n\n ----- ETSI (ETSI-based size: {mcm.ETSI_size()}) ----- \n\n\n")
    etsi_calc_overhead = time.perf_counter()
    x, y, z, t = mcm.ETSI_filter(x, y, z, t)
    etsi_calc_overhead = time.perf_counter() - etsi_calc_overhead
    print(etsi_calc_overhead)
    name = "esti"
    all_result.update({
        f"{name}_size": mcm.ETSI_size(),
    })

    # print("\n----- GZip Compression -----")
    # result, comp_time,  size   = compress_handler.compress("gzip", t, x, y, z)
    # # result, comp_time,  size   = compress_handler.compress("gzip", mcm.etsi_t, mcm.etsi_x, mcm.etsi_y, mcm.etsi_z)
    # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("gzip", result)
    # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
    #
    # print("\n----- Shannon Fennon Compression -----")
    # result, comp_time,  size   = compress_handler.compress("shannon", t, x, y, z)
    # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("shannon", result)
    # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
    #
    # print("\n----- Huffman Compression -----")
    # result, comp_time,  size   = compress_handler.compress("huffman", t, x, y, z)
    # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("huffman", result)
    # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
    #
    # print("\n----- D4 Google Polyline -----")
    # result, comp_time, size   = compress_handler.compress("d4google", t, x, y, z)
    # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("d4google", result)
    # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- All Applied -----")
    result, comp_time, size   = compress_handler.compress("all-applied", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("all-applied", result)
    errors = [math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
    name = "all"
    all_result.update({
        f"{name}_size": size,
        f"{name}_comp_time": comp_time,
        f"{name}_decomp_time": decomp_time,
        f"{name}_max_error": max(errors),
        f"{name}_ave_error": sum(errors) / len(errors),
    })

    # all_result.update({"file_name": file_name})

    with open(args.result_path, "a") as f:
        f.write(json.dumps(all_result) + "\n")
