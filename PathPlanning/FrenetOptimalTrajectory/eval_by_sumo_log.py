import sys, os
import argparse
import csv
import math
import time
import json

from kbm import Car, MCM
from compress_handler import CompressHandler

class SumoLogHandler:
    def __init__(self, log_file_path):
        with open(log_file_path, "r") as f:
            csvreader = csv.DictReader(f)
            self.data = [row for row in csvreader]

            self.id2data = {}
            for id in self.ids():
                self.id2data[id] = [d for d in self.data if d["id"] == id]
                self.id2data[id] = sorted(self.id2data[id], key=lambda d: float(d["time"]))
                # print(self.id2data[id])

    def times(self):
        return sorted(set([float(d["time"]) for d in self.data]))

    def ids(self):
        return list(set([d["id"] for d in self.data]))

    def ids_by_time(self, t):
        return list(set([d["id"] for d in self.data if float(d['time']) == t]))

    def trajectory_by_id_and_time(self, id, t, duration):
        times, x, y, z = None, None, None, None

        data = []
        while True:
            d = self.id2data[id].pop(0)
            # print(f"{id}, {float(d['time'])}")
            if float(d["time"]) < t:
                continue

            else:
                self.id2data[id] = [d] + self.id2data[id]
                break

        for d in self.id2data[id]:
            # print(f"{id}, {t}, {float(d['time'])}")
            if float(d['time']) <= t + duration:
                data.append(d)

            else:
                break

        # print(self.id2data[id])
        # print(data)
        times = [float(v["time"]) for v in data]
        x = [float(v['pos_x']) for v in data]
        y = [float(v['pos_y']) for v in data]
        z = [float(v['pos_z']) for v in data]

        # print(times)
        return times, x, y, z



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Dynamic Map.')
    parser.add_argument('-lfp', '--log_file_path')
    parser.add_argument('-dur', '--duration', type=float, default=20)
    args = parser.parse_args()

    sumo_log_handler = SumoLogHandler(args.log_file_path)

    all_result = []
    for ti in sumo_log_handler.times():
        # print(f"time: {ti}")

        for id in sumo_log_handler.ids_by_time(ti):
            # print(id)
            t, x, y, z = sumo_log_handler.trajectory_by_id_and_time(id, ti, args.duration)

            if len(t) <= 1 or t[-1] - t[0] < args.duration:
                continue


            mcm = MCM(x, y, z, t)
            compress_handler = CompressHandler(
                maxdim = 10,
                mindim = 1,
                acceptance_error = 0.1,
                d_dt = 0.1,
                symbol_length = int(math.log2(2**4)),
            )

            # print(f"\n\n\n ----- 0.1 sampling (normal size: {mcm.size()}) ----- \n\n\n")

            # print("\n----- Proposed Method -----")
            result, comp_time,  size    = compress_handler.compress("proposed", t, x, y, z)
            ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("proposed", result)
            errors = [math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]
            name = "prop"
            # print(f"name: {name}, size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
            print(json.dumps({
                "name": name,
                "id": id,
                "time": ti,
                f"size": size,
                f"comp_time": comp_time,
                f"decomp_time": decomp_time,
                f"max_error": max(errors),
                f"ave_error": sum(errors) / len(errors),
            }))

            # print("\n----- GZip Compression -----")
            result, comp_time,  size   = compress_handler.compress("gzip", t, x, y, z)
            ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("gzip", result)
            errors = [math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]
            name = "gzip"
            # print(f"name: {name}, size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
            print(json.dumps({
                "name": name,
                "id": id,
                "time": ti,
                f"size": size,
                f"comp_time": comp_time,
                f"decomp_time": decomp_time,
                f"max_error": max(errors),
                f"ave_error": sum(errors) / len(errors),
            }))

            # # print("\n----- Shannon Fennon Compression -----")
            # result, comp_time,  size   = compress_handler.compress("shannon", t, x, y, z)
            # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("shannon", result)
            # # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
            # name = "shannon_fennon"
            # all_result.update({
            #     f"{name}_size": size,
            #     f"{name}_comp_time": comp_time,
            #     f"{name}_decomp_time": decomp_time,
            # })

            # print("\n----- Huffman Compression -----")
            result, comp_time,  size   = compress_handler.compress("huffman", t, x, y, z)
            ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("huffman", result)
            errors = [math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]
            name = "huffman"
            # print(f"name: {name}, size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
            print(json.dumps({
                "name": name,
                "id": id,
                "time": ti,
                f"size": size,
                f"comp_time": comp_time,
                f"decomp_time": decomp_time,
                f"max_error": max(errors),
                f"ave_error": sum(errors) / len(errors),
            }))

            # # print("\n----- D3 Google Polyline -----")
            # result, comp_time, size   = compress_handler.compress("d3google", t, x, y, z)
            # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("d3google", result)
            # # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

            # print("\n----- D4 Google Polyline -----")
            result, comp_time, size   = compress_handler.compress("d4google", t, x, y, z)
            ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("d4google", result)
            errors = [math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]
            name = "google"
            # print(f"name: {name}, size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
            print(json.dumps({
                "name": name,
                "id": id,
                "time": ti,
                f"size": size,
                f"comp_time": comp_time,
                f"decomp_time": decomp_time,
                f"max_error": max(errors),
                f"ave_error": sum(errors) / len(errors),
            }))

            # print(f"\n\n\n ----- ETSI (ETSI-based size: {mcm.ETSI_size()}) ----- \n\n\n")
            etsi_calc_overhead = time.perf_counter()
            x, y, z, t = mcm.ETSI_filter(x, y, z, t)
            etsi_calc_overhead = time.perf_counter() - etsi_calc_overhead
            # print(etsi_calc_overhead)
            name = "esti"
            print(json.dumps({
                "name": name,
                "id": id,
                "time": ti,
                f"size": mcm.ETSI_size(),
                f"comp_time": etsi_calc_overhead,
                f"decomp_time": 0,
                f"max_error": 0,
                f"ave_error": 0,
            }))

            # # print("\n----- GZip Compression -----")
            # result, comp_time,  size   = compress_handler.compress("gzip", t, x, y, z)
            # # result, comp_time,  size   = compress_handler.compress("gzip", mcm.etsi_t, mcm.etsi_x, mcm.etsi_y, mcm.etsi_z)
            # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("gzip", result)
            # # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
            #
            # # print("\n----- Shannon Fennon Compression -----")
            # result, comp_time,  size   = compress_handler.compress("shannon", t, x, y, z)
            # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("shannon", result)
            # # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

            # # print("\n----- Huffman Compression -----")
            # result, comp_time,  size   = compress_handler.compress("huffman", t, x, y, z)
            # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("huffman", result)
            # # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
            #
            # # print("\n----- D4 Google Polyline -----")
            # result, comp_time, size   = compress_handler.compress("d4google", t, x, y, z)
            # ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("d4google", result)
            # # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

            # print("\n----- All Applied -----")
            result, comp_time, size   = compress_handler.compress("all-applied", t, x, y, z)
            ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("all-applied", result)
            errors = [math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]
            name = "all"
            # print(f"name: {name}, size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
            # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
            # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
            print(json.dumps({
                "name": name,
                "id": id,
                "time": ti,
                f"size": size,
                f"comp_time": comp_time,
                f"decomp_time": decomp_time,
                f"max_error": max(errors),
                f"ave_error": sum(errors) / len(errors),
            }))

            # with open(args.log_file_path.split("/")[-1].split(".")[0] + "_result.json", "w") as f:
            #     for d in all_result:
            #         f.write(json.dumps(all_result))
