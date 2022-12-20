import numpy as np
import sys
import os
import pprint
import time
import math
from math import sin, cos, radians
import gzip
from bitstring import BitArray, Bits

from func import point2func, poly
from compression import ShannonFennonCompression, HuffmanCompression
from google_polyline import D3PolylineCodec, D4PolylineCodec, D1PolylineCodec

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/dynamic_map/usecase/classes/")
# import poly as poly_class


from kbm import MCM, multi_container, ProposedHandler

class NoMethodException(Exception):
    pass

class CompressHandler:
    def __init__(self, maxdim = 10, mindim = 1, acceptance_error = 0.1, d_dt = 0.1, compresslevel=9, symbol_length = 12, google_order=2):
        self.maxdim = maxdim
        self.mindim = mindim
        self.acceptance_error = acceptance_error
        self.d_dt = d_dt

        self.compresslevel = 9

        self.K = symbol_length
        # print(self.K)
        self.shannon_fennon = ShannonFennonCompression()
        self.huffman = HuffmanCompression()

        self.d1_google_polyline = D1PolylineCodec()
        self.d4_google_polyline = D4PolylineCodec()
        self.d3_google_polyline = D3PolylineCodec()
        self.google_order = -math.log10(acceptance_error)

        self.huffman_for_all = HuffmanCompression()

        self.huffman_for_prop = HuffmanCompression()
        self.huffman_for_prop_all = HuffmanCompression()

        pass

    def compress(self, method_name, t, x, y, z):
        take_time = None
        result = None
        size = None

        mcm = MCM(x, y, z, t)

        if method_name == "proposed":
            start_time = time.perf_counter()

            res_x = multi_container(mcm.t[:], mcm.x[:], self.maxdim, self.acceptance_error, self.mindim)
            res_y = multi_container(mcm.t[:], mcm.y[:], self.maxdim, self.acceptance_error, self.mindim)
            res_z = multi_container(mcm.t[:], mcm.z[:], self.maxdim, self.acceptance_error, self.mindim)

            take_time = time.perf_counter() - start_time
            mcm.compressed_data = [res_x, res_y, res_z]

            # x_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_x])
            # y_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_y])
            # z_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_z])
            # mcm.compressed_size = 14 + x_size + y_size + z_size

            x_size = sum([(1 + 1 + 4 * d["efficient_dim"]) for d in res_x])
            y_size = sum([(1 + 1 + 4 * d["efficient_dim"]) for d in res_y])
            z_size = sum([(1 + 1 + 4 * d["efficient_dim"]) for d in res_z])
            mcm.compressed_size = 1 + 1 + 1 + 1 + 1 + x_size + y_size + z_size

        elif method_name == "proposed-filter":
            result, take_time, size = self.compress("proposed", t, x, y, z)
            ans_t, ans_x, ans_y, ans_z, decomp_time = self.decompress("proposed", result)

            start_time = time.perf_counter()
            Ts = sorted(result.compressed_data[0] + result.compressed_data[1] + result.compressed_data[2], key=lambda d: d["start_time"])
            indexes = []
            for d in Ts:
                start_point = int(round(d["start_time"], 2) / self.d_dt)
                end_point = start_point + d["efficient_point"] - 1
                required_point = d["efficient_dim"]
                target_point_num = len([p for p in indexes if start_point <= p and p <= end_point])

                # print(f"{start_point}, {end_point}, {required_point}")
                if required_point <= target_point_num:
                    continue

                density = required_point / (end_point - start_point + 1)
                related_density = 0
                if 1 <= len(indexes) and start_point <= indexes[-1]:
                    related_end = min([end_point, indexes[-1]])
                    related_density = len([p for p in indexes if start_point <= p and p <= related_end]) / (related_end - start_point + 1)

                if related_density <= density:
                    indexes = [i for i in indexes if i < start_point] + [round(start_point + (end_point - start_point) * (float(i) / required_point)) for i in range(0, required_point)] + [i for i in indexes if end_point <= i]
                    # print(indexes)

                else:
                    # print(f"{start_point}, {end_point}, {density}, {required_point}")
                    # print(f"{start_point}, {related_end}, {related_density}")
                    required_point = required_point - len([p for p in indexes if start_point <= p and p < related_end])
                    # print(required_point)
                    indexes = [i for i in indexes if i < related_end] + [round(related_end + (end_point - related_end) * (float(i) / required_point)) for i in range(0, required_point)] + [i for i in indexes if end_point <= i]
                    # print(indexes)
                    # raise Exception("aaa")
            take_time_2 = start_time - time.perf_counter()

                # if len([t for t in times if t <= d["dur"]])
            # print(len(indexes))
            # print("!!!")

            # mcm, take_time_3, _ = self.compress(
            #     "all-applied",
            #     [t[i] for i in indexes],
            #     [(x[i] + ans_x[i]) / 2 for i in indexes],
            #     [(y[i] + ans_y[i]) / 2 for i in indexes],
            #     [(z[i] + ans_z[i]) / 2 for i in indexes]
            #     )
            #
            # mcm, take_time_3, _ = self.compress(
            #     "all-applied",
            #     [t[i] for i in indexes],
            #     [(ans_x[i])  for i in indexes],
            #     [(ans_y[i])  for i in indexes],
            #     [(ans_z[i])  for i in indexes]
            #     )

            mcm, take_time_3, _ = self.compress(
                "all-applied",
                [t[i] for i in indexes],
                [(ans_x[i])  for i in indexes],
                [(ans_y[i])  for i in indexes],
                [(ans_z[i])  for i in indexes]
                )

            mcm.compressed_data = {
                "x": [{"point": d["efficient_point"], "dim": d["efficient_dim"]} for d in result.compressed_data[0]],
                "y": [{"point": d["efficient_point"], "dim": d["efficient_dim"]} for d in result.compressed_data[1]],
                "z": [{"point": d["efficient_point"], "dim": d["efficient_dim"]} for d in result.compressed_data[2]],
                "compressed_data": mcm.compressed_data
            }
            print(len(result.compressed_data[0]))
            print(len(result.compressed_data[1]))
            print(len(result.compressed_data[2]))
            mcm.compressed_size = 5 + mcm.compressed_size + 2 * ( len(result.compressed_data[0]) + len(result.compressed_data[1]) + len(result.compressed_data[2]) )
            # result.t_d = self.d_dt
            take_time = take_time + take_time_2 + take_time_3
            # mcm.compressed_size = len(compressed_data) / 8
            # mcm.compressed_data = compressed_data

        elif method_name == "proposed-all":
            K = self.K
            start_time = time.perf_counter()

            mcm, take_time, _ = self.compress("proposed", t, x, y, z)

            dtc_x = [[d['efficient_point'], d['efficient_dim']] + d['coefficients'].flatten().tolist() for d in mcm.compressed_data[0]]
            dtc_y = [[d['efficient_point'], d['efficient_dim']] + d['coefficients'].flatten().tolist() for d in mcm.compressed_data[1]]
            dtc_z = [[d['efficient_point'], d['efficient_dim']] + d['coefficients'].flatten().tolist() for d in mcm.compressed_data[2]]

            data = [(d,) for d in sum(dtc_x + dtc_y + dtc_z, [])]
            print(data)
            symbols = self.d1_google_polyline.encode(data, 10).encode("ascii")
            result = self.huffman_for_prop_all.compress(symbols)

            sym2bit = {}
            for i in result:
                sym2bit[i.symbol] = i.code

            compressed_data = ""
            for sym in symbols:
                compressed_data = compressed_data + sym2bit[sym]

            take_time = time.perf_counter() - start_time
            mcm.compressed_size = len(compressed_data) / 8
            mcm.compressed_data = compressed_data
            # print(compressed_data)

        elif method_name == "proposed-huffman":
            K = self.K
            start_time = time.perf_counter()

            mcm, take_time, _ = self.compress("proposed", t, x, y, z)

            dtc_x = [[d['efficient_point'], d['efficient_dim'], d['coefficients'].flatten()] for d in mcm.compressed_data[0]]
            dtc_y = [[d['efficient_point'], d['efficient_dim'], d['coefficients'].flatten()] for d in mcm.compressed_data[1]]
            dtc_z = [[d['efficient_point'], d['efficient_dim'], d['coefficients'].flatten()] for d in mcm.compressed_data[2]]

            prop_handler = ProposedHandler()
            bits = prop_handler.element2bit(self.d_dt, self.acceptance_error, dtc_x, dtc_y, dtc_z)
            symbols = [str(int(bits[K * i: K * (i + 1)], 2)) for i in range(0, int(len(bits) / K))]
            if K * int(len(bits) / K) < len(bits):
                symbols = symbols + [str(int(bits[K * int(len(bits) / K):], 2))]

            # start_time = time.perf_counter()

            result = self.huffman_for_prop.compress(symbols)

            overhead_bit = 0
            max_simbit = max([len(i.code) for i in result])
            overhead_bit = (K + max_simbit) * len(result)

            sym2bit = {}
            for i in result:
                sym2bit[i.symbol] = i.code

            # print(symbols)
            compressed_data = ""
            for sym in symbols:
                compressed_data = compressed_data + sym2bit[sym]

            result = compressed_data
            # print(len(result) / 8)
            take_time = take_time + time.perf_counter() - start_time
            mcm.compressed_data = compressed_data
            mcm.compressed_size = len(compressed_data) / 8
            print(compressed_data)


        elif method_name == "gzip":
            bits = mcm.bit_str()
            b = BitArray(bin=bits)

            start_time = time.perf_counter()
            mcm.compressed_data = gzip.compress(b.tobytes(), compresslevel=self.compresslevel, mtime=None)
            take_time = time.perf_counter() - start_time

            mcm.compressed_size = len(mcm.compressed_data) # 1 byte per 1 character

        elif method_name == "shannon":
            return self.__comp_shannon_or_huffman(method_name, t, x, y, z, self.K)

        elif method_name == "huffman":
            return self.__comp_shannon_or_huffman(method_name, t, x, y, z, self.K)

        elif "google" in method_name:
            # trajectory = [(mcm.x[i], mcm.y[i], mcm.z[i]) for i in range(0, len(t))]
            start_time = time.perf_counter()

            if method_name == "d4google":
                trajectory = [(mcm.x[i], mcm.y[i], mcm.z[i], mcm.t[i]) for i in range(0, len(t))]
                mcm.compressed_data = self.d4_google_polyline.encode(trajectory, self.google_order)

            elif method_name == "d3google":
                trajectory = [(mcm.x[i], mcm.y[i], mcm.z[i]) for i in range(0, len(t))]
                mcm.compressed_data = self.d3_google_polyline.encode(trajectory, self.google_order)

            else:
                raise NoMethodException(f"There are no method with the name {method_name}")

            take_time = time.perf_counter() - start_time
            mcm.compressed_size = len(mcm.compressed_data) # 1 byte per 1 character

        elif method_name == "all-applied":
            trajectory = [(mcm.x[i], mcm.y[i], mcm.z[i], mcm.t[i]) for i in range(0, len(t))]

            start_time = time.perf_counter()
            symbols = self.d4_google_polyline.encode(trajectory, self.google_order).encode("ascii")
            result = self.huffman_for_all.compress(symbols)

            sym2bit = {}
            for i in result:
                sym2bit[i.symbol] = i.code

            compressed_data = ""
            for sym in symbols:
                compressed_data = compressed_data + sym2bit[sym]

            max_simbit = max([len(i.code) for i in result])
            overhead_bit = (8 + max_simbit) * len(result)
            # overhead_bit = 0

            take_time = time.perf_counter() - start_time
            mcm.compressed_size = (len(compressed_data) + overhead_bit) / 8
            mcm.compressed_data = compressed_data
            mcm.compressed_overhead = (overhead_bit) / 8

        else:
            raise NoMethodException(f"There are no method with the name {method_name}")

        return mcm, take_time, mcm.compressed_size

    def __comp_shannon_or_huffman(self, method_name, t, x, y, z, K):
        mcm = MCM(x, y, z, t)
        bits = mcm.bit_str()
        # print(bits)
        symbols = [str(int(bits[K * i: K * (i + 1)], 2)) for i in range(0, int(len(bits) / K))]
        if K * int(len(bits) / K) < len(bits):
            symbols = symbols + [str(int(bits[K * int(len(bits) / K):], 2))]

        start_time = time.perf_counter()
        if method_name == "shannon":
            result = self.shannon_fennon.compress(symbols)

        elif method_name == "huffman":
            result = self.huffman.compress(symbols)

        else:
            NoMethodException(f"There are no method with the name {method_name}")

        max_simbit = max([len(i.code) for i in result])
        overhead_bit = (K + max_simbit) * len(result)
        # overhead_bit = 0

        sym2bit = {}
        for i in result:
            sym2bit[i.symbol] = i.code

        # print(symbols)
        compressed_data = ""
        for sym in symbols:
            compressed_data = compressed_data + sym2bit[sym]

        result = compressed_data
        take_time = time.perf_counter() - start_time
        mcm.compressed_size = (len(compressed_data) + overhead_bit) / 8
        mcm.compressed_data = compressed_data
        mcm.compressed_overhead = (overhead_bit) / 8

        return mcm, take_time, mcm.compressed_size

    def __decomp_shannon_or_huffman(self, method_name, result, K):
        take_time = None
        ans = None
        ans_t, ans_x, ans_y, ans_z = None, None, None, None

        start_time = time.perf_counter()
        # print(result)
        if method_name == "shannon":
            ans = self.shannon_fennon.decompress(result.compressed_data, K)

        elif method_name == "huffman":
            ans = self.huffman.decompress(result.compressed_data, K)

        # print(result)
        ans = result.bit2points(ans)
        take_time = time.perf_counter() - start_time
        ans_t = [d for d in ans[0]]
        ans_x = [d for d in ans[1]]
        ans_y = [d for d in ans[2]]
        ans_z = [d for d in ans[3]]

        return ans_t, ans_x, ans_y, ans_z, take_time

    def decompress(self, method_name, result):
        take_time = None
        ans = None
        ans_t, ans_x, ans_y, ans_z = None, None, None, None

        if method_name == "proposed":
            res_x, res_y, res_z = result.compressed_data[0], result.compressed_data[1], result.compressed_data[2]
            t_0 = result.t0
            t_d = result.t_d
            start_time = time.perf_counter()

            # print(res_x)
            points = np.cumsum(np.array([0] + [d["efficient_point"] for d in res_x]))
            ans_x = np.array([[result.x0 + poly(res_x[j]["coefficients"].flatten(), i * t_d)] for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1]) ]).flatten().tolist()
            points = np.cumsum(np.array([0] + [d["efficient_point"] for d in res_y]))
            ans_y = np.array([[result.y0 + poly(res_y[j]["coefficients"].flatten(), i * t_d)] for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1]) ]).flatten().tolist()
            points = np.cumsum(np.array([0] + [d["efficient_point"] for d in res_z]))
            ans_z = np.array([[result.z0 + poly(res_z[j]["coefficients"].flatten(), i * t_d)] for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1]) ]).flatten().tolist()
            ans_t = [t_0 + i * t_d for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1])]
            take_time = time.perf_counter() - start_time

        elif method_name == "proposed-huffman":
            start_time = time.perf_counter()

            ans = self.huffman_for_prop.decompress(result.compressed_data, self.K)
            ans = ProposedHandler().bit2element(ans)
            # print(ans)
            result.compressed_data = [
                [{"efficient_point": d[0], "efficient_dim": d[1], "coefficients": d[2]} for d in ans[2]],
                [{"efficient_point": d[0], "efficient_dim": d[1], "coefficients": d[2]} for d in ans[3]],
                [{"efficient_point": d[0], "efficient_dim": d[1], "coefficients": d[2]} for d in ans[4]]
            ]
            # for d in ans:
            #
            # [[d['efficient_point'], d['efficient_dim'], d['coefficients']
            # result.compressed_data = ans
            ans_t, ans_x, ans_y, ans_z, take_time = self.decompress("proposed", result)

            take_time = take_time + time.perf_counter() - start_time

        elif method_name == "proposed-filter":
            # print(result.compressed_size)
            # print(result.compressed_data)
            x_attrs = result.compressed_data["x"]
            y_attrs = result.compressed_data["y"]
            z_attrs = result.compressed_data["z"]
            result.compressed_data = result.compressed_data["compressed_data"]
            ans_tmp_t, ans_tmp_x, ans_tmp_y, ans_tmp_z, take_time = self.decompress("all-applied", result)

            ans_x = []
            ans_y = []
            ans_z = []

            t_start = 0
            for attr in x_attrs:
                t_end = round(t_start + (attr["point"] - 1) * self.d_dt, 6)

                ts, vs = [], []
                for i in range(0, len(ans_tmp_t)):
                    if ans_tmp_t[i] < t_start:
                        continue

                    elif t_end < ans_tmp_t[i]:
                        break

                    ts.append(ans_tmp_t[i])
                    vs.append(ans_tmp_x[i])

                # print(t_end)
                # print(np.arange(t_start, t_end + self.d_dt, self.d_dt))
                ans, _, _ = point2func(ts, vs, attr["dim"])
                ans_x = ans_x + [poly(ans, t)[0] for t in np.arange(t_start, t_end + self.d_dt, self.d_dt)]
                t_start = t_end + self.d_dt

            t_start = 0
            for attr in y_attrs:
                t_end = t_start + (attr["point"] - 1) * self.d_dt

                ts, vs = [], []
                for i in range(0, len(ans_tmp_t)):
                    if ans_tmp_t[i] < t_start:
                        continue

                    elif t_end < ans_tmp_t[i]:
                        break

                    ts.append(ans_tmp_t[i])
                    vs.append(ans_tmp_y[i])

                ans, _, _ = point2func(ts, vs, attr["dim"])
                ans_y = ans_y + [poly(ans, t)[0] for t in np.arange(t_start, t_end + self.d_dt, self.d_dt)]
                t_start = t_end + self.d_dt

            t_start = 0
            for attr in z_attrs:
                t_end = t_start + (attr["point"] - 1) * self.d_dt

                ts, vs = [], []
                for i in range(0, len(ans_tmp_t)):
                    if ans_tmp_t[i] < t_start:
                        continue

                    elif t_end < ans_tmp_t[i]:
                        break

                    ts.append(ans_tmp_t[i])
                    vs.append(ans_tmp_z[i])

                ans, _, _ = point2func(ts, vs, attr["dim"])
                ans_z = ans_z + [poly(ans, t)[0] for t in np.arange(t_start, t_end + self.d_dt, self.d_dt)]
                t_start = t_end + self.d_dt

            ans_t = [i * self.d_dt for i in range(0, len(ans_x))]

        elif method_name == "proposed-all":
            start_time = time.perf_counter()

            ans = self.huffman_for_prop_all.decompress(result.compressed_data, 8)
            google_polyline_sig = "".join([chr(int(ans[(8*i):(8*i + 8)], base=2)) for i in range(0, int(len(ans) / 8))])
            ans = self.d1_google_polyline.decode(google_polyline_sig, 10)
            print(ans)

            take_time = time.perf_counter() - start_time

            # ans_t = [d[3] for d in ans]
            # ans_x = [d[0] for d in ans]
            # ans_y = [d[1] for d in ans]
            # ans_z = [d[2] for d in ans]
            #
            # ans_x = [result.x0 + d for d in ans_x]
            # ans_y = [result.y0 + d for d in ans_y]
            # ans_z = [result.z0 + d for d in ans_z]
            # ans_t = [result.t0 + d for d in ans_t]

        elif method_name == "gzip":
            # pass
            # print(result)
            start_time = time.perf_counter()
            ans = result.bit2points(Bits(gzip.decompress(result.compressed_data)).bin)
            take_time = time.perf_counter() - start_time
            ans_t = [d for d in ans[0]]
            ans_x = [d for d in ans[1]]
            ans_y = [d for d in ans[2]]
            ans_z = [d for d in ans[3]]

        elif method_name == "shannon":
            return self.__decomp_shannon_or_huffman(method_name, result, self.K)

        elif method_name == "huffman":
            return self.__decomp_shannon_or_huffman(method_name, result, self.K)

        elif "google" in method_name:
            start_time = time.perf_counter()

            ans = None
            if method_name == "d3google":
                ans = self.d3_google_polyline.decode(result.compressed_data, self.google_order)
                ans_t = [result.t_d * i for i in range(0, len(ans))]

            elif method_name == "d4google":
                ans = self.d4_google_polyline.decode(result.compressed_data, self.google_order)
                ans_t = [d[3] for d in ans]

            else:
                raise NoMethodException(f"There are no method with the name {method_name}")

            take_time = time.perf_counter() - start_time
            # print(ans)
            ans_x = [d[0] for d in ans]
            ans_y = [d[1] for d in ans]
            ans_z = [d[2] for d in ans]

            ans_x = [result.x0 + d for d in ans_x]
            ans_y = [result.y0 + d for d in ans_y]
            ans_z = [result.z0 + d for d in ans_z]
            ans_t = [result.t0 + d for d in ans_t]

        elif method_name == "all-applied":
            start_time = time.perf_counter()

            # print(result.compressed_data)
            ans = self.huffman_for_all.decompress(result.compressed_data, 8)
            google_polyline_sig = "".join([chr(int(ans[(8*i):(8*i + 8)], base=2)) for i in range(0, int(len(ans) / 8))])
            ans = self.d4_google_polyline.decode(google_polyline_sig, self.google_order)

            take_time = time.perf_counter() - start_time

            ans_t = [d[3] for d in ans]
            ans_x = [d[0] for d in ans]
            ans_y = [d[1] for d in ans]
            ans_z = [d[2] for d in ans]

            ans_x = [result.x0 + d for d in ans_x]
            ans_y = [result.y0 + d for d in ans_y]
            ans_z = [result.z0 + d for d in ans_z]
            ans_t = [result.t0 + d for d in ans_t]

        else:
            raise NoMethodException(f"There are no method with the name {method_name}")

        return ans_t, ans_x, ans_y, ans_z, take_time


if __name__ == "__main__":
    x_0 = 0
    y_0 = 0
    v = 30
    a = 2
    theta = 30
    T = 10
    t_offset = 20
    d_dt = 0.1

    t = [d_t for d_t in np.arange(t_offset, T + t_offset, d_dt)]
    x = [x_0 + (v * d_t + 1/2.0 * a * d_t * d_t) * cos(radians(theta)) for d_t in t]
    y = [y_0 + (v * d_t + 1/2.0 * a * d_t * d_t) * sin(radians(theta)) for d_t in t]
    z = [0 for d_t in t]

    mcm = MCM(x, y, z, t)
    print(f"----- sample trajectory (original size: {mcm.size()})-----")
    print(f"T: {t}\n\n X: {x}\n\n Y: {y}\n\n Z: {z}")

    compress_handler = CompressHandler(
        maxdim = 10,
        mindim = 1,
        acceptance_error = 0.1,
        d_dt = d_dt,
        symbol_length = 8,
    )

    print("\n----- Proposed Method -----")
    result, comp_time,  size    = compress_handler.compress("proposed", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("proposed", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- Proposed Huffman Method -----")
    result, comp_time,  size    = compress_handler.compress("proposed-huffman", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("proposed-huffman", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- Proposed All Method -----")
    result, comp_time,  size    = compress_handler.compress("proposed-all", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("proposed-all", result)
    # print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    # print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    # print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")


    print("\n----- GZip Compression -----")
    result, comp_time,  size   = compress_handler.compress("gzip", t, x, y, z)
    # result, comp_time,  size   = compress_handler.compress("gzip", mcm.etsi_t, mcm.etsi_x, mcm.etsi_y, mcm.etsi_z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("gzip", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- Shannon Fennon Compression -----")
    result, comp_time,  size   = compress_handler.compress("shannon", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("shannon", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- Huffman Compression -----")
    result, comp_time,  size   = compress_handler.compress("huffman", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("huffman", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- D3 Google Polyline -----")
    result, comp_time, size   = compress_handler.compress("d3google", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("d3google", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- D4 Google Polyline -----")
    result, comp_time, size   = compress_handler.compress("d4google", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("d4google", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")



    print(f"\n\n\n ----- ETSI (ETSI-based size: {mcm.ETSI_size()}) ----- \n\n\n")
    etsi_calc_overhead = time.perf_counter()
    x, y, z, t = mcm.ETSI_filter(x, y, z, t)
    etsi_calc_overhead = time.perf_counter() - etsi_calc_overhead
    print(etsi_calc_overhead)

    print("\n----- GZip Compression -----")
    result, comp_time,  size   = compress_handler.compress("gzip", t, x, y, z)
    # result, comp_time,  size   = compress_handler.compress("gzip", mcm.etsi_t, mcm.etsi_x, mcm.etsi_y, mcm.etsi_z)

    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("gzip", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- Shannon Fennon Compression -----")
    result, comp_time,  size   = compress_handler.compress("shannon", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("shannon", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- Huffman Compression -----")
    result, comp_time,  size   = compress_handler.compress("huffman", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("huffman", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- D4 Google Polyline -----")
    result, comp_time, size   = compress_handler.compress("d4google", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("d4google", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

    print("\n----- All Applied -----")
    result, comp_time, size   = compress_handler.compress("all-applied", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("all-applied", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
