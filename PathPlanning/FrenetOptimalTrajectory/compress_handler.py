import numpy as np

import time
import math
from math import sin, cos, radians
import gzip
from bitstring import BitArray, Bits

from func import point2func, poly
from compression import ShannonFennonCompression, HuffmanCompression
from google_polyline import D3PolylineCodec, D4PolylineCodec

from kbm import MCM, multi_container

class NoMethodException(Exception):
    pass

class CompressHandler:
    def __init__(self, maxdim = 10, mindim = 1, acceptance_error = 0.1, d_dt = 0.1, compresslevel=9, symbol_length = 12):
        self.maxdim = maxdim
        self.mindim = mindim
        self.acceptance_error = acceptance_error / math.sqrt(3.0)
        self.d_dt = d_dt

        self.compresslevel = 9

        self.K = symbol_length
        self.shannon_fennon = ShannonFennonCompression()
        self.huffman = HuffmanCompression()

        self.google_polyline = D4PolylineCodec()

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

            x_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_x])
            y_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_y])
            z_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_z])
            mcm.compressed_size = 14 + x_size + y_size + z_size

        elif method_name == "gzip":
            bits = mcm.bit_str()
            b = BitArray(bin=bits)

            start_time = time.perf_counter()
            mcm.compressed_data = gzip.compress(b.tobytes(), compresslevel=self.compresslevel, mtime=None)
            take_time = time.perf_counter() - start_time

            mcm.compressed_size = len(mcm.compressed_data) # 1 byte per 1 character

        elif method_name == "shannon":
            return self.__comp_shannon_or_huffman(method_name, t, x, y, z)

        elif method_name == "huffman":
            return self.__comp_shannon_or_huffman(method_name, t, x, y, z)

        elif method_name == "google":
            # trajectory = [(mcm.x[i], mcm.y[i], mcm.z[i]) for i in range(0, len(t))]
            trajectory = [(mcm.x[i], mcm.y[i], mcm.z[i], mcm.t[i]) for i in range(0, len(t))]

            start_time = time.perf_counter()
            mcm.compressed_data = self.google_polyline.encode(trajectory, 2)
            take_time = time.perf_counter() - start_time
            mcm.compressed_size = len(mcm.compressed_data) # 1 byte per 1 character

        else:
            raise NoMethodException(f"There are no method with the name {method_name}")

        return mcm, take_time, mcm.compressed_size

    def __comp_shannon_or_huffman(self, method_name, t, x, y, z):
        mcm = MCM(x, y, z, t)
        bits = mcm.bit_str()
        # print(bits)
        symbols = [str(int(bits[self.K * i: self.K * (i + 1)], 2)) for i in range(0, int(len(bits) / self.K))]
        if self.K * int(len(bits) / self.K) < len(bits):
            symbols = symbols + [str(int(bits[self.K * int(len(bits) / self.K):], 2))]

        start_time = time.perf_counter()
        if method_name == "shannon":
            result = self.shannon_fennon.compress(symbols)

        elif method_name == "huffman":
            result = self.huffman.compress(symbols)

        else:
            NoMethodException(f"There are no method with the name {method_name}")

        overhead_bit = 0
        max_simbit = max([len(i.code) for i in result])
        overhead_bit = (self.K + max_simbit) * len(result)

        sym2bit = {}
        for i in result:
            sym2bit[i.symbol] = i.code

        # print(symbols)
        compressed_data = ""
        for sym in symbols:
            compressed_data = compressed_data + sym2bit[sym]

        result = compressed_data
        take_time = time.perf_counter() - start_time
        mcm.compressed_size = len(result) / 8
        mcm.compressed_data = compressed_data

        return mcm, take_time, mcm.compressed_size

    def __decomp_shannon_or_huffman(self, method_name, result):
        take_time = None
        ans = None
        ans_t, ans_x, ans_y, ans_z = None, None, None, None

        start_time = time.perf_counter()
        # print(result)
        if method_name == "shannon":
            ans = self.shannon_fennon.decompress(result.compressed_data, self.K)

        elif method_name == "huffman":
            ans = self.huffman.decompress(result.compressed_data, self.K)

        # print(result)
        ans = result.bit2points(ans)
        take_time = time.perf_counter() - start_time
        ans_t = [result.t0 + d for d in ans[0]]
        ans_x = [result.x0 + d for d in ans[1]]
        ans_y = [result.y0 + d for d in ans[2]]
        ans_z = [result.z0 + d for d in ans[3]]

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

            points = np.cumsum(np.array([0] + [d["efficient_point"] for d in res_x]))
            ans_x = np.array([[result.x0 + poly(res_x[j]["coefficients"].flatten(), t_0 + i * t_d)] for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1]) ]).flatten().tolist()
            points = np.cumsum(np.array([0] + [d["efficient_point"] for d in res_y]))
            ans_y = np.array([[result.y0 + poly(res_y[j]["coefficients"].flatten(), t_0 + i * t_d)] for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1]) ]).flatten().tolist()
            points = np.cumsum(np.array([0] + [d["efficient_point"] for d in res_z]))
            ans_z = np.array([[result.z0 + poly(res_z[j]["coefficients"].flatten(), t_0 + i * t_d)] for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1]) ]).flatten().tolist()
            ans_t = [t_0 + i * t_d for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1])]
            take_time = time.perf_counter() - start_time

        elif method_name == "gzip":
            # pass
            # print(result)
            start_time = time.perf_counter()
            ans = result.bit2points(Bits(gzip.decompress(result.compressed_data)).bin)
            take_time = time.perf_counter() - start_time
            ans_t = [result.t0 + d for d in ans[0]]
            ans_x = [result.x0 + d for d in ans[1]]
            ans_y = [result.y0 + d for d in ans[2]]
            ans_z = [result.z0 + d for d in ans[3]]

        elif method_name == "shannon":
            return self.__decomp_shannon_or_huffman(method_name, result)

        elif method_name == "huffman":
            return self.__decomp_shannon_or_huffman(method_name, result)

        elif method_name == "google":
            start_time = time.perf_counter()
            ans = self.google_polyline.decode(result.compressed_data, 2)
            take_time = time.perf_counter() - start_time
            # print(ans)
            ans_x = [d[0] for d in ans]
            ans_y = [d[1] for d in ans]
            ans_z = [d[2] for d in ans]
            ans_t = [d[3] for d in ans]

            ans_x = [result.x0 + d for d in ans_x]
            ans_y = [result.y0 + d for d in ans_y]
            ans_z = [result.z0 + d for d in ans_z]
            ans_t = [result.t0 + d for d in ans_t]
            # ans_t = [result.t0 + i * result.t_d for i in range(0, len(ans_x))]

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
    d_dt = 0.1

    t = [d_t for d_t in np.arange(0, T, d_dt)]
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
    )

    print("\n----- Proposed Method -----")
    result, comp_time,  size    = compress_handler.compress("proposed", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("proposed", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")

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

    print("\n----- Google Polyline -----")
    result, comp_time, size   = compress_handler.compress("google", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("google", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")



    print(f"\n\n\n ----- ETSI (ETSI-based size: {mcm.ETSI_size()}) ----- \n\n\n")
    x, y, z, t = mcm.ETSI_filter(x, y, z, t)

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

    print("\n----- Google Polyline -----")
    result, comp_time, size   = compress_handler.compress("google", t, x, y, z)
    ans_t, ans_x, ans_y, ans_z, decomp_time = compress_handler.decompress("google", result)
    print(f"size: {size}, comp_time: {comp_time}, decomp_time: {decomp_time}\n result: {result.compressed_data}")
    print(f"T: {ans_t}\n\n X: {ans_x}\n\n Y: {ans_y}\n\n Z: {ans_z}")
    print(f"\nerror: {[math.sqrt((x[i] - ans_x[i])**2 + (y[i] - ans_y[i])**2 + (z[i] - ans_z[i])**2) for i in range(0, len(ans_t))]}")
