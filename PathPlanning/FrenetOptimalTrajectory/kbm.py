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

from func import point2func, poly, mydist
from compression import ShannonFennonCompression, HuffmanCompression
from google_polyline import PolylineCodec

plt.rcParams["font.size"] = 18

class ProposedHandler:
    def __init__(self):
        self.offset = 10

    def element2bit(self, t_interval, error_th, dtc_x, dtc_y, dtc_z):
        result = ""

        # print(f"{t_interval}, {error_th}, {dtc_x}, {dtc_y}, {dtc_z}")
        # print(t_interval)
        # print(f"{int( t_interval * self.offset):08b}")
        # result += struct.pack('<h', int( t_interval * self.offset))
        # result += struct.pack('<h', int( error_th * self.offset))
        # result += struct.pack('<h', int( len(dtc_x) * self.offset))
        # result += struct.pack('<h', int( len(dtc_y) * self.offset))
        # result += struct.pack('<h', int( len(dtc_z) * self.offset))

        # result += f"{int( t_interval * self.offset):08b}"
        # print(result)
        # result += f"{int( error_th * self.offset):08b}"
        # print(result)
        # result += f"{int( len(dtc_x) * self.offset):08b}"
        # print(result)
        # result += f"{int( len(dtc_y) * self.offset):08b}"
        # print(result)
        # result += f"{int( len(dtc_z) * self.offset):08b}"

        # print("".join([f"{x:08b}" for x in struct.pack('<h', int( t_interval * self.offset))[::-1]])[8:16])
        result += "".join([f"{x:08b}" for x in struct.pack('<h', int( t_interval * self.offset))[::-1]])[8:16]
        # print(result)
        result += "".join([f"{x:08b}" for x in struct.pack('<h', int( error_th * self.offset))[::-1]])[8:16]
        # print(result)
        result += "".join([f"{x:08b}" for x in struct.pack('<h', int( len(dtc_x) * self.offset))[::-1]])[8:16]
        # print(result)
        result += "".join([f"{x:08b}" for x in struct.pack('<h', int( len(dtc_y) * self.offset))[::-1]])[8:16]
        # print(result)
        result += "".join([f"{x:08b}" for x in struct.pack('<h', int( len(dtc_z) * self.offset))[::-1]])[8:16]
        # print(result)

        for dtc in [dtc_x, dtc_y, dtc_z]:
            for elm in dtc:
                # print("".join([f"{x:08b}" for x in struct.pack('<h', int(elm[0]))[::-1]]))
                result += "".join([f"{x:08b}" for x in struct.pack('<h', int(elm[0]))[::-1]])[8:16]
                # print(result)
                result += "".join([f"{x:08b}" for x in struct.pack('<h', int(elm[1]))[::-1]])[8:16]
                # print(result)

                for coef in elm[2]:
                    result += "".join([f"{x:08b}" for x in struct.pack('<f', coef)[::-1]])[0:32]
                    # result += f"{struct.pack('<f', coef)}"
                    # print(result)

        return result

    def bit2element(self, bits):
        bit_str = bits[:]

        # print(bit_str)
        # print(BitArray(bin="0" * 8 + bit_str[0:8]).int)
        t_interval = float(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[0:8]).int))[0] / self.offset)
        error_th = float(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[8:16]).int))[0] / self.offset)
        dtc_x_num = int(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[16:24]).int))[0] / self.offset)
        dtc_y_num = int(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[24:32]).int))[0] / self.offset)
        dtc_z_num = int(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[32:40]).int))[0] / self.offset)

        # t_interval = int(BitArray(bin="0" * 8 + bit_str[0:8]).int / self.offset)
        # error_th = int(BitArray(bin="0" * 8 + bit_str[8:16]).int / self.offset)
        # dtc_x_num = int(BitArray(bin="0" * 8 + bit_str[16:24]).int / self.offset)
        # dtc_y_num = int(BitArray(bin="0" * 8 + bit_str[24:32]).int / self.offset)
        # dtc_z_num = int(BitArray(bin="0" * 8 + bit_str[32:40]).int / self.offset)


        dtc_x = []
        dtc_y = []
        dtc_z = []

        bit_str = bit_str[40:]
        # print(bit_str)
        # print(bit_str[])
        for i in range(0, dtc_x_num):
            elm = []

            # print(bit_str[0:8])
            elm.append(int(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[0:8]).int))[0]))
            # print(elm)
            elm.append(int(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[8:16]).int))[0]))

            coeffs = []
            for j in range(0, elm[1]):
                coeffs.append(struct.unpack('f', struct.pack('I', int(bit_str[(16 + 32 * j):(16 + 32 * (j + 1))], 2)))[0])

            elm.append(np.array(coeffs))

            dtc_x.append(elm)
            bit_str = bit_str[(8 * 2 + 32 * len(coeffs))::]
            # print(bit_str)

        # print("!!!")
        # print(bit_str)
        for i in range(0, dtc_y_num):
            elm = []

            elm.append(int(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[0:8]).int))[0]))
            elm.append(int(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[8:16]).int))[0]))

            coeffs = []
            for j in range(0, elm[1]):
                coeffs.append(struct.unpack('f', struct.pack('I', int(bit_str[(16 + 32 * j):(16 + 32 * (j + 1))], 2)))[0])

            elm.append(np.array(coeffs))

            dtc_y.append(elm)
            bit_str = bit_str[(8 * 2 + 32 * len(coeffs)):]
            # print(bit_str)

        # print("!!!")
        # print(bit_str)
        for i in range(0, dtc_z_num):
            elm = []

            elm.append(int(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[0:8]).int))[0]))
            elm.append(int(struct.unpack('h', struct.pack('h', BitArray(bin="0" * 8 + bit_str[8:16]).int))[0]))

            # print(elm)
            coeffs = []
            for j in range(0, elm[1]):
                coeffs.append(struct.unpack('f', struct.pack('I', int(bit_str[(16 + 32 * j):(16 + 32 * (j + 1))], 2)))[0])

            elm.append(np.array(coeffs))

            dtc_z.append(elm)
            bit_str = bit_str[(8 * 2 + 32 * len(coeffs)):]
            # print(bit_str)

        # print("!!!")
        # print(bit_str)
        return t_interval, error_th, dtc_x, dtc_y, dtc_z

class MCM:
    def __init__(self, x, y, z, t):
        assert(len(x) == len(y))
        assert(len(y) == len(z))
        assert(len(z) == len(t))

        self.t0 = t[0]
        self.t_d = t[1] - t[0]
        self.x0 = x[0]
        self.y0 = y[0]
        self.z0 = z[0]

        self.x = [x[i] - self.x0 for i in range(0, len(t))]
        self.y = [y[i] - self.y0 for i in range(0, len(t))]
        self.z = [z[i] - self.z0 for i in range(0, len(t))]
        self.t = [t[i] - self.t0 for i in range(0, len(t))]

        # self.etsi_x, self.etsi_y, self.etsi_z, self.etsi_t = self.ETSI_filter(self.x, self.y, self.z, self.t)

        # ----- convert float to shot int -----
        self.factor = 20
        self.short_x = [int(d * self.factor) for d in self.x]
        self.short_y = [int(d * self.factor) for d in self.y]
        self.short_z = [int(d * self.factor) for d in self.z]
        self.short_t = [int(d * self.factor) for d in self.t]

        # ----- express as short int -----
        self.t_bit = 16
        self.x_bit = 16
        self.y_bit = 16
        self.z_bit = 16

        self.size_1_point = self.t_bit + self.x_bit + self.y_bit + self.z_bit

        self.compressed_data = None
        self.compressed_size = None
        self.compressed_overhead = 0

        self.ITS_HEADER_OVERHEAD = 0

    def cossim(self, x, y):
        return np.dot(x,y)/(np.linalg.norm(x)*np.linalg.norm(y))

    def ETSI_size(self):
        _, _, _, _ = self.ETSI_filter(self.x, self.y, self.z, self.t)

        return self.size_with_header_overhead(len(_) * self.size_1_point / 8.0)

    def ETSI_filter(self, x, y, z, t):
        result = []

        # print(t)
        points = [(t[i], x[i], y[i], z[i]) for i in range(0, len(t))]
        speeds = [np.linalg.norm(np.array(points[i][1:3]) - np.array(points[i - 1][1:3])) / (points[i][0] - points[i-1][0]) for i in range(1, len(points))]
        heads = [math.degrees(self.cossim(np.array(points[i][1:3]) - np.array(points[i-1][1:3]), np.array(points[i-1][1:3]) - np.array(points[i-2][1:3]))) for i in range(2, len(points))]
        speeds = speeds + [speeds[-1]]
        heads = heads + [heads[-1], heads[-1]]

        last_index = 0
        for i in range(0, len(points)):
            if len(result) <= 0:
                result.append(points[i])

            else:
                dT = points[i][0] - points[last_index][0]
                dP = np.linalg.norm(np.array(points[i][1:3]) - np.array(points[last_index][1:3]))
                dV = speeds[i] - speeds[last_index]
                dH = heads[i] - heads[last_index]

                # print(f"{dT}, {dP}, {dV}, {dH}")
                if 0.1 <= dT and (4 <= dP or 0.5 <= dV or 4 <= dH or 1.0 <= dT):
                    # print("!!!")
                    last_index = i
                    result.append(points[i])

        return [p[1] for p in result], [p[2] for p in result], [p[3] for p in result], [p[0] for p in result]

    def size(self):
        # return {ITS_PDU_HEADER (4 byte) + (size of t0, x0, y0, z0)}(20 Byte) + size_1_point * number of waypoints
        return self.size_with_header_overhead(self.size_1_point * len(self.t) / 8.0)

    def size_with_header_overhead(self, trajectory_container_size):
        return trajectory_container_size + self.ITS_HEADER_OVERHEAD

    def bit_str(self):
        result = ''

        for i in range(0, len(self.t)):
            # result = result + "".join([f"{x:08b}" for x in struct.pack('<h', self.short_t[i])[::-1]])[0:self.t_bit]
            # result = result + "".join([f"{x:08b}" for x in struct.pack('<f', self.x[i])[::-1]])[0:self.x_bit]
            # result = result + "".join([f"{x:08b}" for x in struct.pack('<f', self.y[i])[::-1]])[0:self.y_bit]
            # result = result + "".join([f"{x:08b}" for x in struct.pack('<f', self.z[i])[::-1]])[0:self.z_bit]

            result = result + "".join([f"{x:08b}" for x in struct.pack('<h', self.short_t[i])[::-1]])[0:self.t_bit]
            result = result + "".join([f"{x:08b}" for x in struct.pack('<h', self.short_x[i])[::-1]])[0:self.x_bit]
            result = result + "".join([f"{x:08b}" for x in struct.pack('<h', self.short_y[i])[::-1]])[0:self.y_bit]
            result = result + "".join([f"{x:08b}" for x in struct.pack('<h', self.short_z[i])[::-1]])[0:self.z_bit]

        return result

    def bit2points(self, bit_str):
        # one_point_bit_length (OPBL)
        OPBL = self.size_1_point

        # assert (len(bit_str) % OPBL == 0)
        bit_str = bit_str[0:len(bit_str)-(len(bit_str) % OPBL)]

        be = 0
        en = be + self.t_bit
        t_bits = [bit_str[OPBL * i + be: OPBL * i + en] for i in range(0, int(len(bit_str) / OPBL))]

        be = en
        en = en + self.x_bit
        x_bits = [bit_str[OPBL * i + be: OPBL * i + en] for i in range(0, int(len(bit_str) / OPBL))]

        be = en
        en = en + self.y_bit
        y_bits = [bit_str[OPBL * i + be: OPBL * i + en] for i in range(0, int(len(bit_str) / OPBL))]

        be = en
        en = en + self.z_bit
        z_bits = [bit_str[OPBL * i + be: OPBL * i + en] for i in range(0, int(len(bit_str) / OPBL))]

        # print(t_bits)
        # t = [self.t0 + struct.unpack('f', struct.pack('I', int(bit + '0' * (32 - self.t_bit), 2)))[0] for bit in t_bits]
        # x = [self.x0 + struct.unpack('f', struct.pack('I', int(bit + '0' * (32 - self.x_bit), 2)))[0] for bit in x_bits]
        # y = [self.y0 + struct.unpack('f', struct.pack('I', int(bit + '0' * (32 - self.y_bit), 2)))[0] for bit in y_bits]
        # z = [self.z0 + struct.unpack('f', struct.pack('I', int(bit + '0' * (32 - self.z_bit), 2)))[0] for bit in z_bits]

        # for bit in x_bits:
        #     print(bit + '0' * (16 - self.x_bit))
        #     print(BitArray(bin=(bit + '0' * (16 - self.t_bit))).int)

        t = [struct.unpack('h', struct.pack('h', BitArray(bin=(bit + '0' * (16 - self.t_bit))).int ))[0] for bit in t_bits]
        x = [struct.unpack('h', struct.pack('h', BitArray(bin=(bit + '0' * (16 - self.x_bit))).int ))[0] for bit in x_bits]
        y = [struct.unpack('h', struct.pack('h', BitArray(bin=(bit + '0' * (16 - self.y_bit))).int ))[0] for bit in y_bits]
        z = [struct.unpack('h', struct.pack('h', BitArray(bin=(bit + '0' * (16 - self.z_bit))).int ))[0] for bit in z_bits]

        # ----- convert short int to float -----
        t = [self.t0 + d / self.factor for d in t]
        x = [self.x0 + d / self.factor for d in x]
        y = [self.y0 + d / self.factor for d in y]
        z = [self.z0 + d / self.factor for d in z]

        return t, x, y, z

# total_time = 0
# def multi_container2(t, x, max_dim, error_th, start_dim=1):


def multi_container(t, x, max_dim, error_th, start_dim=1):
    # global total_time
    efficiency = 0
    result = {
        "start_time": t[0],
        "efficient_point": 0,
        "efficient_dim":  0,
        "coefficients": [],
        "ans": []
    }

    assert(start_dim < max_dim + 1)
    for dim in range(start_dim, max_dim + 1):

        # ----- fast version -----
        point = min([max([dim, int(efficiency * dim + 1)]), len(t)])
        coef, diff_x_max, ans_x = point2func(t[:point], x[:point], dim)
        if diff_x_max <= error_th:
            for point in range(max([dim, int(efficiency * dim + 1)]), len(t) + 1):
                if error_th < math.fabs(x[point-1] - poly(coef, t[point-1])):
                    coef, diff_x_max, ans_x = point2func(t[:point], x[:point], dim)

                    if error_th < diff_x_max:
                        break

                    else:
                        continue

                else:
                    continue

        if error_th < diff_x_max:
            # print(f"{efficiency}, {(point - 1) / float(dim)}")
            if efficiency < (point - 1) / float(dim):
                # print(f"{point}, {len(t[(point):])}")
                t_coef, t_diff_x_max, t_ans_x = point2func(t[:point - 1], x[:point - 1], dim)
                # print(f"over error: {dim}, {point - 1}, {t_diff_x_max}")

                efficiency = (point - 1) / float(dim)
                result["efficient_point"] = point - 1
                result["efficient_dim"] = dim
                result["coefficients"] = t_coef
                result["ans"] = t_ans_x
                result["diff_max"] = t_diff_x_max
                result["error_th"] = error_th
                result["type"] = "less error"

                assert(len(t_ans_x) == (point - 1))
                continue

            else:
                continue

        elif point == len(t):
            # print(f"{efficiency}, {(point) / float(dim)}")
            if efficiency < (point) / float(dim):
                coef, diff_x_max, ans_x = point2func(t[:point], x[:point], dim)
                # print(f"{point}, {len(t[(point):])}")
                # coef, diff_x_max, ans_x = point2func(t[:point], x[:point], dim)
                # print(f"last: {dim}, {point}, {diff_x_max}")
                efficiency = point / float(dim)
                result["efficient_point"] = point
                result["efficient_dim"] = dim
                result["coefficients"] = coef
                result["ans"] = ans_x
                result["diff_max"] = diff_x_max
                result["error_th"] = error_th
                result["type"] = "last"

                assert(len(ans_x) == (point))
                continue

            else:
                continue

        else:
            print(point)
            print(len(t))
            raise Exception("unexpected state.")

        # ----- slow version -----
        # for point in range(max([dim, int(efficiency * dim + 1)]), len(t) + 1):
        #     # print(t)
        #     # st = time.perf_counter()
        #     coef, diff_x_max, ans_x = point2func(t[:point], x[:point], dim)
        #     # total_time += time.perf_counter() - st
        #     # print(total_time)
        #
        #     # print(f"middle: {point}, {len(t)}, {point == len(t)}")
        #
        #     if error_th < diff_x_max:
        #         # print(f"{efficiency}, {(point - 1) / float(dim)}")
        #         if efficiency < (point - 1) / float(dim):
        #             # print(f"{point}, {len(t[(point):])}")
        #             t_coef, t_diff_x_max, t_ans_x = point2func(t[:point - 1], x[:point - 1], dim)
        #             # print(f"over error: {dim}, {point - 1}, {t_diff_x_max}")
        #
        #             efficiency = (point - 1) / float(dim)
        #             result["efficient_point"] = point - 1
        #             result["efficient_dim"] = dim
        #             result["coefficients"] = t_coef
        #             result["ans"] = t_ans_x
        #             result["diff_max"] = t_diff_x_max
        #             result["error_th"] = error_th
        #             result["type"] = "less error"
        #
        #             assert(len(t_ans_x) == (point - 1))
        #             break
        #
        #         else:
        #             break
        #
        #     elif point == len(t):
        #         # print(f"{efficiency}, {(point) / float(dim)}")
        #         if efficiency < (point) / float(dim):
        #             # print(f"{point}, {len(t[(point):])}")
        #             # coef, diff_x_max, ans_x = point2func(t[:point], x[:point], dim)
        #             # print(f"last: {dim}, {point}, {diff_x_max}")
        #             efficiency = point / float(dim)
        #             result["efficient_point"] = point
        #             result["efficient_dim"] = dim
        #             result["coefficients"] = coef
        #             result["ans"] = ans_x
        #             result["diff_max"] = diff_x_max
        #             result["error_th"] = error_th
        #             result["type"] = "last"
        #
        #             assert(len(ans_x) == (point))
        #             break
        #
        #         else:
        #             break
        #
        #
        #     elif diff_x_max <= error_th:
        #         # t_coef, t_diff_x_max, t_ans_x = coef, diff_x_max, ans_x
        #         # print(f"less error: {dim}, {point}, {diff_x_max}")
        #         continue
        #
        #     else:
        #         raise Exception("unexpected state.")

    if result["efficient_point"] <= 0:
        # print(result)
        # print(x)
        # print(t)
        # print("!!!!")
        # print(efficiency)
        return []

    if result["efficient_point"] == len(x):
        # print("----")
        return [result]

    else:
        point = result["efficient_point"]
        # print(f"last: {point}, {len(t[(point):])}")
        return [result] + multi_container(t[(point):], x[(point):], max_dim, error_th, start_dim)


def clip(v, v_min, v_max):
    if v < v_min:
        return v_min

    elif v_max < v:
        return v_max

    else:
        return v

def kbm(x_c, y_c, v_c, theta_c, acc, stear, d_t, veh_length):
    # cite from: https://shuffleai.blog/blog/Simple_Understanding_of_Kinematic_Bicycle_Model.html

    x_next = x_c + v_c * cos(theta_c) * d_t
    y_next = y_c + v_c * sin(theta_c) * d_t
    v_next = clip(v_c + acc * d_t, 0, float('inf'))
    theta_next = theta_c + v_c * (tan(stear) / veh_length) * d_t

    # print(f"x: {x_next}, y: {y_next}, v: {v_next}, acc: {acc}, theta: {math.degrees(theta_next)}")
    return x_next, y_next, v_next, theta_next

class Car:
    def __init__(self, x_init, y_init, v_init, theta_init, s_init, d_t=0.1, veh_length=5, v_max=200.0/3600.0, s_max = 45, sv_max = 0.1):
        self.x_log = [x_init]
        self.y_log = [y_init]
        self.v_log = [v_init]
        self.t_log = [0]
        self.s_log = [s_init]
        self.theta_log = [theta_init]

        self.d_t = d_t
        self.veh_length = veh_length
        self.vmax = v_max
        self.s_max = s_max
        self.sv_max = sv_max


    def path(self):
        return self.x_log, self.y_log, self.t_log

    def path_by_delta_t(self, d_t):
        x_r = [self.x_log[0]]
        y_r = [self.y_log[0]]
        t_r = [self.t_log[0]]

        t_pivot = self.t_log[0]
        for i in range(0, len(self.t_log)):
            if d_t <= self.t_log[i] - t_pivot:
                t_pivot = self.t_log[i]
                x_r.append(self.x_log[i])
                y_r.append(self.y_log[i])
                t_r.append(self.t_log[i])

        return x_r, y_r, t_r


    def path_by_point_num(self, point_num, skip_num):
        return (self.x_log[0::skip_num])[0:point_num], (self.y_log[0::skip_num])[0:point_num], (self.t_log[0::skip_num])[0:point_num]

    def path_filterd_by_ETSI(self):
        x_r = [self.x_log[0]]
        y_r = [self.y_log[0]]
        t_r = [self.t_log[0]]
        v_r = [self.v_log[0]]
        theta_r = [self.theta_log[0]]

        min_dt = 1.0

        for i in range(0, len(self.t_log)):
            dt = math.fabs(self.t_log[i] - t_r[-1])
            dv = math.fabs(self.v_log[i] - v_r[-1])
            dh = math.fabs(math.degrees(self.theta_log[i]) - math.degrees(theta_r[-1]))
            # print(self.x_log[i])
            # print(self.y_log[i])
            # print(np.array([float(self.x_log[i]), float(self.y_log[i])]))
            # print(np.array([float(x_r[-1]), float(y_r[-1])]))
            # print(np.array(float(self.x_log[i]), float(self.y_log[i])))
            # print(np.array(float(x_r[-1]), float(y_r[-1])))
            dd = mydist([float(self.x_log[i]), float(self.y_log[i])], [float(x_r[-1]), float(y_r[-1])])

            if 0.1 <= self.t_log[i] - t_r[-1]:
                if 1.0 <= dt or 0.5 <= dv or 4 <= dh or 4 <= dd:
                    # print(f"{dt}, {dv}, {dh}, {dd}")
                    x_r.append(self.x_log[i])
                    y_r.append(self.y_log[i])
                    t_r.append(self.t_log[i])
                    v_r.append(self.v_log[i])
                    theta_r.append(self.theta_log[i])

                    if dt < min_dt:
                        min_dt = dt

        return x_r, y_r, t_r, min_dt

    def latest_time(self):
        return self.t_log[-1]

    def generate_path_by_delta_t(self, d_t, acc, stear):
        x_next, y_next, v_next, theta_next = kbm(
            self.x_log[-1],
            self.y_log[-1],
            self.v_log[-1],
            self.theta_log[-1],
            acc,
            stear,
            d_t,
            self.veh_length,
        )

        self.x_log.append(x_next)
        self.y_log.append(y_next)
        self.v_log.append(clip(v_next, 0.0, self.vmax))
        self.theta_log.append(theta_next)
        self.t_log.append(self.latest_time() + d_t)
        self.s_log.append(stear)
        # print(self.t_log)


    def generate_path_by_rotate(self, acc, rotate_radian, force_stop_time=float("inf")):
        theta_init = self.theta_log[-1]
        target_theta = self.theta_log[-1] + rotate_radian
        origin_stear = arctan((target_theta - self.theta_log[-1]) * (self.veh_length / (self.v_log[-1] * self.d_t)))

        if force_stop_time <= self.t_log[-1] + self.d_t:
            return 0

        # print(f"origin_stear: {origin_stear}, sv_max: {self.sv_max}")
        if -self.sv_max * self.d_t <= origin_stear and origin_stear <= self.sv_max * self.d_t:
            self.generate_path_by_delta_t(self.d_t, acc, origin_stear)
            self.generate_path_by_delta_t(self.d_t, acc, 0)

            return 0

        else:
            _, _, _, theta_next_high = kbm(
                self.x_log[-1],
                self.y_log[-1],
                self.v_log[-1],
                self.theta_log[-1],
                acc,
                self.s_log[-1] + self.sv_max * self.d_t,
                self.d_t,
                self.veh_length,
            )

            _, _, _, theta_next_midle = kbm(
                self.x_log[-1],
                self.y_log[-1],
                self.v_log[-1],
                self.theta_log[-1],
                acc,
                self.s_log[-1],
                self.d_t,
                self.veh_length,
            )

            _, _, _, theta_next_low = kbm(
                self.x_log[-1],
                self.y_log[-1],
                self.v_log[-1],
                self.theta_log[-1],
                acc,
                self.s_log[-1] - self.sv_max * self.d_t,
                self.d_t,
                self.veh_length,
            )

        # print(f"target_theta: {target_theta}, stear: {self.s_log[-1]}, theta: {self.theta_log[-1]}, rotate_radian: {rotate_radian}")
        # print(f"theta_next_high: {theta_next_high}, theta_next_midle: {theta_next_midle}, theta_next_low: {theta_next_low}")

        n = 1
        if self.s_log[-1] != 0:
            n = math.fabs(self.s_log[-1] / (self.d_t * self.d_t))

        if theta_next_high <= self.theta_log[-1] + rotate_radian / n:
            self.generate_path_by_delta_t(self.d_t, acc, clip(self.s_log[-1] + self.sv_max * self.d_t, -self.s_max, self.s_max))

        elif theta_next_midle <= self.theta_log[-1] + rotate_radian / n:
            self.generate_path_by_delta_t(self.d_t, acc, clip(self.s_log[-1], -self.s_max, self.s_max))

        else:
            self.generate_path_by_delta_t(self.d_t, acc, clip(self.s_log[-1] - self.sv_max * self.d_t, -self.s_max, self.s_max))

        return self.generate_path_by_rotate(acc, rotate_radian - (self.theta_log[-1] - self.theta_log[-2]), force_stop_time)


    def generate_path_until_time(self, T, acc, stear):
        while self.latest_time() <= T:
            self.generate_path_by_delta_t(self.d_t, acc, stear)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Kinematic Model')
    parser.add_argument('-vi', '--v_init', type=float, default=0.0)
    parser.add_argument('-ai', '--a_init', type=float, default=0.0)
    parser.add_argument('-si', '--s_init', type=float, default=0.0)
    parser.add_argument('-ti', '--t_init', type=float, default=0.0)
    parser.add_argument('-vmax', '--v_max', type=float, default=int(200 / 3600.0 * 1000.0))
    parser.add_argument('-smax', '--stear_max', type=float, default=45.0)
    # parser.add_argument('-accmax', '--acc_max', type=float, default=2.6)
    # parser.add_argument('-dccmax', '--dcc_max', type=float, default=4.5)
    parser.add_argument('-eth', '--error_th', type=float, default=0.2)
    parser.add_argument('--end', type=float, default=10.0)
    parser.add_argument('--dt', type=float, default=0.1)
    parser.add_argument('--d_dt', type=float, default=0.1)
    parser.add_argument('--np', type=int, default=40)
    parser.add_argument('-aui', '--a_update_interval', type=float, default=100)
    parser.add_argument('-sui', '--stear_update_interval', type=float, default=100)
    parser.add_argument('-rui', '--rotate_update_interval', type=float, default=100)
    parser.add_argument('-rv', '--rotate_value', type=float, default=0)
    parser.add_argument('-rt', '--rotate_type', action='store_true')
    # parser.add_argument('-dim', type=int, default=20)
    parser.add_argument('-maxdim', type=int, default=10)
    parser.add_argument('-mindim', type=int, default=1)


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
            # rotate_update_time = car.latest_time() + args.rotate_update_interval
            rotate_update_time = max([car.latest_time(), rotate_update_time + args.rotate_update_interval])

            if args.rotate_type == True:
                rotate_type = -1 * rotate_type

            # print(f"time: {car.latest_time()}, type: rotate, next_time: {rotate_update_time}")

        else:
            car.generate_path_by_delta_t(args.dt, a, stear)

            # print(f"time: {car.latest_time()}, type: strait, next_time: {rotate_update_time}")


    etsi_x, etsi_y, etsi_t, min_dt = car.path_filterd_by_ETSI()
    print(min_dt)
    print(int(round(min_dt, 2) / args.dt))
    print(args.d_dt / args.dt)
    # x, y, t = car.path_by_point_num(args.np, int(round(min_dt, 2) / args.dt))
    x, y, t = car.path_by_point_num(args.np, int(args.d_dt / args.dt))

    d = np.diff(np.array(x)) / args.dt
    for i in range(0, 20):
        print(f"{d}, {np.absolute(d).sum()}")
        d = np.diff(np.array(d)) / args.dt
    # print(x)
    # print(y)
    # print(t)

    # _, diff_x_max, diff_y_max, diff_max, ans_x, ans_y = point2func(t, x, y, args.dim)
    # _, diff_x_max, ans_x = point2func(t, x, args.dim)
    # _, diff_y_max, ans_y = point2func(t, y, args.dim)
    #
    # d_x = np.array(x) - np.array(ans_x)
    # d_y = np.array(y) - np.array(ans_y)
    # diff_max = max([math.sqrt(d_x[i] * d_x[i] + d_y[i] * d_y[i]) for i in range(0, len(d_x))])

    # print('{:.20f}, {:.20f}, {:}'.format(diff_x_max, diff_y_max, args.__dict__))
    # file_name = "_".join([f"{k}_{v}" for k, v in args.__dict__.items()])
    # print('{:.20f}, {:}'.format(diff_max, file_name))

    # fig = plt.figure()
    # plt.scatter(x, y, c="black", label="grand truth")
    # plt.scatter(ans_x, ans_y, c="orange", label="estimate")
    # plt.legend()
    # plt.xlabel("x (m)")
    # plt.ylabel("y (m)")
    # fig.savefig(f"path_img/{file_name}.png")
    #
    # fig = plt.figure()
    # plt.scatter(t, x, c="black", label="grand truth")
    # plt.scatter(t, ans_x, c="orange", label="estimate")
    # plt.legend()
    # plt.xlabel("t (sec)")
    # plt.ylabel("x (m)")
    # fig.savefig(f"path_img/{file_name}_x.png")
    #
    # fig = plt.figure()
    # plt.scatter(t, y, c="black", label="grand truth")
    # plt.scatter(t, ans_y, c="orange", label="estimate")
    # plt.legend()
    # plt.xlabel("t (sec)")
    # plt.ylabel("y (m)")
    # fig.savefig(f"path_img/{file_name}_y.png")

    file_name = "_".join([f"{k}_{v}" for k, v in args.__dict__.items()])
    # args = [
    #     (t[:], x[:], args.maxdim, math.sqrt(args.error_th/3)),
    #     (t[:], y[:], args.maxdim, math.sqrt(args.error_th/3)),
    #     (t[:], [0] * len(t), args.maxdim, math.sqrt(args.error_th/3)),
    # ]

    # ----- proposed -----
    acceptance_error = args.error_th / math.sqrt(3.0)
    z = [0] * len(t)

    # --- compress ---
    start_time = time.perf_counter()
    res_x = multi_container(t[:], x[:], args.maxdim, acceptance_error, args.mindim)
    res_y = multi_container(t[:], y[:], args.maxdim, acceptance_error, args.mindim)
    res_z = multi_container(t[:], z[:], args.maxdim, acceptance_error, args.mindim)
    take_time = time.perf_counter() - start_time

    # --- decompress ---
    points = np.cumsum(np.array([0] + [d["efficient_point"] for d in res_x]))
    ans_x = [[poly(res_x[j]["coefficients"].flatten(), i * args.d_dt)] for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1]) ]
    points = np.cumsum(np.array([0] + [d["efficient_point"] for d in res_y]))
    ans_y = [[poly(res_y[j]["coefficients"].flatten(), i * args.d_dt)] for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1]) ]
    points = np.cumsum(np.array([0] + [d["efficient_point"] for d in res_z]))
    ans_z = [[poly(res_z[j]["coefficients"].flatten(), i * args.d_dt)] for j in range(0, len(points) - 1) for i in range(points[j], points[j + 1]) ]


    print("----- original points -----")
    print((t, x, y, z))

    # ans_x = sum([d["ans"] for d in res_x], [])
    start_point_x = np.array([0] + [len(d["ans"]) for d in res_x])
    start_point_x = list(np.cumsum(start_point_x))
    # print(start_point_x)
    start_time_x = [t[d] for d in start_point_x[:-1]]
    start_point_x = [ans_x[d] for d in start_point_x[:-1]]
    # print(start_time_x)
    # print(start_point_x)

    # ans_y = sum([d["ans"] for d in res_y], [])
    start_point_y = np.array([0] + [len(d["ans"]) for d in res_y])
    start_point_y = list(np.cumsum(start_point_y))
    # print(start_point_y)
    start_time_y = [t[d] for d in start_point_y[:-1]]
    start_point_y = [ans_y[d] for d in start_point_y[:-1]]
    # print(start_time_y)
    # print(start_point_y)

    # print(len(x))
    # print(len(y))
    # print(len(ans_x))
    # print(len(ans_y))
    # print([d["efficient_dim"] for d in res_x])
    # print([d["efficient_dim"] for d in res_y])
    colors = ["orange", "green"]

    # fig = plt.figure()
    fig = plt.figure(figsize=(8,6))
    plt.scatter(x, y, c="black", label="original waypoint")
    plt.scatter(ans_x, ans_y, c="orange", label="approximated waypoint")
    plt.legend()
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    fig.savefig(f"path_img/{file_name}.pdf", transparent=True)

    # fig = plt.figure()
    fig = plt.figure(figsize=(8,6))
    plt.scatter(t, x, c="black", label="original waypoint")
    plt.scatter(t, ans_x, c="orange", label="approximated waypoint")
    plt.scatter(start_time_x, start_point_x, c="green", label="start point of polynomial")
    plt.legend()
    plt.xlabel("passing time (sec)")
    plt.ylabel("x (m)")
    fig.savefig(f"path_img/{file_name}_x.pdf", transparent=True)

    # fig = plt.figure()
    fig = plt.figure(figsize=(8,6))
    plt.scatter(t, y, c="black", label="original waypoint")
    plt.scatter(t, ans_y, c="orange", label="approximated waypoint")
    plt.scatter(start_time_y, start_point_y, c="green", label="start point of polynomial")
    plt.legend()
    plt.xlabel("passing time (sec)")
    plt.ylabel("y (m)")
    fig.savefig(f"path_img/{file_name}_y.pdf", transparent=True)

    # print(min_dt)
    # etsi_x, etsi_y, etsi_t = x, y, t

    print("----- prev 1 points -----")
    # ----- previous 1 -----
    mcm = MCM(etsi_x, etsi_y, [0 for _ in range(0, len(etsi_t))], etsi_t)
    bits = mcm.bit_str()
    b = BitArray(bin=bits)
    # print(b)
    print(mcm.size())
    # print(mcm.bit2points(bits))

    # gz = gzip.open(b, mode='rb', compresslevel=9, encoding=None, errors=None, newline=None)
    start_time = time.perf_counter()
    # print(len(b.tobytes()))
    gzip_com = gzip.compress(b.tobytes(), compresslevel=9, mtime=None)
    print(len(gzip_com))
    print(time.perf_counter() - start_time)
    # f = open('mcm_bits.dat', 'w')
    # f.write(bits)
    # f.close()
    print("----- prev 2 points -----")

    # print(K)
    # K = 8
    # K = 16
    # K = 32
    # K = 64
    # K = 16 * 16
    # K = 16 * 128
    K = 16 * 256
    # K = 2**6
    K = int(math.log2(K))
    # print(len(bits))
    # symbols = "".join([str(int(bits[K * i: K * (i + 1)], 2)) for i in range(0, int(len(bits) / K))])
    symbols = [str(int(bits[K * i: K * (i + 1)], 2)) for i in range(0, int(len(bits) / K))]
    # print([str(int(bits[K * i: K * (i + 1)], 2)) for i in range(0, int(len(bits) / K))])
    # print(len([str(int(bits[K * i: K * (i + 1)], 2)) for i in range(0, int(len(bits) / K))]))
    # symbols = symbols + str(int(bits[K * i: K * (i + 1)], 2))
    # print(symbols)
    # print(K * int(len(bits) / K))
    if K * int(len(bits) / K) < len(bits):
        symbols = symbols + [str(int(bits[K * int(len(bits) / K):], 2))]
    start_time = time.perf_counter()
    result = ShannonFennonCompression().compress(symbols)
    size = 0
    sym2bit = {}
    overhead_bit = 0
    # max_simbit = max([math.log2(len(i.code)) + 1 for i in result])
    max_simbit = max([len(i.code) for i in result])
    # print([i.code for i in result])
    overhead_bit = (K + max_simbit) * len(result)
    for i in result:
        # print(f"Character-- {i.original}:: Code-- {i.code} :: Probability-- {i.probability}")
        sym2bit[i.symbol] = i.code


    compressed_data = ""
    for sym in symbols:
        # print(sym)
        # print(sym2bit[sym])
        compressed_data = compressed_data + sym2bit[sym]

    print(time.perf_counter() - start_time)
    print(f"{len(compressed_data) / 8}, {overhead_bit / 8.0}, {max_simbit}")

    print("----- prev 3 points -----")
    symbols = [str(int(bits[K * i: K * (i + 1)], 2)) for i in range(0, int(len(bits) / K))]
    if K * int(len(bits) / K) < len(bits):
        symbols = symbols + [str(int(bits[K * int(len(bits) / K):], 2))]
    start_time = time.perf_counter()
    # print(symbols)
    result = HuffmanCompression().compress(symbols)
    # print(result)
    size = 0
    sym2bit = {}
    # max_simbit = max([len(i.code) for i in result])
    # max_simbit = max([math.log2(len(i.code)) + 1 for i in result])
    max_simbit = max([len(i.code) for i in result])
    overhead_bit = (K + max_simbit) * len(list(result[:]))

    for i in result:
        # print(f"Character-- {i.original}:: Code-- {i.code} :: Probability-- {i.probability}")
        sym2bit[i.symbol] = i.code

    compressed_data = ""
    # print(sym2bit)
    for sym in symbols:
        compressed_data = compressed_data + sym2bit[sym]

    print(time.perf_counter() - start_time)
    print(len(compressed_data) / 8)
    print(f"{len(compressed_data) / 8}, {overhead_bit / 8.0}")
    # print(compressed_data)

    print("----- prev 4 google-polyline -----")
    trajectory = [(x[i], y[i], z[i]) for i in range(0, len(x))]
    # trajectory = [(etsi_x[i], etsi_y[i], 0) for i in range(0, len(etsi_x))]
    print(trajectory)
    comp = PolylineCodec()
    start_time = time.perf_counter()
    sig = comp.encode(trajectory, 2)
    print(time.perf_counter() - start_time)
    print(sig)
    print(len(sig))
    deco = comp.decode(sig, 2)
    print(deco)

    print("----- proposed -----")
    dim_dict = {
        "x": [d["efficient_dim"] for d in res_x],
        "y": [d["efficient_dim"] for d in res_y]
    }
    x_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_x])
    y_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_y])
    z_size = sum([(2 + 2 + 4 * d["efficient_dim"]) for d in res_z])
    # z_size = 2 + 2 + 4
    size = 14 + x_size + y_size + z_size
    # print(t)
    print('{:}, {:}, {:}, {:}'.format(size, mcm.size(), take_time, file_name))
