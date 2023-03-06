import itertools
import six
import math

class D1PolylineCodec(object):
    def _pcitr(self, iterable):
        return six.moves.zip(iterable, itertools.islice(iterable, 1, None))

    def _py2_round(self, x):
        # The polyline algorithm uses Python 2's way of rounding
        return int(math.copysign(math.floor(math.fabs(x) + 0.5), x))

    def _write(self, output, curr_value, prev_value, factor):
        curr_value = self._py2_round(curr_value * factor)
        prev_value = self._py2_round(prev_value * factor)
        coord = curr_value - prev_value
        coord <<= 1
        coord = coord if coord >= 0 else ~coord

        while coord >= 0x20:
            output.write(six.unichr((0x20 | (coord & 0x1f)) + 63))
            coord >>= 5

        output.write(six.unichr(coord + 63))

    def _trans(self, value, index):
        byte, result, shift = None, 0, 0

        while byte is None or byte >= 0x20:
            byte = ord(value[index]) - 63
            index += 1
            result |= (byte & 0x1f) << shift
            shift += 5
            comp = result & 1

        return ~(result >> 1) if comp else (result >> 1), index

    def decode(self, expression, precision=5, geojson=False):
        coordinates, index, lat, lng, length, factor = [], 0, 0, 0, len(expression), float(10 ** precision)
        higt = 0
        # time = 0

        while index < length:
            # time_change, index = self._trans(expression, index)
            lat_change, index = self._trans(expression, index)
            # lng_change, index = self._trans(expression, index)
            # higt_change, index = self._trans(expression, index)
            # time += time_change
            lat += lat_change
            # lng += lng_change
            # higt += higt_change
            coordinates.append((lat / factor,))

        if geojson is True:
            coordinates = [t[::-1] for t in coordinates]

        return coordinates

    def encode(self, coordinates, precision=5, geojson=False):
        if geojson is True:
            coordinates = [t[::-1] for t in coordinates]

        output, factor = six.StringIO(), int(10 ** precision)

        self._write(output, coordinates[0][0], 0, factor)
        # self._write(output, coordinates[0][1], 0, factor)
        # self._write(output, coordinates[0][2], 0, factor)
        # self._write(output, coordinates[0][3], 0, factor)

        for prev, curr in self._pcitr(coordinates):
            self._write(output, curr[0], prev[0], factor)
            # self._write(output, curr[1], prev[1], factor)
            # self._write(output, curr[2], prev[2], factor)
            # self._write(output, curr[3], prev[3], factor)

        return output.getvalue()

class D3PolylineCodec(object):
    def _pcitr(self, iterable):
        return six.moves.zip(iterable, itertools.islice(iterable, 1, None))

    def _py2_round(self, x):
        # The polyline algorithm uses Python 2's way of rounding
        return int(math.copysign(math.floor(math.fabs(x) + 0.5), x))

    def _write(self, output, curr_value, prev_value, factor):
        curr_value = self._py2_round(curr_value * factor)
        prev_value = self._py2_round(prev_value * factor)
        coord = curr_value - prev_value
        coord <<= 1
        coord = coord if coord >= 0 else ~coord

        while coord >= 0x20:
            output.write(six.unichr((0x20 | (coord & 0x1f)) + 63))
            coord >>= 5

        output.write(six.unichr(coord + 63))

    def _trans(self, value, index):
        byte, result, shift = None, 0, 0

        while byte is None or byte >= 0x20:
            byte = ord(value[index]) - 63
            index += 1
            result |= (byte & 0x1f) << shift
            shift += 5
            comp = result & 1

        return ~(result >> 1) if comp else (result >> 1), index

    def decode(self, expression, precision=5, geojson=False):
        coordinates, index, lat, lng, length, factor = [], 0, 0, 0, len(expression), float(10 ** precision)
        higt = 0
        # time = 0

        while index < length:
            # time_change, index = self._trans(expression, index)
            lat_change, index = self._trans(expression, index)
            lng_change, index = self._trans(expression, index)
            higt_change, index = self._trans(expression, index)
            # time += time_change
            lat += lat_change
            lng += lng_change
            higt += higt_change
            coordinates.append((lat / factor, lng / factor, higt / factor))

        if geojson is True:
            coordinates = [t[::-1] for t in coordinates]

        return coordinates

    def encode(self, coordinates, precision=5, geojson=False):
        if geojson is True:
            coordinates = [t[::-1] for t in coordinates]

        output, factor = six.StringIO(), int(10 ** precision)

        self._write(output, coordinates[0][0], 0, factor)
        self._write(output, coordinates[0][1], 0, factor)
        self._write(output, coordinates[0][2], 0, factor)
        # self._write(output, coordinates[0][3], 0, factor)

        for prev, curr in self._pcitr(coordinates):
            self._write(output, curr[0], prev[0], factor)
            self._write(output, curr[1], prev[1], factor)
            self._write(output, curr[2], prev[2], factor)
            # self._write(output, curr[3], prev[3], factor)

        return output.getvalue()

class PolylineCodec(D3PolylineCodec):
    pass

class D4PolylineCodec(object):
    def _pcitr(self, iterable):
        return six.moves.zip(iterable, itertools.islice(iterable, 1, None))

    def _py2_round(self, x):
        # The polyline algorithm uses Python 2's way of rounding
        return int(math.copysign(math.floor(math.fabs(x) + 0.5), x))

    def _write(self, output, curr_value, prev_value, factor):
        curr_value = self._py2_round(curr_value * factor)
        prev_value = self._py2_round(prev_value * factor)
        coord = curr_value - prev_value
        coord <<= 1
        coord = coord if coord >= 0 else ~coord

        while coord >= 0x20:
            output.write(six.unichr((0x20 | (coord & 0x1f)) + 63))
            coord >>= 5

        output.write(six.unichr(coord + 63))

    def _trans(self, value, index):
        byte, result, shift = None, 0, 0

        while byte is None or byte >= 0x20:
            byte = ord(value[index]) - 63
            index += 1
            result |= (byte & 0x1f) << shift
            shift += 5
            comp = result & 1

        return ~(result >> 1) if comp else (result >> 1), index

    def decode(self, expression, precision=5, geojson=False):
        coordinates, index, lat, lng, length, factor = [], 0, 0, 0, len(expression), float(10 ** precision)
        higt = 0
        time = 0

        while index < length:
            time_change, index = self._trans(expression, index)
            lat_change, index = self._trans(expression, index)
            lng_change, index = self._trans(expression, index)
            higt_change, index = self._trans(expression, index)
            time += time_change
            lat += lat_change
            lng += lng_change
            higt += higt_change
            coordinates.append((time/ factor, lat / factor, lng / factor, higt / factor))

        if geojson is True:
            coordinates = [t[::-1] for t in coordinates]

        return coordinates

    def encode(self, coordinates, precision=5, geojson=False):
        if geojson is True:
            coordinates = [t[::-1] for t in coordinates]

        output, factor = six.StringIO(), int(10 ** precision)

        self._write(output, coordinates[0][0], 0, factor)
        self._write(output, coordinates[0][1], 0, factor)
        self._write(output, coordinates[0][2], 0, factor)
        self._write(output, coordinates[0][3], 0, factor)

        for prev, curr in self._pcitr(coordinates):
            self._write(output, curr[0], prev[0], factor)
            self._write(output, curr[1], prev[1], factor)
            self._write(output, curr[2], prev[2], factor)
            self._write(output, curr[3], prev[3], factor)

        return output.getvalue()

if __name__ == "__main__":
    print("----- D4 -----")
    comp = D4PolylineCodec()

    # trajectory = [(38.5555555555555, -120.2, 0.0), (40.777777777777, -120.9, 0.0), (43.22222222222, -126.4, 0.0)]
    trajectory = [(0.0, 38.5555555555555, -120.2, 0.0), (1.0, 40.777777777777, -120.9, 1.0), (2.0, 43.22222222222, -126.4, 0.0)]
    trajectory = trajectory * 10
    print(trajectory)
    sig = comp.encode(trajectory[0:1], 0)
    print(sig)
    print(len(sig))
    deco = comp.decode(sig, 0)
    print(deco)


    print("----- D1 -----")
    comp = D1PolylineCodec()
    data1 = [(2, ), (10,), (1.55,), (2.55,), (4.55,)]
    data2 = [(3, ), (10,), (1.55,), (2.55,), (4.55,)]
    data3 = [(4, ), (10,), (1.55,), (2.55,), (4.55,)]
    sig = comp.encode(data1 * 10, 2)
    print(sig)
    print(len(sig))
    deco = comp.decode(sig, 2)
    print(deco)

    print("----- D3 -----")
    data4 = [(data1[i][0], data2[i][0], data3[i][0]) for i in range(0, len(data1))]
    comp = D3PolylineCodec()
    sig = comp.encode(data4 * 10, 2)
    print(sig)
    print(len(sig))
    deco = comp.decode(sig, 2)
    print(deco)