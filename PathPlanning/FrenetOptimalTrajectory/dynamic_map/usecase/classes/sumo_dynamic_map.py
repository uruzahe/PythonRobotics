from xml.etree import ElementTree

from .util import *


class DynamicMap:
    def __init__(self, net_file_path):
        self.data = self.__import_netfile(net_file_path)
        self.edge2edge = self.__edge2edge()
        self.edge2lanes = self.__edge2lanes()

    def max_speed(self, edge_id, lane_id):
        lane_index = lane_id.split("_")[1]

        if lane_index not in self.edge2lanes[edge_id]:
            lane_index = '0'

        return float(self.edge2lanes[edge_id][lane_index]["speed"])

    def road_width(self, edge_id, current_position):
        lanes = self.edge2lanes[edge_id]
        first_lane_shape = self.__parse_shape(lanes['0']['shape'])
        last_lane_shape = self.__parse_shape(lanes[str(len(lanes) - 1)]['shape'])

        # # print(first_lane_shape)
        # # print(current_position)
        a1, b1 = liner_coefficients_by_2_points(first_lane_shape[0], first_lane_shape[1])
        a2, b2 = liner_coefficients_by_2_points(last_lane_shape[0], last_lane_shape[1])

        return [distance_from_point(a1, b1, current_position), -distance_from_point(a2, b2, current_position)]


    def waypoints_by_edges(self, edges):
        if 0 < len(edges) <= 1:
            from_edge = edges[0]

            # # print(from_edge)
            # # print(self.edge2lanes[from_edge])
            for lane_index, lane in self.edge2lanes[from_edge].items():
                return [(float(d.split(",")[0]), float(d.split(",")[1])) for d in lane['shape'].split(" ")]

                break

        elif 2 <= len(edges):
            from_edge = edges[0]
            to_edge = edges[1]

            from_lane_index = self.edge2edge[from_edge][to_edge]["fromLane"]
            to_lane_index = self.edge2edge[from_edge][to_edge]["toLane"]

            from_lane_shape = self.edge2lanes[from_edge][from_lane_index]['shape'].split(" ")
            to_lane_shape = self.edge2lanes[to_edge][to_lane_index]['shape'].split(" ")

            if len(edges) == 2:
                return [(float(d.split(",")[0]), float(d.split(",")[1])) for d in from_lane_shape] + [(d.split(",")[0], d.split(",")[1]) for d in to_lane_shape]

            else:
                return [(float(d.split(",")[0]), float(d.split(",")[1])) for d in from_lane_shape] + waypoints_by_edges(edge[1:])

        else:
            return []

    def __edge2lanes(self):
        result = {}

        for e in self.data["edge"]:
            result[e["id"]] = {}

            for lane in e["lane"]:
                result[e["id"]][lane["index"]] = lane

        return result

    def __edge2edge(self):
        result = {}

        for conn in self.data["connection"]:
            if conn["from"] not in result:
                result[conn["from"]] = {}

            result[conn["from"]][conn["to"]] = conn

        return result

    def __import_netfile(self, net_file_path):
        tree = ElementTree.parse(net_file_path)
        elem = tree.getroot()

        return parse_xmlobj_of_element_tree(elem)

    def __parse_shape(self, shape_str):
        result = []

        for xy in shape_str.split(" "):
            result.append([float(d) for d in xy.split(",")])

        return result
