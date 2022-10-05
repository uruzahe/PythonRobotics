import numpy as np
import math
import sys
from .util import *

# import pyjion
# pyjion.enable()

class Path:
    def __init__(self, path = []):
        self.data = {float(d[0]): d[1:4] for d in path}

    def state(self, target_t):
        data = sorted(self.data.items(), key=lambda x:x[0])

        tl = [float(d[0]) for d in data]
        pl = [np.array(d[1]) for d in data]

        vl = [(pl[i+1] - pl[i]) / (tl[i + 1] - tl[i]) for i in range(0, len(pl) - 1)]
        vl.append(vl[-1])

        al = [(vl[i+1] - vl[i]) / (tl[i + 1] - tl[i]) for i in range(0, len(vl) - 1)]
        al.append(al[-1])
        al.append(al[-1])

        index = len([t for t in tl if t < target_t])

        if len(pl) <= index:
            index = len(pl) - 1

        elif index - 1 < 0:
            index = 1

        p = self.vec(tl[index], pl[index], tl[index - 1], pl[index - 1], target_t)
        v = self.vec(tl[index], vl[index], tl[index - 1], vl[index - 1], target_t)
        a = self.vec(tl[index], al[index], tl[index - 1], al[index - 1], target_t)
        vec = pl[index] - pl[index - 1]
        angle = 0
        if vec[0] == 0:
            angle = 0
        else:
            angle = math.atan(vec[1]/vec[0])
        progress = index / (len(pl) - 1)
        # # print(f"vec: {vec}")

        return angle, p, v, a, progress


    def position(self, t):
        _, p, _, _, _ = self.state(t)

        return p

    def speed(self, t):
        _, _, v, _, _ = self.state(t)

        return v

    def accel(self, t):
        _, _, _, a, _ = self.state(t)

        return a

    def vec(self, t1, p1, t2, p2, t):
        w = (t - t1) / (t2 - t1)

        return (1 - w) * np.array(p1) + w * np.array(p2)

    def max_time(self):
        return max([float(t) for t in self.data.keys()])

    def all_paths(self):
        return self.data

    def times(self):
        data = sorted(self.data.items(), key=lambda x:x[0])

        return [float(d[0]) for d in data]

class Obstacle:
    def __init__(self, t, position=[0, 0, 0], speed=[0, 0, 0], accel=[0, 0, 0]):
        self.t = t
        self.position = position
        self.speed = speed
        self.accel = accel

    def pos(self, t=None):
        return self.positions(t)[0]

    def positions(self, t=None):
        if t is None:
            return [self.position]

        else:
            dt = t - self.t
            new_position = list(np.array(self.position) + np.array(self.speed) * dt + np.array(self.accel) * dt * dt)

            return [new_position]

class CarObstacle(Obstacle):
    def __init__(self, t, width, length, heading=0, id=None, position=[0, 0, 0], speed=[0, 0, 0], accel=[0, 0, 0], path=None):
        super().__init__(t, position, speed, accel)

        self.id = id
        self.heading = heading
        self.width = width
        self.length = length
        self.path = None
        # assert(np.isnan(self.heading) is False)
        if path is not None:
            self.path = path
            # # # print(path)
            angle, p, v, a, progress = self.path.state(t)

            self.position = p
            self.speed = v
            self.accel = a
            self.heading = angle

    def pos(self, t=None):
        position = self.position

        if t is not None:
            if self.path is not None:
                position = self.path.position(t)

            else:
                position = super().positions(t)[0]

        return position

    def positions(self, t=None):
        return self.position_to_shapes(self.pos(t), self.heading, self.width, self.length)

    def check_collision(self, obstacle_obj, t):
        # for ob_c in obstacle_obj.corners(obstacle_obj.pos(t), obstacle_obj.heading, obstacle_obj.width, obstacle_obj.length):
        #     if any([ math.dist(ob_c, self_c) <= 0.5 for self_c in self.corners(self.pos(t), self.heading, self.width, self.length) ]):
        #         return True
        #
        # return False

        self_p = self.pos(t)

        if self.length < math.dist(self_p, obstacle_obj.pos(t)):
            return False

        a, b, c, d = self.corners(self.pos(t), self.heading, self.width, self.length, 0, ACC_l(math.dist(self.speed, [0, 0, 0]), self.length))
        print(f"{sys._getframe().f_code.co_name}, t: {t}, id: {self.id}, ob_id: {obstacle_obj.id}, pos: {self_p}, ob_pos: {obstacle_obj.pos(t)}, a: {a}, b: {b}, c: {c}, d: {d}")
        print(obstacle_obj.positions(t))
        for p in obstacle_obj.positions(t):
            if is_collide(a, b, c, d, p):
                print(f"collision point: {p}")
                # ----- collide -----
                return True

            else:
                # ----- no collide -----
                continue

        return False

    def corners(self, position, heading, width, length, width_mergin=0, length_mergin=0):
        # length_mergin = 0
        # # # print(heading)
        # # # print(width)

        # length = length
        # print(length)
        # print(length_mergin)
        rad = math.radians(heading)
        # # # print(heading)
        R = np.array([
            [np.cos(rad),   -np.sin(rad),   0],
            [np.sin(rad),   np.cos(rad),    0],
            [0,             0,              1]
            ])

        a = np.array(position) + np.dot(R, np.array([length_mergin + 0,         width/2.0,  position[2]]))
        b = np.array(position) + np.dot(R, np.array([-length - length_mergin,    width/2.0,  position[2]]))
        c = np.array(position) + np.dot(R, np.array([-length - length_mergin,    -width/2.0, position[2]]))
        d = np.array(position) + np.dot(R, np.array([length_mergin + 0,         -width/2.0, position[2]]))

        # # print(f"id: {self.id}, R: {R}, width: {width}, length: {length}, pos: {position}, a: {a}, b: {b}, c: {c}, d: {d}, rad: {rad}, heading: {heading}")
        # assert(np.isnan(rad) is False)
        return a, b, c, d

    def position_to_shapes(self, position, heading, width, length):
        a, b, c, d = self.corners(position, heading, width, length)

        # ab = [list(w * a + (1 - w) * b) for w in np.linspace(0, 1, int(np.linalg.norm(a - b)) + 1)]
        # bc = [list(w * b + (1 - w) * c) for w in np.linspace(0, 1, int(np.linalg.norm(b - c)) + 1)]
        # cd = [list(w * c + (1 - w) * d) for w in np.linspace(0, 1, int(np.linalg.norm(c - d)) + 1)]
        # da = [list(w * d + (1 - w) * a) for w in np.linspace(0, 1, int(np.linalg.norm(d - a)) + 1)]
        #
        # return ab + bc + cd + da

        return a, b, c, d

class ObstacleHandler:
    def __init__(self):
        self.__obstacles = []
        self.__car_obstacles = {}

    def add_obstacle(self, new_ob):
        self.__obstacles.append(new_ob)

    def possible_same_ves(self, new_ob):
        return [v for v in self.__car_obstacles.values() if v.check_collision(new_ob, new_ob.t) is True]

    def erase_carob_by_id(self, id):
        self.__car_obstacles.pop(id)

    def new_carob_id(self):
        return max([int(v.id) for v in self.__car_obstacles.values()] + 1)


    def add_car_obstacle(self, new_ob):
        self.__car_obstacles[new_ob.id] = new_ob

    def clear_obstacle(self, target_time=None):
        if target_time is None:
            self.__obstacles.clear()
            self.__car_obstacles.clear()

        else:
            self.__obstacles = [ob for ob in self.__obstacles if target_time <= ob.t]
            self.__car_obstacles = {k: v for k, v in self.__car_obstacles.items() if target_time <= v.t}


    def positions(self, t=None):
        result = []

        for ob in self.all_obstacles():
            # # # print(result)
            result = result + list(ob.positions(t))

        return result

    def leading_carob(self, position,):
        pass



    def all_obstacles(self):
        return self.__obstacles + [v for v in self.__car_obstacles.values()]

    def car_obstacles(self):
        return [v for v in self.__car_obstacles.values()]
