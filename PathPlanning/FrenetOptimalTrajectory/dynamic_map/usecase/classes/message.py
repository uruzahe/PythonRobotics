from .obstacles import *
 
class CAM:
    def __init__(self, id, time, position, speed, accel, angle, width, length):
        self.id = id
        self.time = time
        self.position = position
        self.speed = speed
        self.accel = accel
        self.angle = angle
        self.width = width
        self.length = length

class MCM:
    def __init__(self, time, ppath, fpath):
        self.time = time
        self.ppath = ppath
        self.fpath = fpath

class CPM:
    def __init__(self, time):
        self.time = time
        self.positions = {}
    
    def add_obstacles(self, ob, t):
        self.positions[ob.id] = ob.positions(t)

    

    
    
        


    #def __init__(self, t, width, length, heading, id, position, speed, accel, path):
        #ob = CarObstacle(t, width, length, heading, id, position, speed, accel, path)
        #self.position = {}
        #
    #def cpm_dict(self, width, length, heading, id, position):
        #self.position[id] = ob.positions