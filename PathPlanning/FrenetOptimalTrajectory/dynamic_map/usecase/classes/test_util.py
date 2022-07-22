from util import *

a = [1, 1]
b = [-1 , 1]
c = [-1, -1]
d = [1, -1]

p = [0, 0]
assert(is_collide(a, b, c, d, p) == True)

p = [0.5, 0.8]
assert(is_collide(a, b, c, d, p) == True)

p = [3, 2]
assert(is_collide(a, b, c, d, p) == False)

p = [3, 3]
assert(is_collide(a, b, c, d, p) == False)

p = [1, 1.0001]
assert(is_collide(a, b, c, d, p) == False)
