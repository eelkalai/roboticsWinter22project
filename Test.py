import Visibility_Graph as vg
import PathFinder
import numpy as np
from math import atan, pi

# a = np.array([[0, 3, 1], [1, 6, 5], [9, 7, 3], [3, 4, 6]])
# print(a[0, 1])
# a = np.sort(a, 0)
# print(a)
# print(np.empty((0, 3), float))

# a = np.array([])
# print(a.size)

p1 = np.array([1, 2])
p2 = np.array([2, 3])
print(PathFinder.wallAngle(p1, p2))
print(np.rad2deg(PathFinder.wallAngle(p1, p2)))
print(atan(1))
print(np.rad2deg(atan(1)))

print(pi/18)
