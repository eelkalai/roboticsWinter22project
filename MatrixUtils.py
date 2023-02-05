import numpy as np


def RotationMatrix(ang, axis, homogeneous=False):
    r = np.array([[np.cos(ang), np.sin(ang)],
                  [-np.sin(ang), np.cos(ang)]])
    r = np.insert(r, axis, np.zeros(2), axis=0)
    r = np.insert(r, axis, np.zeros(3), axis=1)
    r[axis, axis] = 1
    if axis == 1:
        r = r.transpose()
    if homogeneous:
        r = np.insert(r, 3, np.zeros(3), axis=0)
        r = np.insert(r, 3, np.zeros(4), axis=1)
        r[-1, -1] = 1
    return r