import numpy as np


def Normalize(vec):
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm


def AxesToVec(x, y, z):
    return np.array([x, y, z])


def VecToAxes(vec):
    return vec[0], vec[1], vec[2]
