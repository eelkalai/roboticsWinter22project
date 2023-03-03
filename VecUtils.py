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


def Distance2D(start, end):
    return np.linalg.norm(start[:2] - end[:2])


def Intersection(p1, p2, p3, p4):
    numerator = np.linalg.det(np.column_stack([(p1-p3)[:2], (p3-p4)[:2]]))
    denominator = np.linalg.det(np.column_stack([(p1-p2)[:2],(p3-p4)[:2]]))
    if denominator == 0 or numerator == 0:
        return np.array([])
    t = numerator/denominator
    if t < 0:
        return np.array([])
    return (1-t)*p1 + t*p2
