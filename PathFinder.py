import numpy as np
import time

import VecUtils
from shapely.geometry import Polygon, Point


class ActionPerStep:
    MOVE = 1
    END = 2
    SCAN = 3
    OBSTACLE = 4


def DecideOnAction(next_point, speed, cur_pos, drone):
    distance = VecUtils.Distance2D(cur_pos, next_point)
    scan = drone.performScanRelative()
    if scan.size > 0:
        ind = np.argsort(np.absolute(scan[:, 1]))
        scan = scan[ind]
        if scan[0, 1] < 3:
            return ActionPerStep.OBSTACLE
    if distance > drone.getStepSize():
        return ActionPerStep.MOVE
    return ActionPerStep.END


def FindTangent(drone):
    scan = drone.performScanRelative()
    if scan.size == 0:
        return scan
    ind = np.argsort(scan[:, 1])
    scan = (scan[ind])[(0, -1), :]
    scan[0][0] = scan[0][0] + drone.getSize()
    scan[0][1] = scan[0][1] - drone.getSize()
    scan[1][0] = scan[0][0] + drone.getSize()
    scan[1][1] = scan[1][1] + drone.getSize()
    scan = drone.getLidarWorldNumpy(scan)
    return scan


def BypassObstacle(drone, start_pos, end_pos):
    finished = False
    poly = Polygon()
    while not finished:
        next_pos, finished = ObstacleNextPos(drone, start_pos, end_pos)
        print(drone.getPosNumpy(),next_pos)
        drone.MoveStep(next_pos, 2)
        if not finished:
            poly.union(Point(next_pos[0], next_pos[1]))
    return poly


def ObstacleNextPos(drone, start_pos, end_pos):
    tangent = FindTangent(drone)
    cur_pos = drone.getPosNumpy()
    if tangent.size == 0:
        return cur_pos, True
    intersection = VecUtils.Intersection(start_pos, end_pos, cur_pos, tangent[0])
    if intersection.size != 0:
        return intersection, True
    return tangent[0], False


def Bug2(drone, x, y):
    cur_pos = drone.getPosNumpy()
    end_pos = cur_pos + np.array((x, y, 0))
    next_point = end_pos
    polygons = []
    speed = 0
    drone.MoveStep(end_pos, 2)
    while True:
        cur_pos = drone.getPosNumpy()
        action = DecideOnAction(next_point, speed, cur_pos,drone)
        if action == ActionPerStep.OBSTACLE:
            polygons.append(BypassObstacle(drone, cur_pos, end_pos))
        elif action == ActionPerStep.MOVE:
            print(drone.getPosNumpy(), end_pos)
            drone.MoveStep(end_pos, 2)
        elif action == ActionPerStep.END:
            return


def MoveInLine(drone, x, y):
    cur_pos = drone.getPosNumpy()
    end_pos = cur_pos + np.array((x, y, 0))
    distance = VecUtils.Distance2D(cur_pos, end_pos)
    speed = 0
    stop_distance = 0
    print("speed=", speed,
          ", delta=", distance, ", stopDistance=", stop_distance)
    while distance > 2:
        cur_pos = drone.getPosNumpy()
        distance = VecUtils.Distance2D(cur_pos, end_pos)
        stop_distance = StopDistance(drone, speed)
        if min(drone.getLidarRange(), distance) > stop_distance and speed < drone.getMaxSpeed():
            speed = speed + drone.getAccelerationRate()
        stop_distance = StopDistance(drone, speed)
        if min(drone.getLidarRange(), distance) <= stop_distance:
            speed = speed - drone.getAccelerationRate()
        stop_distance = StopDistance(drone, speed)
        print("speed=", speed,
              ", delta=", distance, ", cur_pos=", cur_pos, ", end pos=", end_pos)
        drone.MoveStep(end_pos, speed)


def StopDistance(drone, speed):
    if speed == drone.getAccelerationRate():
        return drone.getAccelerationRate() * drone.getStepSize()
    return (speed ** 2) / (2 * drone.getAccelerationRate())


def SolvePolyPath(x_0, x_1, v_0, v_1, t_0, t_1):
    # x(t)=a+bt+ct^2+dt^3
    # x'(t)=b+2ct+3dt^2
    A = np.array([[1, t_0, t_0 ** 2, t_0 ** 3], [1, t_1, t_1 ** 2, t_1 ** 3], [0, 1, 2 * t_0, 3 * t_0 ** 2],
                  [0, 1, 2 * t_1, 3 * t_1 ** 2]])
    B = np.array([x_0, x_1, v_0, v_1])
    print(np.linalg.solve(A, B))
