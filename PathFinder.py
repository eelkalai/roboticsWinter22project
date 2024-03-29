import numpy as np
import time

import VecUtils
from shapely.geometry import Polygon, Point
from math import sqrt, atan, pi


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
    scan = (scan[ind])
    if scan.size < 2:
        return np.empty((0, 3), float)
    scan = scan[(0, -1), :]
    scan = drone.getLidarWorldNumpy(scan)
    scan[0] = scan[1] - scan[0] + drone.getPosNumpy()
    scan[1] = scan[0]
    return scan[0]


def BypassObstacle(drone, start_pos, end_pos):
    finished = False
    poly = Polygon()
    while not finished:
        next_pos, finished = ObstacleNextPos(drone, start_pos, end_pos)
        print(drone.getPosNumpy(), next_pos)
        og_pos = drone.getPosNumpy()
        drone.MoveStep(next_pos, 2)
        time.sleep(2)
        if not finished:
            poly.union(Point(next_pos[0], next_pos[1]))
    return poly


def ObstacleNextPos(drone, start_pos, end_pos):
    tangent = FindTangent(drone)
    cur_pos = drone.getPosNumpy()
    if tangent.size == 0:
        return end_pos, True
    intersection = np.array([])  # .Intersection(start_pos, end_pos, cur_pos, tangent[0])
    if intersection.size != 0:
        return intersection, True
    tangent[-1] = start_pos[-1]
    return tangent, False


def Bug2(drone, x, y, relative=False):
    cur_pos = drone.getPosNumpy()
    end_pos = np.array([x,y,0],dtype= cur_pos.dtype)
    if relative:
        end_pos = end_pos + cur_pos
    end_pos[-1] = cur_pos[-1]
    next_point = end_pos
    polygons = []
    speed = 0
    drone.MoveStep(end_pos, 2)
    while True:
        cur_pos = drone.getPosNumpy()
        action = DecideOnAction(end_pos, speed, cur_pos, drone)
        if action == ActionPerStep.OBSTACLE:
            polygons.append(BypassObstacle(drone, cur_pos, end_pos))
        elif action == ActionPerStep.MOVE:
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


def distance_and_angleFromObs(drone, max_duration):
    o1 = drone.getLidarRelativeNumpy()
    if o1.size == 0:
        return None, None
    elif abs(o1[1]) < 1:
        return o1[0], 0
    else:
        duration = 0
        while duration < max_duration:
            time.sleep(0.1)
            o2 = drone.getLidarRelativeNumpy()
            if o2.size > 1:
                return distanceFromOrigin2Line(o1, o2), wallAngle(o1, o2)
            duration += 0.1
        return np.linalg.norm(o1[:2]), None


def distanceFromOrigin2Line(o1, o2):
    # y = mx + b
    m = (o2[1] - o1[1]) / (o2[0] - o1[0])
    b = o1[1] - m * o1[0]
    return abs(b) / sqrt(m ** 2 + 1)


def wallAngle(o1, o2):
    # alpha = atan((o2[1]-o1[1]) / (o2[0]-o1[0]))
    # if alpha > pi/18:
    #     alpha = alpha - pi
    # return alpha
    return atan((o2[1] - o1[1]) / (o2[0] - o1[0]))


def FolloWall(drone):
    speed = 1
    save_distance = 5
    interval = 0.1
    count = 0
    drone.goInLine(0, speed)
    ref_angle = 0
    while count < speed / (save_distance * interval):
        distance, angle = distance_and_angleFromObs(drone, interval)
        print("distance: ", distance)
        if distance is not None:
            if angle is not None:
                ref_angle = angle
                print("ref_angle: ", ref_angle)
            alpha = (distance - save_distance) / (save_distance + distance)
            print("alpha: ", alpha)
            drone.goInLine(ref_angle + alpha, speed)
            count = -interval
        time.sleep(interval)
        count += interval
    drone.stop()
