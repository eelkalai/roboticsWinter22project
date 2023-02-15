import numpy as np
import time

import VecUtils


class ActionPerStep:
    MOVE = 1
    END = 2


def DecideOnAction():
    return ActionPerStep.MOVE


def MoveToPoint(drone, x, y):
    cur_pos = drone.getPosNumpy()
    end_pos = cur_pos + np.array((x, y, 0))
    step_destination = end_pos
    distance = VecUtils.Distance2D(cur_pos, end_pos)
    speed = 0
    stop_distance = 0
    print("speed=", speed, ", delta=", distance, ", stopDistance=", stop_distance)
    while True:
        action = DecideOnAction()
        if action == ActionPerStep.MOVE:
            drone.MoveStep(end_pos, speed)
            time.sleep(drone.getStepSize())
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
    while distance > drone.getAccelerationRate() * drone.getStepSize():
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
              ", delta=", distance, ", stopDistance=", stop_distance)
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
