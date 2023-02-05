import numpy as np
import time


def MoveInLine(drone, x, y):
    cur_pos = drone.getPosNumpy()
    end_pos = cur_pos + np.array((x, y, 0))
    speed = 0
    while np.linalg.norm(end_pos - cur_pos) > speed ** 2 / 2 / drone.ACCELERATION:
        if speed < drone.MAX_SPEED:
            speed = speed + drone.ACCELERATION
        drone.MoveSlightly(end_pos, speed)
        cur_pos = drone.getPosNumpy()
    while speed > 0:
        speed = speed - drone.ACCELERATION
        drone.MoveSlightly(end_pos, speed)
        time.sleep\
            (drone.STEP_SIZE * speed)


def SolvePolyPath(x_0, x_1, v_0, v_1, t_0, t_1):
    # x(t)=a+bt+ct^2+dt^3
    # x'(t)=b+2ct+3dt^2
    A = np.array([[1, t_0, t_0 ** 2, t_0 ** 3], [1, t_1, t_1 ** 2, t_1 ** 3], [0, 1, 2 * t_0, 3 * t_0**2],
                  [0, 1, 2 * t_1, 3 * t_1**2]])
    B = np.array([x_0, x_1, v_0, v_1])
    print(np.linalg.solve(A, B))