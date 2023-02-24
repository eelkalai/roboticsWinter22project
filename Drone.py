import numpy as np

import MatrixUtils
import VecUtils
from DroneClient import DroneClient
import time


class Drone(DroneClient):

    def __init__(self):
        super().__init__()
        self.__STEP_SIZE = 1
        self.__MAX_SPEED = 25
        self.__ACCELERATION = 0.5
        self.__LIDAR_FOV = 90
        self.__LIDAR_RANGE = 35

    def MoveStep(self, end_pos, speed):
        x, y, z = VecUtils.VecToAxes(end_pos)
        self.flyToPosition(x, y, z, speed)

    def FollowObstacle(self):
        pass

    def getPosNumpy(self):
        cur_pos = self.getPose().pos
        return VecUtils.AxesToVec(cur_pos.x_m, cur_pos.y_m, cur_pos.z_m)

    def getRotNumpy(self):
        cur_rot = self.getPose().orientation
        return VecUtils.AxesToVec(cur_rot.x_rad, cur_rot.y_rad, cur_rot.z_rad)

    def getLidarRelativeNumpy(self):
        theta = -self.getPose().orientation.z_rad
        r_z = MatrixUtils.RotationMatrix(theta, 2)
        lidar_data = self.getLidarData().points
        if len(lidar_data) > 3:
            lidar_data = lidar_data[:3]
        if len(lidar_data) == 3:
            lidar_data = np.array(lidar_data)
            lidar_data = r_z.dot(lidar_data)
        return lidar_data

    def getLidarWorldNumpy(self):
        lidar = self.getLidarRelativeNumpy()
        if lidar[0] == 0:
            return lidar
        return lidar + self.getPosNumpy()

    def getStepSize(self):
        return self.__STEP_SIZE

    def getMaxSpeed(self):
        return self.__MAX_SPEED

    def getAccelerationRate(self):
        return self.__ACCELERATION

    def getLidarFOV(self):
        return self.__LIDAR_FOV

    def getLidarRange(self):
        return self.__LIDAR_RANGE

    def performScan(self, interval=0.1, duration=1):
        lidar_points = np.empty((0, 3), float)
        for i in range(round(duration / interval)):
            cur_point = self.getLidarWorldNumpy()
            if cur_point[0] != 0:
                lidar_points = np.append(lidar_points, np.array([cur_point]), axis=0)
            time.sleep(interval)
        return lidar_points
