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
        if len(lidar_data) == 3:
            lidar_data = np.array(lidar_data)
            lidar_data = r_z.dot(lidar_data)
        return lidar_data

    def getLidarWorldNumpy(self):
        return self.getLidarRelativeNumpy() + self.getPosNumpy()

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
