import numpy as np

import MatrixUtils
import VecUtils
from DroneClient import DroneClient
import time


class Drone(DroneClient):
    STEP_SIZE = 1
    MAX_SPEED = 25
    ACCELERATION = 0.5
    LIDAR_FOV = 90
    LIDAR_RANGE = 35

    def MoveSlightly(self, end_pos, speed):
        x, y, z = VecUtils.VecToAxes(end_pos)
        self.flyToPosition(x, y, z, speed)
        time.sleep(self.STEP_SIZE)

    def getPosNumpy(self):
        cur_pos = self.getPose().pos
        return VecUtils.AxesToVec(cur_pos.x_m, cur_pos.y_m, cur_pos.z_m)

    def getLidarNumpyRelative(self):
        theta = self.getPose().orientation.z_rad
        r_z = MatrixUtils.RotationMatrix(theta,2)
        lidar_data = self.getLidarData().points
        if len(lidar_data) == 3:
            lidar_data = np.array(lidar_data)
            lidar_data = r_z.dot(lidar_data)
        return lidar_data

    def getLidarNumpyWorld(self):
        return self.getLidarNumpyRelative() + self.getPosNumpy()