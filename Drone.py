import VecUtils
from DroneClient import DroneClient
import time


class Drone(DroneClient):
    STEP_SIZE = 1
    MAX_SPEED = 5
    ACCELERATION = 0.5

    LIDAR_FOV = 90
    LIDAR_RANGE = 35

    def MoveSlightly(self, end_pos, speed):
        x, y, z = VecUtils.VecToAxes(end_pos)
        self.flyToPosition(x, y, z, speed)
        time.sleep(self.STEP_SIZE * speed)

    def getPosNumpy(self):
        cur_pos = self.getPose().pos
        return VecUtils.AxesToVec(cur_pos.x_m, cur_pos.y_m, cur_pos.z_m)
