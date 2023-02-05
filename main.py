import PathFinder
from Drone import Drone
from DroneClient import DroneClient
import time
import airsim.utils

if __name__ == "__main__":
    client = Drone()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -100)

    # time.sleep(3)
    # client.flyToPosition(-346, -420, -100, 10)
    # client.MoveInLine(50, 50)
    while True:
        x = int(input())
        y = int(input())
        PathFinder.MoveInLine(client, x, y)
        print(client.getLidarData())
        time.sleep(1)

        
