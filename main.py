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
    client.getLidarData()
    client.getLidarWorldNumpy()
    # time.sleep(3)
    # client.flyToPosition(-346, -420, -100, 10)
    # client.MoveInLine(50, 50)
    while True:
        action = input("Action:[Help(1)]")
        try:
            action = int(action)
        except ValueError:
            continue
        if action == 0:
            quit()
        elif action == 1:
            print("Actions:")
            print("0-quit")
            print("1-Help")
            print("2-MoveInLine")
            print("3-SetPosition")
        elif action == 2:
            x = int(input("x:"))
            y = int(input("y:"))
            PathFinder.MoveInLine(client, x, y)
        elif action == 3:
            x = int(input("x:"))
            y = int(input("y:"))
            z = int(input("z:"))
            client.setAtPosition(x, y, z)
        elif action == 4:
            print("lidar_world: ",client.getLidarWorldNumpy())
            print("lidar_reletive: ",client.getLidarRelativeNumpy())
            print("lider: ",client.getLidarData().points)
        else:
            print("position: ", client.getPosNumpy())
        print(client.getLidarData())
        time.sleep(1)

        
