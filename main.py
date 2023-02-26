import PathFinder
from Drone import Drone
from DroneClient import DroneClient
import time
import airsim.utils
import Visibility_Graph as vg

if __name__ == "__main__":
    client = Drone()
    client.connect()
    print(client.isConnected())
    time.sleep(4)
    # client.setAtPosition(-346, -700, -100) #start position
    client.setAtPosition(-235, -600, -100)  # start before obstacle
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
            print("4-GetLidar")
            print("5-GetPosition")
            print("6-GetRotation")
            print("7-plot lidar data x10")
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
        elif action == 5:
            print("position: ", client.getPosNumpy())
        elif action == 6:
            print('Rotation: ', client.getRotNumpy())
        elif action == 7:
            obs = int(input("obstacle:"))
            interval = float(input("time interval: "))
            duration = float(input("time duration: "))
            lidar_point = client.performScanWorld(interval, duration)
            print("position: ", client.getPosNumpy())
            print('Rotation: ', client.getRotNumpy())
            print("lidar point in world frame:", lidar_point)
            vg.plot_lidar_data(obs, lidar_point[:, 0], lidar_point[:, 1])
        time.sleep(1)
