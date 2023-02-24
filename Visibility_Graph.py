"""
Visibility Graph for 100m high
"""

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from shapely.geometry import Polygon
from shapely.geometry import Point


def oped_obstacle_csv():
    # Create a dictionary to store the obstacles polygons
    polygons = {}
    # open the obstacle csv
    obstacles_100m = pd.read_csv('obstacles_100m_above_sea_level.csv')
    # run in loop for the column to creat polygon using geom
    for index, row in obstacles_100m.iterrows():
        x = row[1]
        y = row[2]
        obs_type = row[4]
        if polygons.get(obs_type) is None:
            polygons[obs_type] = Polygon()
        polygons[obs_type] = polygons[obs_type].union(Point(x, y))
    # close the loop of every polygon
    for key in polygons.keys():
        polygons[key] = polygons[key].convex_hull
    return polygons


def plot_all_polygon():
    polygons = oped_obstacle_csv()
    # plot the polygons
    for poly in polygons.values():
        x, y = poly.exterior.xy
        plt.plot(x, y)
        # show the plot
    plt.grid()
    plt.show()


def find_polygon(polygon_number):
    polygons = oped_obstacle_csv()
    return polygons[polygon_number]


def plot_polygon(polygon_number):
    polygon = find_polygon(polygon_number)
    x, y = polygon.exterior.xy
    plt.plot(x, y)


def plot_lidar_data(polygon_number, x_obstacle, y_obstacle):
    plot_polygon(polygon_number)
    plt.scatter(x_obstacle, y_obstacle, s=2, c='red')
    plt.show()
