"""
Visibility Graph for 100m high
"""

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from shapely.geometry import Polygon
from shapely.geometry import Point

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

# plot the polygons
for poly in polygons.values():
    x, y = poly.exterior.xy
    plt.plot(x, y)

# show the plot
plt.gca().invert_xaxis()
plt.grid()
plt.show()