import json
import math
import os
from time import time

import cv2
#import matplotlib.pyplot as plt
import numpy as np



# Grid dimensions
gridWidth = float(10) # [m]
gridHeight = float(10) # [m]

# Map dimensions
mapWidth = float(5000) # 5m [m]
mapHeight = float(5000) # 5m [m]

# Robot start position with respect to the map
robotXOffset = mapWidth / 2
robotYOffset = mapHeight / 2

# Defining Sensor Characteristics
assert gridWidth == gridHeight
alpha = float(gridWidth)
beta = float(0.03)

# Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
l0 = float(0)
locc = 0.4
lfree = -0.4

# Defining an l vector to store the log odds values of each cell
#l_array = np.zeros([int(mapHeight/gridHeight), int(mapWidth/gridWidth)], dtype=np.float32)
l_array = np.zeros([500, 500], dtype=np.float32)
l_array[...] = l0


def inverseSensorModel(X_robot, theta, Xi, sensorData):
    minDelta = -1.

    # ******************Compute r and phi**********************
    r = np.linalg.norm(Xi - X_robot)
    #phi = np.arctan2(Xi[1] - X_robot[1], Xi[0] - X_robot[0]) - theta
    phi = math.atan2(Xi[1] - X_robot[1], Xi[0] - X_robot[0]) - theta

    for i in range(num_rays):
        sensorTheta = sensorThetas[i]
        if abs(phi - sensorTheta) < minDelta or minDelta == -1:
            Zk = sensorData[i] * 100
            thetaK = sensorTheta
            minDelta = abs(phi - sensorTheta)

    # ******************Evaluate the three cases**********************
    #print(Xi, r, Zk)
    #import pdb; pdb.set_trace()
#    if r > min(float(Zmax), Zk + alpha / 2) or abs(phi - thetaK) > beta / 2 or Zk > Zmax or Zk < Zmin:
    if r > min(float(Zmax), Zk + 0.5) or abs(phi - thetaK) > beta / 2 or Zk > Zmax or Zk < Zmin:
        return l0
#    elif Zk < Zmax and abs(r - Zk) < alpha / 2:
    elif Zk < Zmax and abs(r - Zk) < 0.5:
        return locc
    elif r <= Zk:
        return lfree

    return l0

# ******************Code the Occupancy Grid Mapping Algorithm**********************
def occupancyGridMapping(Robotx, Roboty, Robottheta, sensorData):
    X_robot = np.array([Robotx, Roboty]) * 100
    X_robot[0] = X_robot[0] + 250
    X_robot[1] = X_robot[1] + 200
    for y in range(500):
        for x in range(500):
            xi = x + 0.5
            yi = y + 0.5

#    for y in range(int(mapHeight / gridHeight)):
#        for x in range(int(mapWidth / gridWidth)):
#            xi = int(x * gridWidth + gridWidth / 2 - robotXOffset)
#            yi = int(-(y * gridHeight + gridHeight / 2) + robotYOffset)
            #xi = int(x * gridWidth + gridWidth / 2)
            #yi = int(y * gridHeight + gridHeight / 2)
            Xi = np.array([xi, yi])
#            distance = np.linalg.norm(Xi - X_robot)
#            print(X_robot)
#            print(Xi)
#            print(distance)
#            import pdb; pdb.set_trace()
#            if distance > Zmax:
#                continue
            try:
#                print(f"not continue {x}, {y}")
                l_array[y][x] = l_array[y][x] + \
                    inverseSensorModel(X_robot, Robottheta, Xi, sensorData) - l0
            except Exception as e:
                print(str(x) + "," + str(y) )
                print(e)
                return False
    return True


def visualization(time_idx):
    # Graph Format
#    plt.title("Map")
#    plt.xlim(0, int(mapWidth / gridWidth))
#    plt.ylim(0, int(mapHeight / gridHeight))
#    map_array = np.zeros([
#        int(mapHeight/gridHeight),
#        int(mapWidth/gridWidth),
#        3],
#        dtype=np.uint8
#    )
#    # Draw every grid of the map:
#    for y in range(int(mapHeight / gridHeight)):
#        #print("Remaining Rows= " + str(mapHeight / gridHeight - y))
#        for x in range(int(mapWidth / gridWidth)):
#            if l_array[y][x] == 0: # gray unknown state
#                map_array[y][x] = 100
#                #plt.plot(x, y, 'g.')
#            elif l_array[y][x] > 0: # black occupied state
#                map_array[y][x] = 0
#                #plt.plot(x, y, 'k.')
#            else: # white free state
#                map_array[y][x] = 255
#                #plt.plot(x, y, 'r.')

    map_array = np.zeros([
        500,
        500,
        3],
        dtype=np.uint8
    )
    map_array[...] = 100

    # Draw every grid of the map:
    for y in range(500):
        for x in range(500):
            if l_array[y][x] == 0: # gray unknown state
                map_array[y][x] = 100
            elif l_array[y][x] > 0: # black occupied state
                map_array[y][x] = 0
            else: # white free state
                map_array[y][x] = 255

    cv2.imwrite(f"maps/map_{time_idx:05d}.jpg", map_array)

#    # Save the image and close the plot
#    plt.savefig("./Images/Map1.png")
#    plt.clf()






