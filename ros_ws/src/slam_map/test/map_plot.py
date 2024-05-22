import math
import matplotlib.pyplot as plt
from pathlib import Path

# Sensor characteristic: Min and Max ranges of the beams
Zmax = float(10)
Zmin = float(0.3)

# Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
l0 = float(0)
locc = 0.4
lfree = -0.4

# Grid dimensions
gridWidth = float(0.1)
gridHeight = float(0.1)

# Map dimensions
mapWidth = float(10)
mapHeight = float(10)

# Robot size with respect to the map
robotXOffset = mapWidth / 10  # 6000
robotYOffset = mapHeight / 10  # 5000

# Defining an l vector to store the log odds values of each cell
l = [
    [l0 for j in range(int(mapHeight / gridHeight))]
    for i in range(int(mapWidth / gridWidth))
]

base_path = Path("ros_ws/src/slam_map/resource/map")


def inverseSensorModel(x, y, theta, xi, yi, sensorData):
    # Defining Sensor Characteristics
    # double Zk, thetaK, sensorTheta;
    minDelta = float(-1)
    alpha = float(1)
    beta = float(90)

    # ******************Compute r and phi**********************
    r = math.sqrt(math.pow(xi - x, 2) + math.pow(yi - y, 2))
    phi = math.atan2(yi - y, xi - x) - theta

    # Scaling Measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
    for i in range(8):
        if i == 0:
            sensorTheta = -90 * (math.pi / 180)
        elif i == 1:
            sensorTheta = -37.5 * (math.pi / 180)
        elif i == 6:
            sensorTheta = 37.5 * (math.pi / 180)
        elif i == 7:
            sensorTheta = 90 * (math.pi / 180)
        else:
            sensorTheta = (-37.5 + (i - 1) * 15) * (math.pi / 180)

        if abs(phi - sensorTheta) < minDelta or minDelta == -1:
            Zk = sensorData[i]
            thetaK = sensorTheta
            minDelta = abs(phi - sensorTheta)

    # ******************Evaluate the three cases**********************
    if (
        r > min(float(Zmax), Zk + alpha / 2)
        or abs(phi - thetaK) > beta / 2
        or Zk > Zmax
        or Zk < Zmin
    ):
        return l0
    elif Zk < Zmax and abs(r - Zk) < alpha / 2:
        return locc
    elif r <= Zk:
        return lfree

    return l0


# ******************Code the Occupancy Grid Mapping Algorithm**********************
def occupancyGridMapping(Robotx, Roboty, Robottheta, sensorData):
    for x in range(int(mapWidth / gridWidth)):
        for y in range(int(mapHeight / gridHeight)):
            xi = int(x * gridWidth + gridWidth / 2 - robotXOffset)
            yi = int(-(y * gridHeight + gridHeight / 2) + robotYOffset)
            if math.sqrt(math.pow(xi - Robotx, 2) + math.pow(yi - Roboty, 2)) <= Zmax:
                try:
                    l[x][y] = (
                        l[x][y]
                        + inverseSensorModel(
                            Robotx, Roboty, Robottheta, xi, yi, sensorData
                        )
                        - l0
                    )
                except Exception as e:
                    print(str(x) + "," + str(y))
                    print(e)
                    return False

    return True


def visualization():
    # Graph Format
    plt.title("Map")
    plt.xlim(0, int(mapWidth / gridWidth))
    plt.ylim(0, int(mapHeight / gridHeight))

    # Draw every grid of the map:
    for x in range(int(mapWidth / gridWidth)):
        print("Remaining Rows= " + str(mapWidth / gridWidth - x))
        for y in range(int(mapHeight / gridHeight)):
            if l[x][y] == 0:  # Green unkown state
                plt.plot(x, y, "g.")
            elif l[x][y] > 0:  # Black occupied state
                plt.plot(x, y, "k.")
            else:  # Red free state
                plt.plot(x, y, "r.")

    # Save the image and close the plot
    plt.savefig(base_path / "Map5.png")
    plt.clf()


posesFile = open(base_path / "poses.txt", "r")
measurementFile = open(base_path / "measurement.txt", "r")
measurementData = []
count = 0
while True:

    measurementData.clear()
    linePose = posesFile.readline()
    if not linePose:
        break

    poses = linePose.strip().split(" ")
    timeStamp = poses[0]
    robotX = float(poses[1])
    robotY = float(poses[2])
    robotTheta = float(poses[3])  # y축

    lineMeasure = measurementFile.readline()
    measurements = lineMeasure.strip().split(" ")

    if timeStamp != measurements[0]:
        break

    for i in range(8):
        measurementData.append(float(measurements[i + 1]))

    result = occupancyGridMapping(robotX, robotY, robotTheta, measurementData)
    # result = occupancyGridMapping(
    #     robotX, robotY, (robotTheta / 10) * (math.pi / 180), measurementData
    # )
    print(f"count:{count}")
    count += 1
    if result == False:
        break

visualization()

measurementFile.close()
posesFile.close()
