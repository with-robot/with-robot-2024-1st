import json
import os
from time import time
import cv2
import numpy as np

import sys
library_path = "/root/Workspace/with-robot-2024-1st/ros_ws/src/mapping"
if not library_path in sys.path:
    sys.path.append(library_path)
from utils import (
    visualization_pose,
    get_robot_pose,
    parse_measurment_data,
    find_corresponding_measurement_idx,
)


if __name__=="__main__":

    s_total_time = time()

    # Prepare output directories
    out_root = "output"
    os.makedirs(out_root, exist_ok=True)

    out_pose_map_dir = os.path.join(out_root, "pose_maps")
    os.makedirs(out_pose_map_dir, exist_ok=True)

    out_position_map_dir = os.path.join(out_root, "position_maps")
    os.makedirs(out_position_map_dir, exist_ok=True)

    # Load sensor info
    with open("sensor.json") as fp:
        sensor = json.load(fp)

    # Sensor characteristic: Min and Max ranges of the beams
    Zmax = sensor["range_max"] # minimum range value [m]
    Zmax = 10000
    Zmin = sensor["range_min"] # maximum range value [m]
    num_rays = sensor["num_rays"] # number of rays of lidar sensor

    # TODO: I can't understand angle_min, angle_max, angle_increment recieved from unity
    # So, assign theta by dividing pi equally
    #angle_min = sensor["angle_min"] # start angle of the scan [rad]
    #angle_max = sensor["angle_max"] # end angle of the scan [rad]
    #angle_increment = sensor["angle_increment"] # angular distance between measurements [rad]
    angle_min = -np.pi / 2
    angle_max = np.pi / 2
    angle_increment = np.pi / (num_rays - 1)
    sensorThetas = []
    for i in range(num_rays):
        sensorThetas.append(angle_min + i * angle_increment)

    # Initialize map
    pose_map_array = np.zeros([
        500,
        500,
        3],
        dtype=np.uint8
    )
    pose_map_array[...] = 100

    # Fetch pose data
    pose_list = []
    with open("input/poses.txt") as fp:
        lines = fp.readlines()
        for line in lines:
            poses = line.strip().split(" ")
            pose_list.append(poses)

    # Fetch measurement data
    measurement_list = []
    with open("input/measurement.txt") as fp:
        lines = fp.readlines()
        for line in lines:
            measurements = line.strip().split(" ")
            measurement_list.append(measurements)

    #time_diff_threshold = 504970.5 # median
    #time_diff_threshold = 406980.0 + 20000
    time_diff_threshold = np.inf

    for i in range(len(pose_list)):
        if i < 20:
            continue
        print(f"[{i}/{len(pose_list)}]")

        s_time = time()

        p_time_stamp, robotX, robotY, robotZ, rot_mat, robotTheta = get_robot_pose(pose_list[i])
        m_idx, time_diff = find_corresponding_measurement_idx(
            p_time_stamp,
            measurement_list
        )

        if time_diff > time_diff_threshold:
            continue

        measurements = measurement_list[m_idx]
        measurementData = parse_measurment_data(measurements)

        visualization_pose(
            pose_map_array,
            i,
            robotX,
            robotY,
            robotZ,
            rot_mat,
            robotTheta,
            sensorThetas,
            measurementData,
            out_pose_map_dir,
        )

            #visualization(time_idx)

    #        visualization_position_only(
    #            pose_map_array,
    #            i,
    #            robotX,
    #            robotY,
    #        )
#            print(i, robotTheta)
            #pose_map_array[...] = 100

#            index = pose_map_array != 100
#            float_map = pose_map_array.astype(np.float32)
#            pose_map_array[index] = (float_map[index] * 0.9).astype(np.uint8)

#            for _r in range(360):
#                visualization_pose(pose_map_array, i, robotX, robotY, _r, [])

    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 0, [])
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 270 * np.pi / 180, [])
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 89 * np.pi / 180, [])
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 90 * np.pi / 180, [])
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 91 * np.pi / 180, [])
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 45 * np.pi / 180)
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 60 * np.pi / 180)
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 90 * np.pi / 180)
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 130 * np.pi / 180)
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 150 * np.pi / 180)
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 180 * np.pi / 180)
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 200 * np.pi / 180)
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 270 * np.pi / 180)
    #        visualization_pose(pose_map_array, time_idx, robotX, robotY, 300 * np.pi / 180)

        e_time = time()
        elp_m = (e_time - s_time) // 60
        elp_s = (e_time - s_time) % 60
        print(f"iter {i} elp: {elp_m}m {elp_s}s")

    e_total_time = time()
    elp_m = (e_total_time - s_total_time) // 60
    elp_s = (e_total_time - s_total_time) % 60
    print(f"iter elp: {elp_m}m {elp_s}s")
