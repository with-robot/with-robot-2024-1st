import cv2
import numpy as np


def draw_line(
    map_array,
    x1,
    y1,
    x2,
    y2,
    line_color,
    start_point_color,
    end_point_color,
):
    theta = np.arctan2(y2 - y1, x2 - x1) # y/x convention

    if theta == 0:
        a = 0
        b = 1
        c = -y1
    elif theta == np.pi:
        a = 0
        b = 1
        c = -y1
    elif theta == np.pi / 2:
        a = 1
        b = 0
        c = -x1
    elif theta == -np.pi / 2:
        a = 1
        b = 0
        c = -x1
    else:
        a1 = (y2 - y1) / (x2 - x1)
        b1 = y1 - a1 * x1
        a = a1
        b = -1
        c = b1

    def f(x, y):
        return a * x + b * y + c

    def is_on_ray(xmin, ymin, xmax, ymax):
        if f(xmin, ymax) * f(xmin, ymin) <= 0:
            return True
        elif f(xmin, ymin) * f(xmax, ymin) <= 0:
            return True
        elif f(xmax, ymin) * f(xmax, ymax) <= 0:
            return True
        elif f(xmax, ymax) * f(xmin, ymax) <= 0:
            return True
        return False

    image_h, image_w = map_array.shape[:2]
    x_list = range(int(x1), min(int(x2)+1, image_w)) \
        if x1 < x2 else range(int(x1), max(int(x2)-1, 0), -1)
    y_list = range(int(y1), min(int(y2)+1, image_h)) \
        if y1 < y2 else range(int(y1), max(int(y2)-1, 0), -1)

    def is_in_image(x, y):
        if x < 0 or y < 0 or x >= image_w or y >= image_h:
            return False
        return True

    xy_list = []
    if np.abs(np.tan(theta)) <= 1:
        for _x in x_list:
            tmp_y = int(-(a * _x + c) / b)
            tmp_y_list = [tmp_y-1, tmp_y, tmp_y+1]
            for _y in tmp_y_list:
                if not is_in_image(_x, _y):
                    continue
                xmin, ymin = _x, _y - 1
                xmax, ymax = _x + 1, _y
                if is_on_ray(xmin, ymin, xmax, ymax):
                    xy_list.append((_x, _y))
    else:
        for _y in y_list:
            tmp_x = int(-(b * _y + c) / a)
            tmp_x_list = [tmp_x-1, tmp_x, tmp_x+1]
            for _x in tmp_x_list:
                if not is_in_image(_x, _y):
                    continue
                xmin, ymin = _x, _y - 1
                xmax, ymax = _x + 1, _y
                if is_on_ray(xmin, ymin, xmax, ymax):
                    xy_list.append((_x, _y))

    xs = np.array(xy_list)[:, 0]
    ys = np.array(xy_list)[:, 1]
    map_array[ys, xs] = line_color

    ix1 = int(x1)
    iy1 = int(y1)
    ix2 = int(x2)
    iy2 = int(y2)
    if is_in_image(ix1, iy1):
        map_array = cv2.circle(
            map_array, (ix1, iy1), 2, color=start_point_color, thickness=-1
        )
    if is_in_image(ix2, iy2):
        map_array = cv2.circle(
            map_array, (ix2, iy2), 2, color=end_point_color, thickness=-1
        )


def convert_distance_to_global_coordinates(d, theta, R, robot_position):
    # Convert distance to local coordinates (assuming 2D LIDAR)
    x_prime = d * np.cos(theta)
    y_prime = d * np.sin(theta)
    z_prime = 0

    # ray end point position in lidar view
    P_in_lidar_view = np.array([x_prime, y_prime, z_prime])

    # unity:lidar_link
    # unity:lidar_link_sensor
    translation_robot_view_to_lidar_view = np.array([
        (0.09593944 + 0.05), # unity z -> our x
        (-1.03744e-05), # unity x -> our y
        0.1283962, # unity y -> our z
    ])

    P_in_robot_view = P_in_lidar_view + \
        translation_robot_view_to_lidar_view

    # Translate based on robot's global position
    P_global = R.dot(P_in_robot_view) + robot_position

    return P_global


def visualization_pose(
    map_array,
    time_idx,
    robot_x,
    robot_y,
    robot_z,
    rot_mat,
    robot_theta,
    sensorThetas,
    measurements,
    out_dir,
):
    scale = 100

    image_h, image_w = map_array.shape[:2]

    robot_position_in_global_view = np.array([robot_x, robot_y, robot_z])

    lidar_position_in_lidar_view = np.array([0, 0, 0])

    # unity:lidar_link
    # unity:lidar_link_sensor
    translation_robot_view_to_lidar_view = np.array([
        (0.09593944 + 0.05), # unity z -> our x
        (-1.03744e-05), # unity x -> our y
        0.1283962, # unity y -> our z
    ])
    lidar_position_in_robot_view = lidar_position_in_lidar_view + \
        translation_robot_view_to_lidar_view

    lidar_position_in_global = rot_mat.dot(lidar_position_in_robot_view) + \
        robot_position_in_global_view

    # Top-view: x, y coordinate
    # Transform it to map coordinate
    x1 = int(lidar_position_in_global[0] * scale + int(image_w / 2))
    y1 = int(lidar_position_in_global[1] * scale + int(image_h / 2))

    for idx, distance in enumerate(measurements):
        sensor_theta = sensorThetas[idx]
        ray_end_point_global = convert_distance_to_global_coordinates(
            distance,
            sensor_theta,
            rot_mat,
            robot_position_in_global_view,
        )
        x2 = int(ray_end_point_global[0] * scale + int(image_w / 2))
        y2 = int(ray_end_point_global[1] * scale + int(image_h / 2))

        # Draw
        draw_line(
            map_array,
            x1,
            y1,
            x2,
            y2,
            line_color=(255, 255, 255),
            start_point_color=(255, 0, 0),
            end_point_color=(0, 0, 0),
        )

    # Draw virtual line to show robot's pose
    ray_end_point_global = convert_distance_to_global_coordinates(
        1, # virutal distance
        0,
        rot_mat,
        robot_position_in_global_view,
    )
    x2 = int(ray_end_point_global[0] * scale + int(image_w / 2))
    y2 = int(ray_end_point_global[1] * scale + int(image_h / 2))
    draw_line(
        map_array,
        x1,
        y1,
        x2,
        y2,
        line_color=(0, 0, 255),
        start_point_color=(255, 0, 0),
        end_point_color=(0, 255, 0),
    )

    # Visualization
    cv2.imwrite(f"{out_dir}/pose_{time_idx:05d}.png", map_array)


def visualization_position_only(
    map_array,
    time_idx,
    robot_x,
    robot_y,
):
    image_h, image_w = map_array.shape[:2]
    # Draw every grid of the map:
    x1 = int(robot_x * 100 + int(image_w / 2))
    y1 = int(robot_y * 100 + int(image_h / 2))

    ix1 = int(x1)
    iy1 = int(y1)
    start_point_color = (255, 0, 0)
    map_array = cv2.circle(
        map_array, (ix1, iy1), 2, color=start_point_color, thickness=-1
    )

    cv2.imwrite(f"position_maps/{time_idx:05d}.png", map_array)


def get_rotation_matrix(roll, pitch, yaw):
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)],
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)],
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1],
    ])
    R = Rz @ Ry @ Rx
    return R


def get_robot_pose(data):
    timeStamp = data[0]
    _x, _y, _z, _rx, _ry, _rz = data[1:]

    robotX = float(_x)
    robotY = float(_y)
    robotZ = float(_z)

    rx = float(_rx)
    ry = float(_ry)
    rz = float(_rz)
    R = get_rotation_matrix(rx, ry, rz)
    return timeStamp, robotX, robotY, robotZ, R, rz


def parse_measurment_data(data):
    measurement_time_stamp = data[0]
    measurement_raw_data = data[1:]
    result = [float(m) for m in measurement_raw_data]
    return result


def find_corresponding_measurement_idx(p_time_stamp, measurement_list):
    pose_time = float(p_time_stamp)
    min_diff = np.inf
    min_idx = -1
    for idx, _m in enumerate(measurement_list):
        m_time_stamp = _m[0]
        measurement_time = float(m_time_stamp)
        diff = abs(pose_time - measurement_time)
        if diff < min_diff:
            min_diff = diff
            min_idx = idx
    return min_idx, min_diff

