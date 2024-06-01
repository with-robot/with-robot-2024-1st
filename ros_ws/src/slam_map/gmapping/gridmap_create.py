#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from gmapping.utils_lib.gridmap import GridMap
import math
import numpy as np
import os


class GridmapPubNode(Node):
    def __init__(self, scan_topic, world_frame_id, cell_size):
        super().__init__("gridmap_make_node")

        self.world_frame_id = world_frame_id
        self.occ_grid_pub = self.create_publisher(
            OccupancyGrid, "/occ_grid_map", qos_profile=10
        )
        self.create_subscription(
            TwistStamped,
            "unity_tf",
            self.odom_callback,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.receive_scan, qos_profile=10
        )

        self.pub_timer = self.create_timer(0.2, self.publish_gridmap)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self, qos=10)
        self.cell_size = cell_size
        self.nanoseconds = 0
        self.gridmap = None
        self.odometry = None
        
        self.prepare_log_file()
        self.get_logger().info(f"Gridmap node has started: {world_frame_id}")

    def prepare_log_file(self):
        pkg_base = get_package_share_directory("gmapping")
        self.measurement_file = os.path.join(pkg_base, "resource", "measurements.txt")
        if os.path.exists(self.measurement_file):
            os.remove(self.measurement_file)

        self.pose_file = os.path.join(pkg_base, "resource", "pose.txt")
        if os.path.exists(self.pose_file):
            os.remove(self.pose_file)

        self.occ_map_file = os.path.join(pkg_base, "resource", "occ_map.txt")

    def odom_callback(self, msg):
        self.odometry = msg

    def lidar_callback(self, msg):
        self.laser_scan = msg

    def receive_scan(self, scan):

        time = Time.from_msg(scan.header.stamp)
        # Run if only laser scan from simulation is updated
        # 0.2초단위로 수신
        if (
            self.odometry == None
            or self.odometry.twist.linear.x == None
            or time.nanoseconds <= self.nanoseconds
        ):
            return

        self.nanoseconds = time.nanoseconds

        def euler_from_quaternion(x, y, z, w) -> tuple:
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)

            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)

            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)

            return roll_x, pitch_y, yaw_z  # in radians

        try:
            # if self.tfBuffer.can_transform(self.world_frame_id, scan.header.frame_id, scan.header.stamp):
            #     trans = self.tfBuffer.lookup_transform(self.world_frame_id, scan.header.frame_id, scan.header.stamp)
            # else:            
            #     now = self.get_clock().now().to_msg()
            #     trans = self.tfBuffer.lookup_transform(self.world_frame_id, scan.header.frame_id, now)

            # __, __, yaw = euler_from_quaternion(
            #     trans.transform.rotation.x,
            #     trans.transform.rotation.y,
            #     trans.transform.rotation.z,
            #     trans.transform.rotation.w,
            # )
            # x = self.sub_odom.linear.x
            # y = self.sub_odom.linear.y
            # current_pos = (self.odometry.twist.linear.x+5.0, 5.0-self.odometry.twist.linear.y)
            # 좌표를 +축으로 이동. (-5 ~ 5 ) ==> (0, 10)
            current_pos = (self.odometry.twist.linear.x, self.odometry.twist.linear.y)
            theta = self.odometry.twist.angular.z
            # if (self.nanoseconds // 1_000_000_000) % 3 == 0:
            #     self.get_logger().info(f"gridmap current_pos: {current_pos}, angular: {theta}")


            if self.gridmap is None:
                self.gridmap = GridMap(
                    center=(0,0),
                    logger=self.get_logger(),
                    cell_size=0.1,
                    map_width=10,
                )

            # ranges는 65개
            robot_body_offset = 0.1

            for i, range_ in enumerate(scan.ranges):
                # if r <= scan.range_max and r >= scan.range_min:
                # if 0.0 < range_ + robot_body_offset < 10.0:
                    # 방사 각도
                yaw_ray = (theta + scan.angle_min + i * scan.angle_increment)
                # self.get_logger().info(f"scan: {range_}, yaw_ray: {yaw_ray}")
                # 위치, 각도, 길이, threathold확율값
                self.gridmap.add_ray(current_pos, yaw_ray, range_, 0.7)

            # pose와 measurement를 저장한다.
            # self.save_pose_measurement(scan)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().info(f"tf2 exception: {e}")
            return

    # 위치와 센서측정거리를 파일로 저장한다.
    def save_pose_measurement(self, scan):
        with open(self.measurement_file, "a") as f:
            f.write(f"{self.nanoseconds}")
            for distance in scan.ranges:
                f.write(f",{distance:0.3f}")
            f.write("\n")

        with open(self.pose_file, "a") as f:
            f.write(f"{self.nanoseconds}")
            f.write(f",{self.odometry.twist.linear.x}")
            f.write(f",{self.odometry.twist.linear.y}")
            f.write(f",{self.odometry.twist.angular.z}")
            f.write("\n")
        
        # self.get_logger().info(f"saved")

    """주기적으로 작성 중인 맵을 발행"""

    def publish_gridmap(self, event=None):

        if self.gridmap is None:
            return
        # 초기 0으로 설정되었다가,
        # 빈 셀인경우 0.847씩 차감하고, 대상 셀인 경우, 0.847씩 가산한다. (0.8은 70%에 대한 로그 확율치)
        # 셀이 갱신되어 감에 따라, -6.91이 되어 가거나 +6.91이 되어간다.
        # 최소값 -6.91을 0으로 변환. 0 ~ 13.82이 됨.
        # 13.82*7.23 = 100 => 0~100% 확율로 변환 50%이면 미 확인셀이되는 셈.
        # Reshape and scale the Gridmap_ 한행으로 변환, 각 셀은 0 ~ 100의 값을 갖는다.
        reshaped_gridmap = (self.gridmap.get_map().reshape((1, -1))[0] + 6.91) * 7.23

        # Unexplored cells are set to negative 1
        # 초기값 0인 상태: 비탐색 지역
        for index, key in enumerate(reshaped_gridmap):
            if key == 6.91 * 7.23:
                reshaped_gridmap[index] = -1

        # 그리드맵을 작성
        occ_grid = OccupancyGrid()
        occ_grid.header.frame_id = self.world_frame_id
        occ_grid.header.stamp = self.get_clock().now().to_msg()
        # resolution means the size of each cell
        occ_grid.info.resolution = self.cell_size
        occ_grid.info.width = int(self.gridmap.map_width[0] / self.cell_size)
        occ_grid.info.height = int(self.gridmap.map_width[1] / self.cell_size)
        occ_grid.info.origin.position.x = self.gridmap.origin[0]
        occ_grid.info.origin.position.y = self.gridmap.origin[1]
        occ_grid.info.origin.position.z = 0.0
        occ_grid.info.origin.orientation.x = 0.0
        occ_grid.info.origin.orientation.y = 0.0
        occ_grid.info.origin.orientation.z = 0.0
        occ_grid.info.origin.orientation.w = 1.0
        # map data is the occupancy grid map
        occ_grid.data = reshaped_gridmap.astype(dtype=np.int8).tolist()

        # Publishing the message
        self.occ_grid_pub.publish(occ_grid)

        self.save_map(occ_grid.data)

    def save_map(self, data):
        data_str=' '.join(str(x) for x in data)
        with open(self.occ_map_file, "w") as f:
            f.write(data_str)


def main(args=None):
    rclpy.init(args=args)

    driver = GridmapPubNode(
        scan_topic="jetauto/scan", world_frame_id="odom", cell_size=0.1
    )
    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()
