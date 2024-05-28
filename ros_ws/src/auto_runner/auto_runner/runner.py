import math
import re
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from yolov8_msgs.srv import CmdMsg
from sensor_msgs.msg import LaserScan
from collections import deque
from nav_msgs.msg import OccupancyGrid
from auto_runner.map_transform import occ_gridmap
from auto_runner.mmr_sampling import find_farthest_coordinate
from auto_runner.lib import car_drive, common, path_location

laser_scan: LaserScan = None
grid_map: list[list[int]] = None


class GridMap(Node):
    def __init__(self) -> None:
        super().__init__("grid_map_node")

        # 기본 콜백 그룹 생성
        self._default_callback_group = ReentrantCallbackGroup()

        # 맵 수신 10*10
        self.map_pub = self.create_subscription(
            OccupancyGrid,
            "/occ_grid_map",
            self.receive_map,
            10,
        )

    def receive_map(self, map: OccupancyGrid):
        global grid_map
        raw_data = np.array(map.data, dtype=np.int8)
        grid_map = occ_gridmap(raw_data)


class LidarScanNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_scan_node")

        # 기본 콜백 그룹 생성
        self._default_callback_group = ReentrantCallbackGroup()

        # lidar subscriber
        self.create_subscription(
            LaserScan,
            "jetauto/scan",
            callback=self.scan_handler,
            qos_profile=10,
        )

        self.get_logger().info(f"LidarScanNode started...")

    def scan_handler(self, msg: LaserScan):
        global laser_scan
        laser_scan = msg


class AStartSearchNode(Node):

    ROT_TORQUE = 0.3  # 회전토크
    ROT_THETA = 1.10  # 회전각도크기
    FWD_TORQUE = 0.1  # 직진방향 토크

    def __init__(self) -> None:
        super().__init__("path_search_astar")
        queue_size = 1
        # odom 위치정보 취득
        qos_profile = QoSProfile(depth=queue_size)
        self.create_subscription(
            TwistStamped,
            "/unity_tf",
            callback=self.unity_tf_sub_callback,
            qos_profile=qos_profile,
        )
        self.pub_jetauto_car = self.create_publisher(
            Twist, "jetauto_car/cmd_vel", queue_size
        )

        self.srv_jetauto_car = self.create_service(
            CmdMsg, "jetauto_car/cmd_msg", self.send_command
        )

        self.get_logger().info(f"AStartpath_searchNode started...")
        self.twist_msg = Twist()
        self.setup()

        _path_finder = path_location.PathFinder(self, algorithm="a-start")
        self.robot_ctrl = car_drive.RobotController(self, _path_finder, common.Dir.X)


    def unity_tf_sub_callback(
        self, msg: TwistStamped
    ) -> None:  # msg로부터 위치정보를 추출
        if not grid_map:
            return
        # self.robot_ctrl = self.robot_ctrl
        # pfinder = self.path_find
        # 센서 데이터
        self.robot_ctrl.set_tfdata(msg)
        self.robot_ctrl.update_map(grid_map)

        if self.robot_ctrl.check_rotate_state():
            return

        if self.is_near():
            # 급 감속
            self._send_message(title="급감속", x=-self.FWD_TORQUE * 4 / 5)

        # 직진, 방향 보정
        self.robot_ctrl.adjust_body()

        # 로봇 다음동작
        try:
            x, theta = self.robot_ctrl.next_action()
        except Exception as e:
            self.get_logger().info(f"controller stopped:{e}")
            # 맵 생성까지 대기
            return

        if self.robot_ctrl.is_rotate_state():
                self._send_message(title="회전전 감속", x=-self.FWD_TORQUE * 4 / 5)

        # 제어메시지 발행
        self._send_message(title="주행지시", x=x, theta=theta)


    # 로봇이 전방물체와 50cm이내 접근상태이면 True를 반환
    def is_near(self) -> bool:
        if laser_scan:
            time = Time.from_msg(laser_scan.header.stamp)
            # Run if only laser scan from simulation is updated
            if time.nanoseconds > self.nanoseconds:
                self.nanoseconds = time.nanoseconds

                ### Driving ###
                nearest_distance = 0.5  # 50cm
                win = 8  # # of elements in forward, left, right sensor groups
                length = len(laser_scan.ranges)
                center = length // 2
                distances = []
                for i in range(length // 8):
                    if i * win < center and (i + 1) * win > center:
                        shift1, shift2 = 0, 1
                    elif (i + 1) * win < center:
                        shift1, shift2 = 0, 0
                    else:
                        shift1, shift2 = 1, 1
                    distances.append(
                        min(
                            laser_scan.ranges[win * i + shift1 : win * (i + 1) + shift2]
                        )
                    )
                self.get_logger().info(
                    f"distances:{distances[4:6]}/기준:{nearest_distance}"
                )
                return min(distances[4:6]) <= nearest_distance

        return False

    def setup(self) -> None:
        self.nanoseconds = 0
        # self.dest_pos = (2, 9)
        self.get_logger().info("초기화 처리 완료")

    def send_command(self, request, response) -> None:
        self.get_logger().info(f"request: {request}")
        response.success = True
        if request.message == "reset":
            self.setup()
        elif request.message == "set_dest":
            x = int(request.data.x)
            y = int(request.data.y)
            self.set_destpos((x, y))
        elif request.message == "set_torque":
            self.FWD_TORQUE = request.data.x
        else:
            response.success = False

        return response
    
    def _send_message(
        self, *, title: str = None, x: float = 0.0, theta: float = 0.0
    ) -> None:
        self.get_logger().info(f"#### {title} ### torque:{x}, angular:{theta} ####")
        self.twist_msg.linear = Vector3(x=x, y=0.0, z=0.0)
        self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=theta)
        self.pub_jetauto_car.publish(self.twist_msg)

    def print_log(self, message) -> None:
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)

    path_node = AStartSearchNode()
    lidar_node = LidarScanNode()
    grid_node = GridMap()

    executor = MultiThreadedExecutor()
    executor.add_node(grid_node)
    executor.add_node(path_node)
    executor.add_node(lidar_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        path_node.destroy_node()
        rclpy.shutdown()
