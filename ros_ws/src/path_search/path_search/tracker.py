#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from std_msgs.msg import Header
from rclpy.time import Time, Duration

# from nav_msgs.msg import Odometry
from collections import deque
import math


DST_POS = (1, 9)


class AStartSearchNode(Node):

    # SLAM 맵 10X10
    SLAM_MAP = [
        [0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 1, 0],
        [0, 1, 1, 0, 0, 1, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    ]
    direction: str
    rotate_state: bool

    def __init__(self):
        super().__init__("path_finder_astar")
        queue_size = 10
        # odom 위치정보 취득
        qos_profile = QoSProfile(depth=queue_size)
        self.create_subscription(
            TwistStamped,
            "/unity_tf",
            callback=self._unity_tf_sub_callback,
            qos_profile=qos_profile,
        )
        self.pub_jetauto_car = self.create_publisher(
            Twist, "jetauto_car/cmd_vel", queue_size
        )

        # SLAM 맵에서 -90도 회전된 각도 (라디안 단위)
        rotation_angle = -math.pi / 2

        # 회전 변환 행렬
        self.rotation_matrix = [
            [math.cos(rotation_angle), -math.sin(rotation_angle)],
            [math.sin(rotation_angle), math.cos(rotation_angle)],
        ]

        self.get_logger().info(f"AStartpath_searchNode started...")
        self.twist_msg = Twist()
        self.setup()

    def setup(self):
        self._direction: str = "x"
        self.change_direction(direction=self._direction, state=False)
        self.last_message_time: Time = Time(nanoseconds=0)
        self.old_pos: tuple[int, int] = (-1, -1)
        self.previos_pos: tuple[float, float] = None
        self.complete: bool = False
        self.prev_angular_z: float = -math.pi
        self.amend_count: int = 0

    @classmethod
    def change_direction(cls, *, direction: str = None, state: bool = None):
        if direction:
            cls.direction = direction
        if state is not None:
            cls.rotate_state = state

    def _unity_tf_sub_callback(self, input_msg: TwistStamped):

        message_time = Time.from_msg(input_msg.header.stamp)

        # time_diff = message_time.nanoseconds - self.last_message_time.nanoseconds
        # if time_diff < Duration(nanoseconds=100000000).nanoseconds:  # 0.1초
        #     self.get_logger().info(f" nanoseconds passed")
        #     return
        # else:
        #     self.last_message_time = message_time

        # msg로부터 위치정보를 추출
        current_pos = (input_msg.twist.linear.x, input_msg.twist.linear.y)

        # 좌표변환
        new_cor = self._get_cordinate(current_pos)

        # 회전여부 파악
        stop_rotating = False
        if self.rotate_state:
            if self.is_rotating(input_msg.twist.angular):
                return
            else:
                stop_rotating = True

        self.prev_angular_z = input_msg.twist.angular.z
        self.change_direction(direction=self._direction, state=False)

        # 초기설정
        # if new_cor == (0, 0) and self.old_pos == (-1, -1):
        #     self.get_logger().info(f"초기설정")
        #     self.setup()

        if new_cor == DST_POS:
            if not self.complete:
                self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
                self.twist_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
                self.pub_jetauto_car.publish(self.twist_msg)
                self.get_logger().info(f"도착지[{new_cor}]에 도착했습니다.")
                self.complete = True
            return
        elif new_cor == self.old_pos:
            # 수평위치 보정
            self._amend_h_pos(input_msg.twist.angular)
            if not stop_rotating:
                return

        self.old_pos = new_cor

        # 다음 위치
        paths = self.find_path(new_cor, DST_POS)
        self.get_logger().info(f"A* PATH: {paths}")

        if len(paths) == 0:
            self.get_logger().info(f"길찾기 실패: 목표위치({DST_POS})")
            return

        npos1, npos2 = paths[1], paths[2] if len(paths) > 2 else None
        x, theta = self._get_torq_theta(new_cor, npos1, None)

        self.get_logger().info(f"현재:{new_cor}, 1차: {npos1}")
        self.get_logger().info(f"토크: {x}, 회전: {theta}")

        # 메시지 발행
        self.twist_msg.linear = Vector3(x=x, y=0.0, z=0.0)
        self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=theta)
        self.pub_jetauto_car.publish(self.twist_msg)

        self.get_logger().info(f"publish_msg: {self.twist_msg}")

    # 회전여부를 파악한다.
    def is_rotating(self, angular: object):
        def angle_diff(angle1, angle2):
            if angle1 < angle2:
                angle1 += 2 * math.pi

            _diff = angle1 - angle2
            return (
                _diff
                if abs(_diff) < math.pi
                else (2 * math.pi - abs(_diff)) * _diff / abs(_diff)
            )

        rotation = angle_diff(angular.z, self.prev_angular_z)
        rotation_angle = math.pi / 2
        if rotation_angle * 0.8 < rotation:
            break_torque = self.get_break_torque(rotation, rotation_angle)
            self.get_logger().info(f"break rot_torque: {break_torque}")

            self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
            self.twist_msg.linear = Vector3(x=0.0, y=0.0, z=break_torque)
            self.pub_jetauto_car.publish(self.twist_msg)

        # magnitude = math.sqrt(x**2 + y**2 + z**2)
        result = rotation < rotation_angle * 0.92

        self.get_logger().info(f"angular_diff: {rotation} / {result}")
        return result

    def get_break_torque(
        self,
        angle: float,
        target_angle: float,
        max_torque: float = 0.05,
        min_torque: float = 0.00,
    ):
        """
        현재 회전각도에 따라 토크의 크기를 결정한다.
        회전각도가 90도에 가까워질수록 토크의 크기를 로그형태로 줄여나간다.
        회전각도가 0도에 가까울수록 토크의 크기는 최대값(2.0)에 가깝고, 90도에 가까워질수록 토크의 크기는 최소값(0.3)에 가깝다.
        """
        _torque = (
            (max_torque - min_torque)
            * math.log(angle + 1.0)
            / math.log(target_angle + 1.0)
        )
        return -_torque if _torque > min_torque else -min_torque

    # 몸체 평행상태 조정
    def _amend_h_pos(self, angular) -> tuple:
        if self.rotate_state:
            return

        angular_z = -angular.z % (math.pi * 2)
        # self.get_logger().info(f"angular:{angular}, magnitude: {magnitude}")
        amend_theta = 0.0
        dir_text = ""
        if self.direction == "x":
            amend_theta = -(math.pi - angular_z)
            # z값이 이전치와 차이가 어느정도 나면 보정한다.
            dir_text = "Z방향"

        elif self.direction == "-x":
            angular_z = angular_z if angular_z > math.pi else math.pi * 2 - angular_z
            amend_theta = -(math.pi * 2 - angular_z)
            dir_text = "-Z방향"

        elif self.direction == "y":
            amend_theta = -(math.pi / 2 - angular_z)
            # z값이 이전치와 차이가 어느정도 나면 보정한다.
            dir_text = "Y방향"

        elif self.direction == "-y":
            amend_theta = math.pi * 3 / 4 - angular_z
            dir_text = "-Y방향"

        if self.amend_count % 3 == 0 and abs(amend_theta) > 5 / 180 * math.pi:
            self.get_logger().info(
                f"amend_theta: {angular_z} || {amend_theta} /{5 / 360 * 2 * math.pi}"
            )
            self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=amend_theta * 3 / 5)
            self.twist_msg.linear = Vector3(x=0.5, y=0.0, z=0.0)
            self.pub_jetauto_car.publish(self.twist_msg)
            self.get_logger().info(f"{dir_text} 위치보정")
            self.amend_count += 1

    """현재위치, 다음위치, 그 다음위치를 입력으로 받는다.
       앞방향 토크와 방향 각도를 반환한다.
    """

    def _get_torq_theta(
        self, cord0: tuple, cord1: tuple, cord2: tuple
    ) -> tuple[float, float]:
        # 다음위치와 그 다음위치를 확인해 회전방향을 계산한다.
        self.get_logger().info(f"_get_torq_theta_direction: {self.direction}")
        x0, y0 = cord0
        # x1, y1 = cord1
        # x2, y2 = cord2
        x1, y1 = cord2 if cord2 else cord1

        rot_torque = 0.6
        rot_theta = 1.4  # 57도
        str_torque = 0.5
        if self.direction == "x":
            if y1 > y0:
                dir = rot_torque, rot_theta  # 좌회전
                self._direction = "y"
                self.change_direction(state=True)
            elif y1 < y0:
                dir = rot_torque, -rot_theta  # 우회전
                self._direction = "-y"
                self.change_direction(state=True)
            else:
                dir = str_torque, 0.0  # 직진
                self.change_direction(direction=self._direction, state=False)

        elif self.direction == "-x":
            if y1 > y0:
                dir = rot_torque, -rot_theta  # 우회전
                self._direction = "y"
                self.change_direction(state=True)
            elif y1 < y0:
                dir = rot_torque, rot_theta  # 좌회전
                self._direction = "-y"
                self.change_direction(state=True)
            else:
                dir = str_torque, 0.0  # 직진
                self.change_direction(state=False)

        elif self.direction == "y":
            if x1 > x0:
                dir = rot_torque, -rot_theta  # 우회전
                self._direction = "x"
                self.change_direction(state=True)
            elif x1 < x0:
                dir = rot_torque, rot_theta  # 좌회전
                self._direction = "-x"
                self.change_direction(state=True)
            else:
                dir = str_torque, 0.0  # 직진
                self.change_direction(direction=self._direction, state=False)

        elif self.direction == "-y":
            if x1 > x0:
                dir = rot_torque, rot_theta  # 좌회전
                self._direction = "x"
                self.change_direction(state=True)
            elif x1 < x0:
                dir = rot_torque, -rot_theta  # 우회전
                self._direction = "-x"
                self.change_direction(state=True)
            else:
                dir = str_torque, 0.0  # 직진
                self.change_direction(direction=self._direction, state=False)

        if dir[1] != 0:
            import time

            # breaking
            self.get_logger().info(f"breaking: {-str_torque * 2 / 3}")
            self.twist_msg.linear = Vector3(x=-str_torque * 2 / 3, y=0.0, z=0.0)
            self.twist_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
            # self.pub_jetauto_car.publish(self.twist_msg)
            # time.sleep(0.1)

        return dir

    # 좌표취득
    def _get_cordinate(self, cord: tuple[float, float]):
        # self.get_logger().info(f"_axis_transform: {cord}")
        # PC 맵 좌표를 SLAM 맵 좌표로 변환
        _x = math.floor(5.0 - cord[0])  # z
        _y = math.floor(5.0 - cord[1])  # y

        result = (_x if _x > 0 else 0, _y if _y > 0 else 0)

        # self.get_logger().info(f"cord: {cord} ==> {result}")

        return result

    def _axis_transform2(self, cord: tuple[float, float]):
        self.get_logger().info(f"_axis_transform: {cord}")
        # PC 맵 좌표를 SLAM 맵 좌표로 변환
        _x = self.rotation_matrix[0][0] * cord[0] + self.rotation_matrix[0][1] * cord[1]
        _y = self.rotation_matrix[1][0] * cord[0] + self.rotation_matrix[1][1] * cord[1]

        result = int(round(_x + 4.6, 0)), int(round(_y + 4.6, 0))
        self.get_logger().info(f"_axis_transform Result: {result}")
        return result

    # 전치
    def _transpose(self, matrix: list[list[int]]):
        rows_A = len(matrix)
        cols_A = len(matrix[0])
        T = [[0 for _ in range(rows_A)] for _ in range(cols_A)]

        for i in range(rows_A):
            for j in range(cols_A):
                T[j][i] = matrix[i][j]

        return T

    # a* search
    def find_path(self, start: tuple[int, int], goal: tuple[int, int]):
        """
        A* 알고리즘을 사용하여 최단 경로를 찾습니다.
        :param grid: 2D 그리드 맵
        :param start: 시작 지점 (x, y)
        :param goal: 목표 지점 (x, y)
        :return: 최단 경로
        """
        self.get_logger().info(f"find_path: {start}, goal: {goal}")
        grid = self.SLAM_MAP

        frontier = deque()
        frontier.append((start, [start]))  # 큐에 시작 위치와 경로 추가
        visited = set()  # 방문한 노드 집합
        is_first_search: bool = True
        while frontier:
            curr_node, path = frontier.popleft()

            if curr_node == goal:
                return path

            if curr_node in visited:
                continue

            visited.add(curr_node)

            x, y = curr_node
            for next_x, next_y in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if (
                    0 <= next_x < len(grid)
                    and 0 <= next_y < len(grid[0])
                    and grid[next_x][next_y] != 1
                ):
                    # 최초 다음위치에서 self.direction의 반대방향을 제외시킨다.
                    if is_first_search:
                        is_first_search = False
                        if self.direction == "x" and next_x == x - 1:
                            continue
                        elif self.direction == "-x" and next_x == x + 1:
                            continue
                        elif self.direction == "y" and next_y == y - 1:
                            continue
                        elif self.direction == "-y" and next_y == y + 1:
                            continue

                    frontier.append(((next_x, next_y), path + [(next_x, next_y)]))
                    frontier = deque(
                        sorted(
                            frontier,
                            key=lambda x: len(x[1])
                            + self._heuristic_distance(x[0], goal),
                        )
                    )

        return []

    def _heuristic_distance(self, a, b):
        """
        목표점까지의 추정거리를 추정한다
        """
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)


def main(args=None):
    rclpy.init(args=args)

    path_node = AStartSearchNode()
    rclpy.spin(path_node)

    path_node.destroy_node()
    rclpy.shutdown()
