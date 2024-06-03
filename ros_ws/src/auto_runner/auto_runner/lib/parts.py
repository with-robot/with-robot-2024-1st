from enum import Enum
from typing import TypeVar, Sequence
from auto_runner.lib.common import StateData, MessageHandler, State, Orient, DirType
import math
import threading
import time
import asyncio

state: State = State.ROTATE_STOP


class SensorData:
    tf_sensor: any
    lidar: any

    @classmethod
    def update_data(cls, tf_data: object, scan_data: any):
        cls.tf_sensor = tf_data
        cls.lidar = scan_data


class RotationManger:
    observers: list[object]
    start_angle: float

    def __init__(self, messagenser: MessageHandler):
        self.cmd = messagenser

    # 비동기로 callback
    def add_observer(self, o: object):
        self.observers.append(o)
        self.start_angle = SensorData.tf_sensor.angle

    def notifyall(self, m: any):
        for o in self.observers:
            o.update(m)

    def start(self, callback: object, dir:Orient, orient:DirType=DirType.LEFT):
        self.dir=dir
        self.orient=orient
        self.add_observer(callback)

        global state
        while state != State.ROTATE_STOP:
            time.sleep(0.1)

        state = State.ROTATE_START

        threading(target=self.run).start()

    def run(self):
        old_angle_diff = 0.001
        while True:
            # 회전각 계산
            angle_diff = self.calc_misalignment()

            # 회전완료 여부 체크
            if angle_diff > (math.pi / 2) * 0.75 or (angle_diff - old_angle_diff)/old_angle_diff <= 0.05:
                global state
                state = State.ROTATE_STOP
                self.notifyall(angle_diff)
                break

            self.roate_angle(0.15)
            old_angle_diff = angle_diff

            time.sleep(0.1)

    def roate_angle(self, torque: float):
        self.cmd(torque=torque, angle=1.35)

    def calc_misalignment(self):
        cur_angle = SensorData.tf_sensor.angle
        return cur_angle - self.start_angle

    # 막다른 골목위치 체크
    def check_obstacle(self):
        # 후진모드설정
        pass


class MoveManager:
    def __init__(self, node: MessageHandler, state_data: StateData):
        self.node = node
        self.state_data = state_data


class ObstacleManager:
    def __init__(self, node: MessageHandler, state_data: StateData):
        self.node = node
        self.state_data = state_data

    # 회전과 이동이 구독
    def add_observer(self, o: object):
        self.observers.append(o)
