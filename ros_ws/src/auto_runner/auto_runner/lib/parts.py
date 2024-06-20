from enum import Enum, auto
from typing import TypeVar, Sequence
from auto_runner.lib.common import (
    EvHandle,
    StateData,
    MessageHandler,
    State,
    Orient,
    DirType,
    Observable,
    Message,
    Observer,
    Chainable
)
from auto_runner.lib.map import Map
from auto_runner.lib.path_location import PathFinder
import math
import time
import asyncio

state: StateData = StateData(State.ROTATE_STOP, State.ROTATE_STOP)
orient: StateData = StateData(Orient.X, Orient.X)


class PolicyType(Enum):
    ROTATE = "rotate"
    STRAIGHT = "move"
    BACk = "go_back"


class MapData(Observable):
    _subject = "map"


class PosData(Observable):
    _subject = "pos"


class LidarData(Observable):
    _subject = "lidar"


class IMUData(Observable):
    _subject = "imu"

    @classmethod
    async def update(cls, data: object):
        _data = [
            data.twist.linear.x,
            data.twist.linear.y,
            data.twist.angular.z,
        ]
        await cls.notifyall(Message(data_type=cls._subject, data=_data))


class MoveAction(EvHandle, Observer, Chainable):
    
    def __init__(self, orient:Orient, cur_pos:tuple, paths:list[tuple]):
        super().__init__()
        self.orient = orient
        self.cur_pos = cur_pos
        self.paths = paths
        self.next_pos = paths[1]
        self.imu_data = None
    
    def setup(self, **kwargs):        
        IMUData.add_observer(o=self)  

    def check_condition(self):
        self.action = DirType.FORWARD, self.orient
        (x0, y0), (x1, y1) = self.cur_pos, self.next_pos

        if self.orient in [Orient.X, Orient._X]:
            return y0 != y1
        elif orient in [Orient.Y, Orient._Y]:
            return x0 != x1        
        return False
        
    def run(self):
        if state != State.ROTATE_STOP:
            # 회전중이라면 회전을 중지시킨다.
            self.stop_event.set()
            time.sleep(0.3)
            self.stop_event.clear()

        state.shift(State.RUN)
        while not self.stop_event.is_set():
            if not self.imu_data:
                time.sleep(0.05)
                continue
            _xy, _angle = self.imu_data[:2], self.imu_data[-1]
            with self._lock:
                # 도착 여부 체크
                if PathFinder._transfer2_xy(_xy) == self.next_pos:
                    break

                amend_theta: float = self._adjust_body(self.next_orient, _angle)

                self._notifyall(
                    "node",
                    Message(
                        title="move",
                        data_type="command",
                        data=dict(torque=0.1, theta=amend_theta),
                    ),
                )

        self._notifyall("node", Message(title="완료", data_type="notify", data="move"))

        state.shift(State.ROTATE_STOP)
        IMUData.remove_observer(o=self)

    # 수평상태
    def _adjust_body(self, orient: Orient, angle: float):
        amend_theta: float = self._get_deviation_radian(orient, angle)

        # 간격을 두어 보정한다.
        # 잦은 조정에의한 좌우 흔들림 방지.
        if abs(amend_theta) > 3 / 180 * math.pi:
            # self.node.print_log(
            #     f"[{self.dir_data.cur} 위치보정] {self.dir_data} >> 보정 각:{amend_theta} / 기준:{3 / 180 * math.pi}"
            # )
            pass
        else:
            amend_theta = 0

        return amend_theta

    # x,y,angle.z
    async def update(self, message: Message):
        if message.data_type != "imu":
            return

        self.imu_data = message.data if self.imu_data == message.data else None

    # 각도를 입력받아, 진행방향과 벗어난 각도를 반환한다.
    def _get_deviation_radian(self, orient: Orient, angle: float) -> float:
        # 유니티에서 로봇의 기본회전상태가 180 CCW상태, 즉 음의 값.
        # 회전방향 수치를 양수로 정리한다.
        # -0.1 => 6.23
        _angle = angle % (math.pi * 2)

        if orient == Orient.X:
            amend_theta = -_angle if _angle < math.pi / 2 else 2 * math.pi - _angle

        elif orient == Orient._X:
            amend_theta = math.pi - _angle

        elif orient == Orient.Y:
            # -180 ~ -270
            # 차이량에 대해 음수면 바퀴가 우측방향으로 돌게된다. (우회전으로 보정)
            amend_theta = math.pi / 2 - _angle

        elif orient == Orient._Y:
            amend_theta = math.pi * 3 / 2 - _angle

        else:
            amend_theta = 0.0

        return amend_theta


class RotateAction(EvHandle, Observer, Chainable):
    start_angle: float
    imu_data: any = None

    def __init__(self, orient:Orient, cur_pos:tuple, paths:list[tuple]):
        super().__init__()
        self.orient = orient
        self.cur_pos = cur_pos
        self.paths = paths
        self.next_pos = paths[1]
        
    def setup(self, **kwargs):        
        IMUData.add_observer(o=self)    

    def check_condition(self, orient:Orient, cur_pos:tuple, paths:list[tuple]):

        (x0, y0), (x1, y1) = cur_pos, paths[1]

        action = None
        if orient == Orient.X:
            if y1 > y0:
                action = DirType.LEFT, Orient.Y  # 좌회전
            elif y1 < y0:
                action = DirType.RIGHT, Orient._Y  # 우회전

        elif orient == Orient._X:
            if y1 > y0:
                action = DirType.RIGHT, Orient.Y  # 우회전
            elif y1 < y0:
                action = DirType.LEFT, Orient._Y  # 좌회전

        elif orient == Orient.Y:
            if x1 > x0:
                action = DirType.RIGHT, Orient.X  # 우회전
            elif x1 < x0:
                action = DirType.LEFT, Orient._X  # 좌회전

        elif orient == Orient._Y:
            if x1 > x0:
                action = DirType.LEFT, Orient.X  # 좌회전
            elif x1 < x0:
                action = DirType.RIGHT, Orient._X  # 우회전

        if action is None:
            return False

        self.action = action
        return True
    

    def run(self):
        if state != State.ROTATE_STOP:
            return

        state.shift(State.ROTATE_START)

        while not self.stop_event.is_set():
            if not self.imu_data:
                time.sleep(0.05)
                continue
            _fr_angle = self.imu_data[-1]
            _to_angle = self._get_target_angle(orient.cur, self.dir)
            with self._lock:
                # 회전각 계산
                angle_diff: float = self._get_diff(_fr_angle, _to_angle)

                # 회전완료 여부 체크
                if abs(angle_diff) < 0.2:
                    break

                # sign_ = math.copysign(1, angle_diff) * self.direction
                self._notifyall(
                    "node",
                    Message(
                        data_type="command",
                        data=dict(name="회전", torque=0.15, theta=angle_diff),
                    ),
                )

        self._notifyall("node", Message(data_type="notify", data=dict(name="rotate")))

        state.shift(State.ROTATE_STOP)
        IMUData.remove_observer(o=self)

    # 목표 각도 설정
    def _get_target_angle(self, orient: Orient, dir: DirType) -> float:
        _angle_map = {
            (Orient.X, DirType.LEFT): math.pi / 2,
            (Orient._X, DirType.RIGHT): math.pi / 2,
            (Orient.Y, DirType.LEFT): math.pi,
            (Orient._Y, DirType.RIGHT): math.pi,
            (Orient.X, DirType.RIGHT): 3 * math.pi / 2,
            (Orient._Y, DirType.LEFT): 3 * math.pi / 2,
        }

        _key = (orient, dir)
        return _angle_map[_key] if _key in _angle_map else 2 * math.pi

    # 반환값이 양수면 반시계방향으로 보정해야함
    def _get_diff(self, angle: float, target_angle: float) -> float:
        cur_angle = angle % (2 * math.pi)
        cur_angle = cur_angle if cur_angle > target_angle else (cur_angle + 2 * math.pi)
        return target_angle - cur_angle

    # x,y,angle.z
    async def update(self, message: Message):
        if message.data_type == "map":
            pass
        elif message.data_type == "pos":
            self.tf_data = message.data if self.tf_data == message.data else None


class BackMoveAction(EvHandle, Observer, Chainable):
    start_angle: float
    imu_data: any = None

    def __init__(self):
        super().__init__()
        IMUData.add_observer(o=self)

    def check_condition(self, orient:Orient, cur_pos:tuple, paths:list[tuple]):
        (x0, y0), (x1, y1) = cur_pos, paths[1]
        
        _trap_map = {            
            orient.Y: self.map[x0][y0-1],
            orient._Y: self.map[x0][y0+1],
            orient.X: self.map[x0-1][y0],
            orient._X: self.map[x0+1][y0],
        }
        
        return all( v>0 for k,v in _trap_map.pop(orient).items())        
    
# class PolicyResult:
#     policy: "Policy"
#     is_done: bool
#     kwargs: dict

#     def __init__(self, policy: "Policy", apply_result: bool, kwargs: dict):
#         self.policy = policy
#         self.is_done = apply_result
#         self.kwargs = kwargs


# 다음 처리방향을 세운다.
class Policy:

    @property
    def all_policies(self) -> list[Chainable]:
        return [
            RotateAction,
            MoveAction,
            BackMoveAction,
        ]
    def setup(self, policy: PolicyType, **kwargs):
        if not hasattr(self, policy.value):
            raise Exception(f"Policy {policy} is not implemented.")
        # factory 패턴
        self.action: EvHandle = getattr(self, policy.value)(**kwargs)

    def check_paths(self, orient_state, cur_pos, paths:list[tuple]) -> EvHandle:
        # 체인 패턴
        for p in self.all_policies:
            if p.check_condition(orient_state=orient_state, cur_pos=cur_pos, paths=paths):
                return p

class ObstacleManager:
    def __init__(self, node: MessageHandler, state_data: StateData):
        self.node = node
        self.state_data = state_data

    # 회전과 이동이 구독
    def add_observer(self, o: object):
        self.observers.append(o)
