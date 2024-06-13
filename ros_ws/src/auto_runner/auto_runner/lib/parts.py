from enum import Enum, auto
from abc import abstractmethod, ABCMeta
from typing import TypeVar, Sequence
from auto_runner.lib.common import StateData, MessageHandler, State, Orient, DirType
from auto_runner.lib.map import Map
from auto_runner.lib.path_location import PathFinder
import math
import threading
import time

state: State = State.ROTATE_STOP


class PolicyType(Enum):
    ROTATE = auto()
    STRAIGHT = auto()
    BACk = auto()


class MapPosData:
    map: list[list[int]]
    pos: list[list[tuple[float, float]]]

    @classmethod
    def update_map(cls, map_data: list):
        cls.map = map_data

    @classmethod
    def update_pos(cls, pos_data: list):
        cls.pos = pos_data


class SensorData:
    tf_sensor: list[float, float, float]
    lidar: list[float]

    @classmethod
    def update_lidar(cls, lidar: object):
        cls.lidar = lidar

    @classmethod
    def update_tf(cls, tf_data: object):
        cls.tf_sensor = [
            tf_data.twist.linear.x,
            tf_data.twist.linear.y,
            tf_data.twist.angular.z,
        ]


class Message:
    cmd: dict
    done: bool

    def __init__(self, cmd: dict = {}, done: bool = False):
        self.cmd = cmd
        self.done = done


class Observer:
    def update(self, message: Message):
        """"""


class Observable(ABCMeta):
    _observe_map: dict[str, list[Observer]]

    def __new__(cls):
        cls._observe_map = {}

    # 비동기로 callback
    @classmethod
    def add_observer(cls, n: str, o: Observer):
        cls._observe_map.get(n, []).append(o)

    @classmethod
    def notifyall(cls, n: str, message: Message):
        for o in cls._observe_map.get(n, []):
            o.update(message)


class EvHandle(ABCMeta, threading.Thread):
    _lock = threading.Lock()

    def __init__(self, stop_event: threading.Event):
        super().__init__()
        self._stop_event = stop_event

    def add(self, callback: object, **kwargs):
        self._stop_event.clear()
        Observable.add_observer(self.__class__, callback)
        self._start(**kwargs)

    @abstractmethod
    def _start(self, **kwargs):
        """"""


class MoveAction(EvHandle):
    def __init__(self):
        super().__init__()

    def _start(self, **kwargs):
        # 좌표변환 기능 tf -> (x,y)
        self.dest_pos = kwargs.get("dest_pos", (0, 0))
        self.orient = kwargs.get("orient", Orient.X)
        self.direction = 1 if kwargs.get("direction", DirType.St"forward") == "forward" else -1
        # self.pos_data: StateData = StateData()

    def run(self):
        if state != State.ROTATE_STOP:
            self._stop_event.set()
            time.sleep(0.3)
            self._stop_event.clear()

        global state
        state = State.RUN
        while not self._stop_event.is_set():
            with self._lock:
                # 도착 여부 체크
                if PathFinder._transfer2_xy(SensorData.tf_sensor[:2]) == self.dest_pos:
                    break
                
                amend_theta:float = self._adjust_body()
                self._move(torque=0.15 * self.direction, theta = amend_theta)

            time.sleep(0.1)

        Observable.notifyall(__class__, Message(done=True))
        state = State.STOP

    def _move(self, torque: float, theta: float):
        # 전진
        Observable.notifyall(__class__, Message(cmd={"torque": torque, "theta": theta}))

    # 수평상태
    def _adjust_body(self):
        amend_theta = self._get_deviation_radian()

        # 간격을 두어 보정한다.
        # 잦은 조정에의한 좌우 흔들림 방지.
        if abs(amend_theta) > 3 / 180 * math.pi:
            # self.node.print_log(
            #     f"[{self.dir_data.cur} 위치보정] {self.dir_data} >> 보정 각:{amend_theta} / 기준:{3 / 180 * math.pi}"
            # )
            pass
        else:
            amend_theta =0

        return amend_theta

    # 각도를 입력받아, 진행방향과 벗어난 각도를 반환한다.
    def _get_deviation_radian(self) -> tuple[float, str]:
        # 유니티에서 로봇의 기본회전상태가 180 CCW상태, 즉 음의 값.
        # 회전방향 수치를 양수로 정리한다.
        # -0.1 => 6.23
        _cur_angle = self.angular_data.cur % (math.pi * 2)
        _cur_dir = self.dir_data.cur
        _cur_angle = SensorData.tf_sensor[-1]
        _cur_dir = self.orient

        if _cur_dir == Orient.X:
            amend_theta = (
                -_cur_angle if _cur_angle < math.pi / 2 else 2 * math.pi - _cur_angle
            )

        elif _cur_dir == Orient._X:
            amend_theta = math.pi - _cur_angle

        elif _cur_dir == Orient.Y:
            # -180 ~ -270
            # 차이량에 대해 음수면 바퀴가 우측방향으로 돌게된다. (우회전으로 보정)
            amend_theta = math.pi / 2 - _cur_angle

        elif _cur_dir == Orient._Y:
            amend_theta = math.pi * 3 / 2 - _cur_angle

        else:
            amend_theta = 0.0

        return amend_theta


class RotateAction(EvHandle):
    start_angle: float

    def __init__(self):
        super().__init__()

    # def start(self, callback: object, dir: Orient, orient: DirType = DirType.LEFT):
    #     self.dir = dir
    #     self.orient = orient
    #     self.target_angle = self._get_target_angle(dir, orient)
    #     self.add_observer(callback)

    #     global state
    #     while state != State.ROTATE_STOP:
    #         time.sleep(0.1)

    #     state = State.ROTATE_START

    #     threading(target=self.run).start()

    def run(self):
        if state != State.ROTATE_STOP:
            return

        global state
        state = State.ROTATE_START

        while not self._stop_event.is_set():
            with self._lock:
                # 회전각 계산
                angle_diff = self._get_diff()

                # 회전완료 여부 체크
                if abs(angle_diff) < 0.2:
                    break

                sign_ = math.copysign(1, angle_diff)
                self._rotate_angle(torque=0.15, direction=sign_)

            time.sleep(0.1)

        self.notifyall(Message(done=True))
        state = State.ROTATE_STOP

    # 목표 각도 설정
    def _get_target_angle(self, orient: Orient, dir: DirType) -> float:
        angle_map = {
            (Orient.X, DirType.LEFT): math.pi / 2,
            (Orient._X, DirType.RIGHT): math.pi / 2,
            (Orient.Y, DirType.LEFT): math.pi,
            (Orient._Y, DirType.RIGHT): math.pi,
            (Orient.X, DirType.RIGHT): 3 * math.pi / 2,
            (Orient._Y, DirType.LEFT): 3 * math.pi / 2,
        }

        key = (orient, dir)
        if key in angle_map:
            return angle_map[key]
        else:
            return 2 * math.pi

    def _rotate_angle(self, torque: float, direction: int):
        self.notifyall(message=dict(torque=torque, angle=1.35 * direction))

    # 반환값이 양수면 반시계방향으로 보정해야함
    def _get_diff(self):
        cur_angle = SensorData.tf_sensor.angle % (2 * math.pi)
        cur_angle = (
            cur_angle if cur_angle > self.target_angle else (cur_angle + 2 * math.pi)
        )
        return self.target_angle - cur_angle


class PolicyResult:
    policy: "Policy"
    is_done: bool
    kwargs: dict

    def __init__(self, policy: "Policy", apply_result: bool, kwargs: dict):
        self.policy = policy
        self.is_done = apply_result
        self.kwargs = kwargs


# 다음 처리방향을 세운다.
class Policy:
    apply_result: bool
    apply_kwargs: dict

    def __init__(self, policy: PolicyType):
        _action_map = {
            PolicyType.ROTATE: self.rotate,
            PolicyType.STRAIGHT: self.move_straight,
            PolicyType.BACk: self.move_back,
        }

        self.policy_plan = _action_map.get(policy)
        self.rotate_manager = RotateAction(self.rcntrler.cmd)
        self.move_manager = MoveAction(self.rcntrler.cmd)

    def apply(self, **kwargs):
        # 3. 단기 목표설정 TODO:
        self.policy_plan(dir=kwargs.get("dir"), orient=kwargs.get("orient"))
        self.apply_kwargs = kwargs

    def report(self):
        return PolicyResult(
            policy=self, apply_result=self.apply_result, kwargs=self.apply_kwargs
        )

    def move_back(self, **kwargs):
        pass

    def move_straight(self, **kwargs):
        self.move_manager.start(**kwargs)
        # self.apply_result = self.rcntrler.check_straight_state(**kwargs)

    def rotate(self, **kwargs):
        self.rotate_manager.start(**kwargs)
        # self.apply_result = self.rcntrler.check_rotate_state(**kwargs)


class RobotContler:
    def __init__(self):
        # 맵을 가진다.
        self.pathfinder = PathFinder(algorithm="a-star", dest_pos=(0, 0))

    def pos(self, pos: tuple = (0, 0)):
        self.cur_pos = self.pathfinder.set_cur_pos(pos)

    def operate(self, dest_pos: tuple) -> PolicyResult:

        try:
            self.check_arrived(dest_pos)

            # 경로와 방향을 정한다.
            policy: Policy = self._policy(dest_pos)
            kwargs = {"callback": self.callback_func, "dir": "", "orient": ""}
            policy.apply(**kwargs)

            return policy.report()

        except Exception as e:
            print(e)
            if e == "arrived":
                return
            elif e == "no policy":
                return

    def _policy(self, dest_pos: tuple) -> PolicyType:

        def __check_rotate_policy(self, path: list[tuple]) -> bool:
            if self.path[0] == path[1]:
                return False
            else:
                return True

        # 가야 할 전략
        _policy: PolicyType = None

        # 1. 후진 계획
        if self.pathfinder.is_intrap():
            self.pathfinder.back_path()
            _policy = Policy(PolicyType.BACk)
        else:

            # 2. 전진 계획
            _path = self.pathfinder.find_path(dest_pos)

            if __check_rotate_policy(_path):
                _policy = Policy(PolicyType.ROTATE)
            else:
                _policy = Policy(PolicyType.STRAIGHT)

            if _policy == None:
                raise Exception("no policy")

        return _policy

    def check_arrived(self, dest_pos: tuple):
        if self.pathfinder.cur_pos == dest_pos:
            self.notifyall(dest_pos)
            raise Exception("arrived")

    def callback_func(self, **kwargs):
        print(kwargs)

    def notifyall(self, m: any):
        for o in self.observers:
            o.update(m)


class ObstacleManager:
    def __init__(self, node: MessageHandler, state_data: StateData):
        self.node = node
        self.state_data = state_data

    # 회전과 이동이 구독
    def add_observer(self, o: object):
        self.observers.append(o)
