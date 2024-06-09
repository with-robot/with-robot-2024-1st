from enum import Enum, auto
from typing import TypeVar, Sequence
from auto_runner.lib.common import StateData, MessageHandler, State, Orient, DirType
from auto_runner.lib.map import Map
from auto_runner.lib.path_location import PathFinder
from auto_runner.lib.car_drive import RobotController
import math
import threading
import time
import asyncio


state: State = State.ROTATE_STOP

class PolicyType(Enum):
    ROTATE =auto()
    STRAIGHT = auto()
    BACk = auto()
    

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
    @classmethod
    def add_observer(cls, o: object):
        cls.observers.append(o)

    @classmethod
    def notifyall(self, m: any):
        for o in self.observers:
            o.update(m)

    def start(self, callback: object, dir: Orient, orient: DirType = DirType.LEFT):
        self.dir = dir
        self.orient = orient
        self.target_angle = self._get_target_angle(dir, orient)
        self.add_observer(callback)

        global state
        while state != State.ROTATE_STOP:
            time.sleep(0.1)

        state = State.ROTATE_START

        threading(target=self.run).start()

    def run(self):
        while True:
            # 회전각 계산
            angle_diff = self._get_diff()

            # 회전완료 여부 체크
            if abs(angle_diff) < 0.2:
                global state
                state = State.ROTATE_STOP
                self.notifyall(angle_diff)
                break

            sign_ = math.copysign(1, angle_diff)
            self._rotate_angle(torque=0.15, sign=sign_)

            time.sleep(0.1)

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

    def _rotate_angle(self, torque: float, sign: int):
        self.cmd(torque=torque, angle=1.35 * sign)

    # 반환값이 양수면 반시계방향으로 보정해야함
    def _get_diff(self):
        cur_angle = SensorData.tf_sensor.angle % (2 * math.pi)
        cur_angle = (
            cur_angle if cur_angle > self.target_angle else (cur_angle + 2 * math.pi)
        )
        return self.target_angle - cur_angle

class PolicyResult:
    policy:'Policy'
    is_done:bool
    kwargs:dict

    def __init__(self, policy:'Policy', apply_result:bool, kwargs:dict):
        self.policy = policy
        self.is_done = apply_result
        self.kwargs = kwargs

# 다음 처리방향을 세운다.
class Policy:
    apply_result:bool
    apply_kwargs:dict

    def __init__(self, policy:PolicyType):
        _action_map = {
            PolicyType.ROTATE: self.rotate,
            PolicyType.STRAIGHT: self.move_straight,
            PolicyType.BACk: self.move_back,
        }
            
        self.policy_plan = _action_map.get(policy)
        self.rcntrler = RobotController()

    def apply(self, **kwargs):
        # 3. 단기 목표설정
        self.policy_plan(**kwargs)
        self.apply_kwargs = kwargs

    def report(self):
        return PolicyResult(policy=self, apply_result=self.apply_result, kwargs=self.apply_kwargs)
    
    def move_back(self, **kwargs):
        pass

    def move_straight(self, **kwargs):
        self.apply_result = self.rcntrler.check_straight_state(**kwargs)
    
    def rotate(self, **kwargs):        
        self.apply_result = self.rcntrler.check_rotate_state(**kwargs)
    

class MoveManager:
    def __init__(self):
        # 맵을 가진다.
        self.pathfinder = PathFinder(algorithm='a-star', dest_pos=(0,0))

    def pos(self, pos: tuple=(0,0)):
        self.cur_pos = self.pathfinder.set_cur_pos(pos)

    def move(self, dest_pos: tuple) -> PolicyResult:

        try:
            self.check_arrived(dest_pos)

            # 경로와 방향을 정한다.
            policy:Policy = self.prepare_policy(dest_pos)

            return policy.report()
            
        except Exception as e:
            if e == "arrived":
                return
            elif e=='no policy':
                return
            print(e)

    def prepare_policy(self, dest_pos: tuple) -> PolicyType:
        # 가야 할 전략
        _policy:PolicyType = None
        
        # 1. 후진 계획
        if self.pathfinder.is_intrap():
            self.path = self.pathfinder.back_path()
            _policy = Policy(PolicyType.BACk)
            return

        # 2. 전진 계획
        _path = self.pathfinder.find_path(dest_pos)

        if self.check_rotate_policy(_path):
            _policy = Policy(PolicyType.ROTATE)
        else:
            _policy = Policy(PolicyType.STRAIGHT)
        
        if _policy == None:
            raise Exception("no policy")


    def check_arrived(self, dest_pos: tuple):
        if self.pathfinder.cur_pos == dest_pos:
            self.notifyall(dest_pos)
            raise Exception("arrived")
        
    def check_rotate_policy(self, path:list[tuple]) -> bool:
        if self.path[0] == path[1]:
            return False
        else:
            return True

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
