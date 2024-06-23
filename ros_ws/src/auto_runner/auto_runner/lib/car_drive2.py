from rclpy.node import Node
from auto_runner.lib.common import TypeVar
from auto_runner.lib.parts import *
from auto_runner import mmr_sampling
from auto_runner.lib.path_manage import PathManage

LoggableNode = TypeVar("LoggableNode", bound=MessageHandler)
print_log = mmr_sampling.print_log


class RobotController2:
    action_map = {
        DirType.LEFT: (1.0, 1.35),
        DirType.RIGHT: (1.0, -1.35),
        DirType.FORWARD: (0.6, 0.0),
    }

    def __init__(self, node: Node):
        # 맵을 가진다.
        self.node = node
        self.pathMnger = PathManage(algorithm="a-star", dest_pos=(0, 0))
        self.orient: Orient = Orient.X
        self.dir_type = DirType.FORWARD
        self.cur_pos = (0,0)
        self.path = [(-1,-1)]
        IMUData.subscribe(o=self.get_data)

    def get_data(self, message: Message):
        self.imu_data = message.data

    def prepare(self):
        # 방향, 현재위치, 회전각도 등 취합
        self.cur_pos: tuple = self.pathMnger.transfer2_xy(self.imu_data[:2])
        self.angular_data: float = self.get_data[2]
        print_log(f"<<prepare>> {self.cur_pos} : {self.angular_data}")
        # 목표경로를 정한다.
        self.path = self.pathMnger.search_new_path(self.cur_pos, self.orient)

    def make_plan(self) -> Policy:
        """후진, 회전, 직진여부를 체크하고 해당 policy를 반환한다"""
        policy: EvHandle = Policy.check_paths(self.orient, self.cur_pos, self.path)
        self.dir_type, self.origin = policy.action

        print_log(f"<<make_plan>> policy {policy} :action {policy.action}")
        return policy

    def excute(self, next_plan: EvHandle, **kwargs):
        print_log(f"<<excute>> {next_plan}")
        next_plan.apply(**kwargs)

    @property
    def check_arrived(self):
        return self.cur_pos == self.path[-1]
