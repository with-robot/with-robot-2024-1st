import math, re
from collections import deque
from auto_runner.lib.common import *
from auto_runner.lib.parts import *
from auto_runner.lib.path_location import PathFinder
from rclpy.node import Node
from ros_ws.src.auto_runner.auto_runner import mmr_sampling

LoggableNode = TypeVar("LoggableNode", bound=MessageHandler)


class PathManage:
    def __init__(self, algorithm: str = "astar"):
        MapData.add_observer(self)
        self.path = []
        if algorithm == "astar":
            self.pathfinder = self._astar_method

    @property
    def grid_map(self):
        if self.updated:
            self.updated = False
            return self.grid_map

    def update(self, message: Message):
        if message.data_type == "map" and self.updated == False:
            self.grid_map = message.data
            self.updated = True

    @property
    def all_path(self) -> list:
        return self.path

    def search_new_path(self, cur_pos: tuple, cur_orient: Orient) -> tuple:
        mmr_sampling.print_log(f"<<check_and_dest>> pos:{cur_pos}, dir:{cur_orient}")

        path = []
        count = 3
        exclude = self.paths if len(self.paths) > 0 else [cur_pos]
        while len(path) == 0 and count > 0:
            dest_pos = mmr_sampling.find_farthest_coordinate(
                self.grid_map, cur_pos, exclude
            )
            if not dest_pos:
                mmr_sampling.print_log(f"mmr_sampling failed: map:{self.grid_map}")
                break

            path = self.pathfinder(cur_pos, dest_pos, cur_orient)
            if len(path) == 0:
                exclude.append(dest_pos)
                count -= 1

            mmr_sampling.print_log(f"목표위치: {dest_pos}, A* PATH: {path}")

        if len(path) == 0:
            return []

        self.dest_pos = dest_pos
        self.path = path
        # 다음위치 반환
        return path

    # 위치값을 맵좌표로 변환
    def transfer2_xy(self, pose: Sequence[float, float]) -> tuple[int, int]:
        # PC 맵 좌표를 SLAM 맵 좌표로 변환
        _x = math.floor(5.0 + pose[0])  # x
        _y = math.floor(5.0 + pose[1])  # y

        result = (_x if _x > 0 else 0, _y if _y > 0 else 0)
        self.node.print_log(f"<<_transfer2_xy>> {pose} => {result}")
        return result

    def is_intrap(self, pos: tuple, orient: Orient):
        x, y = pos
        _trap_map = {
            orient.Y: self.grid_map[x][y - 1],
            orient._Y: self.grid_map[x][y + 1],
            orient.X: self.grid_map[x - 1][y],
            orient._X: self.grid_map[x + 1][y],
        }

        return all(v > 0 for k, v in _trap_map.pop(orient).items())

    # a* method
    def _astar_method(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        init_orient: Orient = Orient.X,
    ) -> list[Sequence[int]]:
        """
        A* 알고리즘을 사용하여 최단 경로를 찾습니다.
        :param grid: 2D 그리드 맵
        :param start: 시작 지점 (x, y)
        :param goal: 목표 지점 (x, y)
        :return: 최단 경로
        """

        # 목표지점까지 추정거리
        def _heuristic_distance(self, start_pos, end_pos) -> int:
            """
            목표점까지의 추정거리를 추정한다
            """
            (x1, y1) = start_pos
            (x2, y2) = end_pos
            return abs(x1 - x2) + abs(y1 - y2)

        # 방향 제한조건
        def _check_if_backpath(self, cur_dir: Orient, next_dir: Orient) -> bool:
            # -xx or x-x 패턴
            pattern = re.compile(r"^-(.)\1$|^(.)-\2$")
            # 후진 경로 배제
            return pattern.match(f"{cur_dir.value}{next_dir.value}") is not None

        self.node.print_log(
            f"cur_pos: {start}, goal: {goal}, map:{len(self.grid_map.data)}"
        )
        grid_map = self.grid_map

        if not grid_map:
            return []

        frontier = deque()
        visited = set()  # 방문한 노드 집합

        start = (start[0], start[1], init_orient)
        frontier.append((start, [start]))  # 큐에 시작 위치와 경로 추가

        while frontier:
            curr_node, path = frontier.popleft()

            if curr_node[:2] == goal:
                # 초기입력된 방향값 제거
                path[0] = path[0][:2]
                return path

            if curr_node in visited:
                continue

            visited.add(curr_node)

            x, y, cur_d = curr_node
            for next_x, next_y, next_d in (
                (x + 1, y, Orient.X),
                (x - 1, y, Orient._X),
                (x, y + 1, Orient.Y),
                (x, y - 1, Orient._Y),
            ):
                if (
                    0 <= next_x < len(grid_map[1])
                    and 0 <= next_y < len(grid_map[0])
                    and grid_map[next_x][next_y] != 1
                    # 최초 다음위치에서 self.dir의 반대방향을 제외시킨다.
                    and not _check_if_backpath(cur_d, next_d)
                ):
                    if _check_if_backpath(cur_d, next_d):
                        continue

                    frontier.append(
                        ((next_x, next_y, next_d), path + [(next_x, next_y)])
                    )

            frontier = deque(
                sorted(
                    frontier,
                    key=lambda x: len(x[1]) + _heuristic_distance(x[0][:2], goal),
                )
            )

        return []

    # 경로이탈 여부
    def _check_pose_error(self, cur_pos):
        if any(x for x in self.paths if x == cur_pos):
            return False

        return True


class RobotController2:
    action_map = {     
        DirType.LEFT: (1.0, 1.35),
        DirType.RIGHT: (1.0, -1.35),
        DirType.FORWARD: (0.6, 0.0)
    }

    def __init__(self, node: Node):
        # 맵을 가진다.
        self.node = node
        self.pathMnger = PathManage(algorithm="a-star", dest_pos=(0, 0))
        self.orient: Orient = Orient.X
        self.dir_type = DirType.FORWARD
        IMUData.add_observer(o=self.get_data)

    def get_data(self, message: Message):
        self.imu_data = message.data

    def prepare(self):
        # 방향, 현재위치, 회전각도 등 취합
        self.cur_pos: tuple = self.pathMnger.transfer2_xy(self.get_data[:2])
        self.angular_data: float = self.get_data[2]
        # 목표경로를 정한다.
        self.next_pos = self.pathMnger.search_new_path(self.cur_pos, self.orient)

    def make_plan(self) -> Policy:
        """후진, 회전, 직진여부를 체크하고 해당 policy를 반환한다"""
        policy: EvHandle = Policy.check_paths(self.orient, self.cur_pos, self.path)
        self.dir_type, self.origin = policy.action
        return policy

    def excute(self, next_plan: EvHandle, **kwargs):
        next_plan.apply(**kwargs)
        
    
    @property
    def check_arrived(self, dest_pos: tuple):
        if self.pathMnger.cur_pos == dest_pos:
            self.notifyall(dest_pos)
            raise Exception("arrived")

    def notifyall(self, m: any):
        for o in self.observers:
            o.update(m)
