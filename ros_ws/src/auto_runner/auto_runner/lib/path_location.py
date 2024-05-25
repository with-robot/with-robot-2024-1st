from collections import deque
from auto_runner.lib.common import Dir, MessageHandler, TypeVar, Sequence
from auto_runner import mmr_sampling
import re, math

LoggableNode = TypeVar("LoggableNode", bound=MessageHandler)


class PathFinder:
    """맵과 위치정보에 대한 책임을 갖는다"""

    grid_map: list[Sequence[int]]

    def __init__(
        self,
        node: LoggableNode,
        algorithm: str = "a-star",
        dest_pos: tuple = (-1, -1),
    ) -> None:
        self.node = node
        self.is_found = False
        self.grid_map = []
        self.paths = []
        self.algorithm = algorithm
        self.cur_pos = None
        self.dest_pos = dest_pos

    def find_path(self, **kwargs):
        if self.algorithm == "a-star":
            return self._astar_method(**kwargs)
        else:
            return []

    def update_map(self, map: list[Sequence[int]]) -> None:
        self.grid_map = map

    # 도착위치이면 새로운 도착위치를 반환한다.
    def check_arrival(self, cur_dir):

        if self.dest_pos != self.cur_pos:
            return

        self.dest_pos = None
        next_pos = self.check_and_dest(self.cur_pos, cur_dir)
        self.node._logging(f"<<arrived>> pos:{self.cur_pos}, next:{next_pos}")

    # 다음위치 계산
    def check_and_dest(self, cur_dir) -> tuple[int, int]:
        self.node._logging(f"<<check_and_dest>> pos:{self.cur_pos}, dir:{cur_dir}")
        
        if self.dest_pos and len(self.paths) > 1 and self.paths[1]==self.cur_pos:
            # 정상 이동 시, 경로를 재검색하지 않는다.
            # 완료된 경로 제거
            self.paths = self.paths[1:]
            return self.paths[1]
    
        paths = []
        dest_pos = (0,0)
        _fail_dest_pos = None
        while len(paths) == 0:
            if _fail_dest_pos ==dest_pos:
                return []
            dest_pos = mmr_sampling.find_farthest_coordinate(
                self.grid_map, self.cur_pos
            )
            paths = self._astar_method(self.cur_pos, dest_pos, cur_dir)
            if len(paths) == 0:
                _fail_dest_pos = dest_pos

            self.node._logging(f"목표위치: {dest_pos}, A* PATH: {paths}")
        
        self.dest_pos = dest_pos
        self.paths = paths
        # 다음위치 반환
        return paths[1]

    def set_cur_pos(self, pose: tuple[float, float]):
        self.cur_pos = self._transfer2_xy(pose)

    # 위치값을 맵좌표로 변환    
    def _transfer2_xy(self, pose: tuple[float, float]) -> tuple[int, int]:
        # PC 맵 좌표를 SLAM 맵 좌표로 변환
        _x = math.floor(5.0 - pose[0])  # x
        _y = math.floor(5.0 - pose[1])  # y

        result = (_x if _x > 0 else 0, _y if _y > 0 else 0)
        return result

    # a* method
    def _astar_method(
        self, start: tuple[int, int], goal: tuple[int, int], init_dir: Dir = Dir.X
    ) -> list[Sequence[int]]:
        """
        A* 알고리즘을 사용하여 최단 경로를 찾습니다.
        :param grid: 2D 그리드 맵
        :param start: 시작 지점 (x, y)
        :param goal: 목표 지점 (x, y)
        :return: 최단 경로
        """
        self.node._logging(
            f"find_path: {start}, goal: {goal}, map:{len(self.grid_map)}"
        )
        self.is_found = False

        if not self.grid_map:
            return []

        frontier = deque()
        visited = set()  # 방문한 노드 집합

        start = (start[0], start[1], init_dir)
        frontier.append((start, [start]))  # 큐에 시작 위치와 경로 추가

        while frontier:
            curr_node, path = frontier.popleft()

            if curr_node[:2] == goal:
                self.is_found = True
                # 초기입력된 방향값 제거
                path[0] = path[0][:2]
                return path

            if curr_node in visited:
                continue

            visited.add(curr_node)

            x, y, cur_d = curr_node
            for next_x, next_y, next_d in (
                (x + 1, y, Dir.X),
                (x - 1, y, Dir._X),
                (x, y + 1, Dir.Y),
                (x, y - 1, Dir._Y),
            ):
                if (
                    0 <= next_x < len(self.grid_map[1])
                    and 0 <= next_y < len(self.grid_map[0])
                    and self.grid_map[next_x][next_y] != 1
                    # 최초 다음위치에서 self.dir의 반대방향을 제외시킨다.
                    and not self._check_if_backpath(cur_d, next_d) 
                ):
                    # if self._check_if_backpath(cur_d, next_d):
                    #     continue

                    frontier.append(
                        ((next_x, next_y, next_d), path + [(next_x, next_y)])
                    )

            frontier = deque(
                sorted(
                    frontier,
                    key=lambda x: len(x[1]) + self._heuristic_distance(x[0][:2], goal),
                )
            )

        return []

    # 방향 제한조건
    def _check_if_backpath(self, cur_dir: str, next_dir: str) -> bool:
        # -xx or x-x 패턴
        pattern = re.compile(r"^-(.)\1$|^(.)-\2$")
        # 후진 경로 배제
        return pattern.match(f"{cur_dir}{next_dir}") is not None

    # 목표지점까지 추정거리
    def _heuristic_distance(self, start_pos, end_pos) -> int:
        """
        목표점까지의 추정거리를 추정한다
        """
        (x1, y1) = start_pos
        (x2, y2) = end_pos
        return abs(x1 - x2) + abs(y1 - y2)

    # 경로이탈 여부
    def _check_pose_error(self, new_cor):
        if a_Y(x for x in self.grid_map if x == new_cor):
            return False

        return True
