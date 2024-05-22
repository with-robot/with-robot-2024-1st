from collections import deque
from typing import Sequence
from enum import Enum
import re

class Dir(Enum):
    UP = '^'
    DOWN = 'v'
    LEFT = '<'
    RIGHT = '>'

class AStarPathFinder:
    is_found:bool

    def __init__(self, algorithm:str='a-star', logger:object=None) -> None:
        self.algorithm= algorithm
        self.is_found = False
        self.logger = logger
        self.dir = Dir.DOWN
    
    def set_map(self, map:list[Sequence[int]]) -> None:
        self.grid_map = map

    def logging(self, message) -> None:
        if self.logger:
            self.logger.info(message)
    
    def change_dir(self, dir:Dir) -> None:
        self.dir = dir

    # a* search
    def find_path(
        self, start: tuple[int, int], goal: tuple[int, int]
    ) -> list[Sequence[int]]:
        """
        A* 알고리즘을 사용하여 최단 경로를 찾습니다.
        :param grid: 2D 그리드 맵
        :param start: 시작 지점 (x, y)
        :param goal: 목표 지점 (x, y)
        :return: 최단 경로
        """
        self.logging(f"find_path: {start}, goal: {goal}, map:{len(self.grid_map)}")

        if not self.grid_map:
            return False, list(start)
        
        frontier = deque()
        visited = set()  # 방문한 노드 집합

        start = (start[0], start[1], self.dir)
        frontier.append((start, [start]))  # 큐에 시작 위치와 경로 추가

        while frontier:
            curr_node, path = frontier.popleft()

            if curr_node[:2] == goal:
                # 초기입력된 방향값 제거
                path[0] = path[0][:2]
                return True, path

            if curr_node in visited:
                continue

            visited.add(curr_node)

            x, y, cur_d = curr_node
            for next_x, next_y, next_d in (
                (x + 1, y, Dir.UP),
                (x - 1, y, Dir.Down),
                (x, y + 1, Dir.RIGHT),
                (x, y - 1, Dir.LEFT),
            ):
                if (
                    0 <= next_x < len(self.grid_map[1])
                    and 0 <= next_y < len(self.grid_map[0])
                    and self.grid_map[next_x][next_y] != 1
                ):
                    # 최초 다음위치에서 self.dir의 반대방향을 제외시킨다.
                    if self._check_if_backpath(cur_d, next_d):
                        continue

                    frontier.append(
                        ((next_x, next_y, next_d), path + [(next_x, next_y)])
                    )

            frontier = deque(
                sorted(
                    frontier,
                    key=lambda x: len(x[1]) + self._heuristic_distance(x[0][:2], goal),
                )
            )
        # 초기입력된 방향값 제거
        path[0] = path[0][:2]
        return False, path
    
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
