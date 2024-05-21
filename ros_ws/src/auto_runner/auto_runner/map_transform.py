import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import re

# 300x300 배열 생성


# 10x10 필터 배열 생성
filter_kernel = np.array(
    [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
)

_size = 10
grid_ratio = 0.1
grid_size = int(1 / grid_ratio)
map_size = int(_size / grid_ratio)


# 가우시안 블러 필터 생성
def gaussian_kernel(size, sigma=1):
    kernel = np.zeros((size, size), dtype=np.float32)
    center = size // 2

    for x in range(-center, center + 1):
        for y in range(-center, center + 1):
            kernel[x + center, y + center] = np.exp(
                -(x**2 + y**2) / (2 * sigma**2)
            ) / (2 * np.pi * sigma**2)

    kernel /= kernel.sum()
    return kernel


def pre_process(data) -> np.array:
    data[data == -1] = 95
    # data[data>70] = 1
    # data[data <= 40] = 0
    data = data.reshape(map_size, map_size)

    # 5x5 필터 커널 생성
    filter_kernel = np.array(
        [
            [0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0],
            [1, 1, 1, 1, 0],
            [0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0],
        ],
        dtype=np.uint8,
    )
    filter = gaussian_kernel(5, 1)
    # 패딩처리
    data = np.pad(data, 3 // 2, mode="constant", constant_values=0)
    data = data[2:, 2:]

    # 필터 적용
    # for row in range(2, data.shape[0] - 2):
    #     for col in range(2, data.shape[1] - 2):
    #         sub_data = data[row - 2:row + 3, col - 2:col + 3]
    #         if np.bitwise_and(sub_data.astype(np.int8), filter_kernel.astype(np.int8)).any():
    #             data[row, col] = 90
    #         else:
    #             data[row, col] = 0

    # data = filled_data
    # data[0,:] =100
    # data[99,:]=100
    # data[:,0]=100
    # data[:,99]=100

    # 300x300 배열 생성 (0과 1로 이루어짐)
    # data = np.random.randint(2, size=(300, 300))
    return data


def create_blockmap(data, _grid_size: int, map_size: tuple) -> np.array:

    # data = data.reshape(*map_size, _grid_size, _grid_size)

    result = np.zeros(map_size)

    for i in range(result.shape[0]):
        for j in range(result.shape[1]):
            start_row = i * _grid_size
            end_row = start_row + _grid_size
            start_col = j * _grid_size
            end_col = start_col + _grid_size
            _sub = data[start_row:end_row, start_col:end_col]
            if _sub.mean() > 40:
                result[i, j] = 1
            else:
                result[i, j] = 0

    # 각 행에서 30개의 10x10 서브 배열 추출
    # reshaped = np.reshape(data, (300, 30, 10))

    # 축 전환 및 최종 형상 변환
    # result = np.transpose(reshaped, (1, 0, 2)).reshape(30, 30, 10, 10)

    # for i in range(0, _size):
    #     for j in range(0, _size):
    #         if result[i,j].sum() > 1:
    #             t[i,j] =1

    # for r, d in enumerate(data):
    #     for c, e in enumerate(d):
    #         if e.sum() > 1:
    #             t[r,c] =1

    return result


def rotate_map_cw(data):
    # 2x2 시계 방향 90도 회전 행렬
    # rotation_cw_90_2x2 = np.array([[0, 1],
    # [-1, 0]])
    # return np.dot(data, rotation_cw_90_2x2)
    return np.rot90(data, k=1, axes=(0, 1))


def plot_map(data, paths, map_size=100):
    # 경로 추가
    if paths:
        for x, y in paths:
            data[x][y] = 0.7

    # 그래프 설정
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title(f"{map_size}x{map_size} Grid")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    # 그리드 출력
    ax.imshow(data, cmap="binary", origin="lower")
    # ax.imshow(data, cmap="gist_heat", origin="lower")
    # ax.imshow(data, cmap="gnuplot", origin="lower") #black/white/yellow

     # Draw every grid of the map:
    # for x in range(map_size):
    #     # print("Remaining Rows= " + str(mapWidth / gridWidth - x))
    #     for y in range(map_size):
    #         if data[x][y] == 0:  # Green unkown state
    #             ax.plot(x, y, "g.")
    #         elif data[x][y] > 0:  # Black occupied state
    #             ax.plot(x, y, "k.")
    #         else:  # Red free state
    #             ax.plot(x, y, "r.")

    # 축 범위 설정
    ax.set_xticks(np.arange(0, map_size, map_size // 5))
    ax.set_yticks(np.arange(0, map_size, map_size // 5))
    ax.set_xticklabels(np.arange(0, map_size, map_size // 5))
    ax.set_yticklabels(np.arange(0, map_size, map_size // 5))

    # 그리드 라인 추가
    # ax.set_xticks(np.arange(0, _size, _size // 6), minor=True)
    # ax.set_yticks(np.arange(0, _size, _size // 6), minor=True)
    ax.grid(which="minor", color="gray", linestyle="-", linewidth=1)

    # 그래프 출력
    plt.show()


# a* search
def find_path(
    grid: list, start: tuple[int, int], goal: tuple[int, int], dir: str = "-x"
) -> list[tuple[int, int]]:
    """
    A* 알고리즘을 사용하여 최단 경로를 찾습니다.
    :param grid: 2D 그리드 맵
    :param start: 시작 지점 (x, y)
    :param goal: 목표 지점 (x, y)
    :return: 최단 경로
    """
    start = (start[0], start[1], dir)

    frontier = deque()
    frontier.append((start, [start]))  # 큐에 시작 위치와 경로 추가
    visited = set()  # 방문한 노드 집합

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
            (x + 1, y, "x"),
            (x - 1, y, "-x"),
            (x, y + 1, "y"),
            (x, y - 1, "-y"),
        ):
            if (
                0 <= next_x < len(grid[1])
                and 0 <= next_y < len(grid[0])
                and grid[next_x][next_y] != 1
            ):
                # 최초 다음위치에서 self.dir의 반대방향을 제외시킨다.
                if _check_if_backpath(cur_d, next_d):
                    continue

                frontier.append(((next_x, next_y, next_d), path + [(next_x, next_y)]))

        frontier = deque(
            sorted(
                frontier,
                key=lambda x: len(x[1]) + _heuristic_distance(x[0][:2], goal),
            )
        )

    # return []
    path[0] = path[0][:2]
    return path


def _check_if_backpath(cur_dir: str, next_dir: str) -> bool:

    pattern = re.compile(r"^-(.)\1$|^(.)-\2$")
    # 후진 경로 배제
    return pattern.match(f"{cur_dir}{next_dir}") is not None


def _heuristic_distance(a, b) -> int:
    """
    목표점까지의 추정거리를 추정한다
    """
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def occ_gridmap(data:np=None) -> list[int]:
    data = pre_process(data)
    data = create_blockmap(data, map_size=(10, 10), _grid_size=10)
    data = rotate_map_cw(data)
    return data.tolist()


if __name__ == "__main__":
    # return []
    if 1:
        base_dir = (
            r"D:\unity_works\with-robot-2024-1st\ros_ws\install\gmapping\share\gmapping"
        )
    else:
        base_dir = (
            r"D:\unity_works\with-robot-2024-1st\ros_ws\src\slam_map"
        )
    data = np.loadtxt(f"{base_dir}/resource/occ_map.txt")
    
    data = ' '.join(map(str, data.flatten()))
    raw_data = np.fromstring(data, dtype=float, sep=' ')

    map = occ_gridmap(raw_data)
    print(map)

    paths = find_path(data, (1,1), (5,9))
    print(paths)
    plot_map(map, None, map_size=10)
