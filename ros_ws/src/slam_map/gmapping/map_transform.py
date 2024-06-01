import numpy as np
import matplotlib.pyplot as plt
import os

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
            kernel[x + center, y + center] = np.exp(-(x**2 + y**2) / (2 * sigma**2)) / (2 * np.pi * sigma**2)

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
    filter = gaussian_kernel(5,1)
    # 패딩처리
    # data = np.pad(data, 3 // 2, mode="constant", constant_values=0)
    # data = data[2:,2:]

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


def create_blockmap(data, _grid_size:int, map_size:tuple) -> np.array:

    # data = data.reshape(*map_size, _grid_size, _grid_size)
    
    result = np.zeros(map_size)
    
    for i in range(result.shape[0]):
        for j in range(result.shape[1]):
            start_row = i * _grid_size
            end_row = start_row + _grid_size
            start_col = j * _grid_size
            end_col = start_col + _grid_size
            _sub = data[start_row:end_row, start_col:end_col]
            if _sub.mean() > 35:
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
    # return np.rot90(data, k=1, axes=(0,1))
    return np.array(data).T

def plot_map(data, map_size=100):
    # 그래프 설정
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title(f"{map_size}x{map_size} Grid")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    # 그리드 출력
    ax.imshow(data, cmap="binary", origin="lower")

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


def main():
    base_dir = (
        r"D:\unity_works\with-robot-2024-1st\ros_ws\install\gmapping\share\gmapping"
    )
    data = np.loadtxt(f"{base_dir}/resource/occ_map.txt")

    data = pre_process(data)
    data = create_blockmap(data, map_size=(10,10), _grid_size=10)
    data = rotate_map_cw(data)
    print(data)
    plot_map(data, map_size=10)


if __name__ == "__main__":
    main()
