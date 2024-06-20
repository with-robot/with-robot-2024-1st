import numpy as np
import random
import math
from auto_runner.lib.common import Message, Observable

def print_log(log:str):
        Observable.notifyall('node', Message(data_type='log', data=log))

def euclidean_distance(a, b):
    return np.linalg.norm(a - b)


def find_farthest_coordinate(map: list, base: tuple, exclude:list[tuple]) -> tuple:
    
    # 맵에서 빈셀들을 선택
    arr = np.array(map)
    cordinates = []
    for i in range(arr.shape[0]):
        for j in range(arr.shape[1]):
            if (i, j) in exclude:
                continue
            if arr[i, j] == 0:
                cordinates.append((i, j))
    
    # log(f"cordinates:{cordinates}, exclude:{exclude}")
    # while True:
    #     _choice = random.choice(cordinates)
    #     if _choice != base:
    #         x, y = _choice
    #         break
    
    # 입력 좌표와 다른 좌표들 간의 거리를 계산
    x, y = base
    distances = []
    for cord in cordinates:
        distance = math.sqrt((cord[0] - x) ** 2 + (cord[1] - y) ** 2)
        distances.append((cord, distance))

    if len(distances) == 0:
        return []
    
    # 거리가 가장 먼 좌표 찾기
    top_k = 3
    if len(distances) >= top_k:        
        farthest_top_k_distances = sorted(distances, key=lambda x: x[1], reverse=True)[:top_k]
        farthest_coordinate = random.choice(farthest_top_k_distances)
    else:
        farthest_coordinate = max(distances, key=lambda x: x[1])

    return farthest_coordinate[0]


def mmr_sampling(array, initial_sample, num_samples):
    flattened_array = array.flatten()
    num_elements = flattened_array.size

    # 첫 번째 샘플의 인덱스를 찾기
    # sampled_indices = [np.where(flattened_array == initial_sample)[0][0]]
    sampled_indices = [initial_sample]

    # MMR 방식으로 나머지 샘플 선택
    for _ in range(1, num_samples):
        best_candidate = -1
        max_margin = -np.inf

        for i in range(num_elements):
            if i in sampled_indices:
                continue

            min_distance_to_sampled = min(
                euclidean_distance(flattened_array[i], flattened_array[j])
                for j in sampled_indices
            )
            margin = min_distance_to_sampled

            if margin > max_margin:
                max_margin = margin
                best_candidate = i

        sampled_indices.append(best_candidate)

    return flattened_array[sampled_indices]