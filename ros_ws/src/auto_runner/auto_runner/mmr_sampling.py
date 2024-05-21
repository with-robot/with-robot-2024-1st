import numpy as np
import random
import math

def euclidean_distance(a, b):
    return np.linalg.norm(a - b)


def find_farthest_coordinate(map:list, cord:tuple):
    x, y = cord
    # 임의로 10개의 좌표 생성
    arr = np.array(map)
    coordinates = []
    for i in range(arr.shape[0]):
        for j in range(arr.shape[1]):
            if arr[i, j] == 0:
                coordinates.append((i, j))           
    
    # 입력 좌표와 다른 좌표들 간의 거리를 계산
    distances = []
    for coord in coordinates:
        distance = math.sqrt((coord[0] - x)**2 + (coord[1] - y)**2)
        distances.append((coord, distance))
    
    # 거리가 가장 먼 좌표 찾기
    farthest_coordinate = max(distances, key=lambda x: x[1])
    
    return farthest_coordinate[0] if distance else cord

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
            
            min_distance_to_sampled = min(euclidean_distance(flattened_array[i], flattened_array[j]) for j in sampled_indices)
            margin = min_distance_to_sampled

            if margin > max_margin:
                max_margin = margin
                best_candidate = i
        
        sampled_indices.append(best_candidate)
    
    return flattened_array[sampled_indices]

if __name__=='__main__':
    # # 10x10 배열 생성
    # array = np.random.random((10, 10))

    # # 첫 번째 샘플 값 입력
    # initial_sample = int(input("첫 번째 샘플 값을 입력하세요: "))

    # # MMR 방식으로 4개의 샘플 추가로 추출
    # samples = mmr_sampling(array, initial_sample, 5)

    # # 샘플 출력
    # print("샘플된 값:", samples)

    print(find_farthest_coordinate(0, 0))