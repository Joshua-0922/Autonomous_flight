import random
import heapq
import matplotlib.pyplot as plt
import numpy as np
import itertools

# 3차원 미로 생성 함수
def generate_3d_maze(n, obstacle_prob):
    maze = [[[random.random() > obstacle_prob for _ in range(n)] for _ in range(n)] for _ in range(n)]
    maze[0][0][0] = True  # 시작 지점은 이동 가능
    return maze

# 휴리스틱 함수: 유클리드 거리 사용
def heuristic(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2)**0.5

# 장애물 근처 안전 거리 체크 함수
def is_safe(maze, x, y, z, safety_distance):
    n = len(maze)
    for dx in np.arange(-safety_distance, safety_distance + 1, 0.5):
        for dy in np.arange(-safety_distance, safety_distance + 1, 0.5):
            for dz in np.arange(-safety_distance, safety_distance + 1, 0.5):
                nx, ny, nz = int(round(x + dx)), int(round(y + dy)), int(round(z + dz))
                if 0 <= nx < n and 0 <= ny < n and 0 <= nz < n:
                    if not maze[nx][ny][nz]:
                        return False
    return True

# A* 알고리즘 구현
def a_star_3d(maze, start, goal, safety_distance):
    n = len(maze)
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    directions = list(itertools.product([-1, 0, 1], repeat=3))
    directions.remove((0, 0, 0))  # 자기 자신 제외

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        neighbors = [
            (current[0] + dx, current[1] + dy, current[2] + dz)
            for dx, dy, dz in directions
        ]

        for neighbor in neighbors:
            x, y, z = neighbor
            if 0 <= x < n and 0 <= y < n and 0 <= z < n and is_safe(maze, x, y, z, safety_distance):
                tentative_g_score = g_score[current] + heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None  # 경로가 없는 경우

# 3차원 미로 시각화 함수
def plot_3d_maze(maze, path=None):
    n = len(maze)
    maze_array = np.array(maze)
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 미로 시각화 (장애물과 경로 구분)
    for x in range(n):
        for y in range(n):
            for z in range(n):
                if not maze[x][y][z]:
                    ax.scatter(x, y, z, color='black', s=20)  # 장애물

    if path:
        path_x, path_y, path_z = zip(*path)
        ax.plot(path_x, path_y, path_z, color='red', linewidth=2, marker='o')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(0, n-1)
    ax.set_ylim(0, n-1)
    ax.set_zlim(0, n-1)
    plt.show()

# 메인 실행 부분
if __name__ == "__main__":
    n = int(input("미로 크기 (n x n x n): "))
    obstacle_prob = float(input("장애물 비율 (0과 1 사이): "))
    safety_distance = float(input("안전 거리 (float 값 허용): "))

    start = tuple(map(int, input("시작 지점 (x y z): ").split()))
    goal = tuple(map(int, input("도착 지점 (x y z): ").split()))

    maze = generate_3d_maze(n, obstacle_prob)
    path = a_star_3d(maze, start, goal, safety_distance)

    if path:
        print("경로를 찾았습니다:")
        print(path)
        plot_3d_maze(maze, path)
    else:
        print("경로를 찾을 수 없습니다.")
        plot_3d_maze(maze)
