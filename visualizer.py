import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle


def euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


# 충돌 거리 기준 설정
collision_distance = 0.5


def interpolate(p1, p2, t1, t2, t):
    ratio = (t - t1) / (t2 - t1)
    return p1 + ratio * (p2 - p1)


if __name__ == '__main__':
    # path = [x, y, t]
    path1 = [[4, 10, 0],
    [6.801263740516886, 11.073741801396102, 1],
    [8.820271928034995, 13.292661795162175, 2],
    [8.957819769785576, 13.535097125398801, 3],
    [9.631382740223291, 13.320969777931397, 4],
    [10.36726191262556, 13.344661073795866, 5],
    [10.942425739765387, 12.339158187852611, 6],
    [13.379247278185073, 10.589329399115291, 7],
    [16.0, 10.0, 8],]
    path2 = [[16, 10, 0],
[13.00005282618266, 9.982196789464961, 1],
[10.000387569080523, 9.937382223369013, 2],
[7.001578307638837, 10.021898573808267, 3],
[4.002831677529189, 10.108608713965154, 4],
[4.0, 10.0, 5],]
    # 각 로봇의 x, y 좌표와 시간을 분리
    x1, y1, t1 = zip(*path1)
    x2, y2, t2 = zip(*path2)

    # 원형 장애물 리스트 (x, y, r)
    obstacles = [
        (4.0, 4.0, 4),
        (10.0, 6.0, 2),
        (16.0, 4.0, 4),
        (4.0, 16.0, 4),
        (10.0, 18.0, 2),
        (16.0, 16.0, 4),
    ]

    # 시각화를 위한 설정
    fig, ax = plt.subplots()
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)

    # 장애물 시각화 객체 생성
    for x, y, r in obstacles:
        obstacle = Circle((x, y), r, fc='gray', alpha=0.5)
        ax.add_patch(obstacle)

    # 로봇 반지름 설정
    robot_radius = 1.5

    # 로봇 시각화 객체 생성
    robot1 = Circle((x1[0], y1[0]), robot_radius, fc='blue')
    robot2 = Circle((x2[0], y2[0]), robot_radius, fc='red')
    ax.add_patch(robot1)
    ax.add_patch(robot2)

    # 경로 시각화 객체 생성
    path_line1, = plt.plot([], [], 'b-', linewidth=1)
    path_line2, = plt.plot([], [], 'm-', linewidth=1)

    # 시간 텍스트 객체 생성
    time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=12)

    def check_collision(robot_pos, obstacles, robot_radius):
        for obs_x, obs_y, obs_r in obstacles:
            distance = euclidean_distance(robot_pos, (obs_x, obs_y))
            if distance <= obs_r + robot_radius:
                return True
        return False

    def update(frame):
        current_time = frame

        # 로봇 1
        current_position1 = np.searchsorted(t1, current_time, side='right') - 1
        if current_position1 >= 0:
            if current_position1 < len(t1) - 1:
                x_pos1 = interpolate(x1[current_position1], x1[current_position1 + 1], t1[current_position1],
                                     t1[current_position1 + 1], current_time)
                y_pos1 = interpolate(y1[current_position1], y1[current_position1 + 1], t1[current_position1],
                                     t1[current_position1 + 1], current_time)
            else:
                x_pos1, y_pos1 = x1[-1], y1[-1]

            robot1.set_center((x_pos1, y_pos1))
            path_line1.set_data(x1[:current_position1 + 1] + (x_pos1,), y1[:current_position1 + 1] + (y_pos1,))

        # 로봇 2
        current_position2 = np.searchsorted(t2, current_time, side='right') - 1
        if current_position2 >= 0:
            if current_position2 < len(t2) - 1:
                x_pos2 = interpolate(x2[current_position2], x2[current_position2 + 1], t2[current_position2],
                                     t2[current_position2 + 1], current_time)
                y_pos2 = interpolate(y2[current_position2], y2[current_position2 + 1], t2[current_position2],
                                     t2[current_position2 + 1], current_time)
            else:
                x_pos2, y_pos2 = x2[-1], y2[-1]

            robot2.set_center((x_pos2, y_pos2))
            path_line2.set_data(x2[:current_position2 + 1] + (x_pos2,), y2[:current_position2 + 1] + (y_pos2,))

        # 로봇과 원형 장애물 간 충돌 감지
        if check_collision((x_pos1, y_pos1), obstacles, robot_radius):
            print("Robot 1 collided with an obstacle!")
        if check_collision((x_pos2, y_pos2), obstacles, robot_radius):
            print("Robot 2 collided with an obstacle!")

        # 충돌 감지
        distance = euclidean_distance((x_pos1, y_pos1), (x_pos2, y_pos2))
        if distance <= collision_distance:
            print("Collision!")

        # 시간 텍스트 갱신
        time_text.set_text(f'Time: {current_time:.2f}')

        return robot1, path_line1, robot2, path_line2, time_text

    # 애니메이션 설정
    max_time = max(max(t1), max(t2))
    ani = FuncAnimation(fig, update, frames=np.arange(0, max_time + 0.01, 0.1), blit=True, interval=100, repeat=False)

    plt.show()
