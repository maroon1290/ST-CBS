import numpy as np
import matplotlib.pyplot as plt
import yaml
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
from itertools import combinations


def euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


# 충돌 거리 기준 설정
collision_distance = 0.5


def interpolate(p1, p2, t1, t2, t):
    ratio = (t - t1) / (t2 - t1)
    return p1 + ratio * (p2 - p1)


if __name__ == '__main__':
    # read paths from yaml
    with open('solutions.yaml', 'r') as f:
        paths = yaml.load(f, Loader=yaml.FullLoader)

    with open('configs/deadlock_config.yaml', 'r') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    # 각 로봇의 x, y 좌표와 시간을 분리
    x_list = []
    y_list = []
    t_list = []

    for path in paths:
        x, y, t = zip(*path)
        x_list.append(x)
        y_list.append(y)
        t_list.append(t)

    # 원형 장애물 리스트 (x, y, r)
    circle_obstacles = []
    rectangle_obstacles = []

    for config_obstacle in config['obstacles']:
        if config_obstacle['type'] == 'CircleObstacle':
            circle_obstacle = (config_obstacle['x'], config_obstacle['y'], config_obstacle['radius'])
            circle_obstacles.append(circle_obstacle)
        elif config_obstacle['type'] == 'RectangleObstacle':
            rectangle_obstacle = (config_obstacle['x'], config_obstacle['y'], config_obstacle['width'], config_obstacle['height'])
            rectangle_obstacles.append(rectangle_obstacle)
        else:
            raise ValueError('invalid obstacle type')

    # 시각화를 위한 설정
    fig, ax = plt.subplots()
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)

    # 장애물 시각화 객체 생성
    for x, y, r in circle_obstacles:
        obstacle = Circle((x, y), r, fc='gray', alpha=0.5)
        ax.add_patch(obstacle)

    for x, y, width, height in rectangle_obstacles:
        obstacle = Rectangle((x - width / 2, y - height / 2), width, height, fc='gray', alpha=0.5)
        ax.add_patch(obstacle)

    # 로봇 시각화 객체 생성
    robots = []
    color_list = ['red', 'green', 'blue', 'yellow', 'purple', 'orange', 'pink', 'brown', 'gray', 'olive']
    for i, (x, y) in enumerate(zip(x_list, y_list)):
        robot = Circle((x[0], y[0]), config['robot_radii'][i], fc=color_list[i], alpha=0.5)
        ax.add_patch(robot)
        robots.append(robot)

    # 시작, 도착 위치 시각화 객체 생성
    start_points = []
    end_points = []

    for i, (x, y) in enumerate(zip(x_list, y_list)):
        start_point = Circle((x[0], y[0]), 0.2, fc=color_list[i], alpha=0.5)
        end_point = Rectangle((x[-1] - 0.2, y[-1] - 0.2), 0.4, 0.4, fc=color_list[i], alpha=0.5)
        ax.add_patch(start_point)
        ax.add_patch(end_point)
        start_points.append(start_point)
        end_points.append(end_point)

    # 경로 시각화 객체 생성
    path_lines = []
    for i, (x, y) in enumerate(zip(x_list, y_list)):
        path_line, = ax.plot(x[0], y[0], color=color_list[i], linewidth=1)
        path_lines.append(path_line)

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
        # 시작 위치와 도착 위치 표시
        for i, (start_point, end_point) in enumerate(zip(start_points, end_points)):
            start_point.set_center((x_list[i][0], y_list[i][0]))
            end_point.set_xy((x_list[i][-1] - 0.2, y_list[i][-1] - 0.2))

        # 각 로봇의 위치 갱신
        for i, (robot, path_line, x, y, t) in enumerate(zip(robots, path_lines, x_list, y_list, t_list)):
            current_position = np.searchsorted(t, current_time, side='right') - 1
            if current_position >= 0:
                if current_position < len(t) - 1:
                    x_pos = interpolate(x[current_position], x[current_position + 1], t[current_position],
                                        t[current_position + 1], current_time)
                    y_pos = interpolate(y[current_position], y[current_position + 1], t[current_position],
                                        t[current_position + 1], current_time)
                else:
                    x_pos, y_pos = x[-1], y[-1]

                robot.set_center((x_pos, y_pos))
                path_line.set_data(x[:current_position + 1] + (x_pos,), y[:current_position + 1] + (y_pos,))

            # 로봇과 장애물 간 충돌 감지
            if check_collision((x_pos, y_pos), circle_obstacles, config['robot_radii'][i]):
                print(f"Robot {i} collided with an obstacle!")

        # 각 로봇들간의 충돌 감지
        for i, j in combinations(range(len(robots)), 2):
            x1, y1 = robots[i].center
            x2, y2 = robots[j].center
            distance = euclidean_distance((x1, y1), (x2, y2))
            if distance <= config['robot_radii'][i] + config['robot_radii'][j]:
                print(f"Robot {i} collided with robot {j}!")

        # 시간 텍스트 갱신
        time_text.set_text(f'Time: {current_time:.2f}')

        return robots + path_lines + [time_text]

    # 애니메이션 설정
    max_time = max(max(t) for t in t_list)
    ani = FuncAnimation(fig, update, frames=np.arange(0, max_time + 0.01, 0.01), blit=True, interval=10, repeat=False)

    plt.show()
