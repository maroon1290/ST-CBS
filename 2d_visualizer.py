import numpy as np
import matplotlib.pyplot as plt
import yaml
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
from itertools import combinations
import math

# 1초에 최대 5m까지 이동가능
# 천개로 나누면

def euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def interpolate(x1, x2, y1, y2, time_step):
    dx = x2 - x1
    dy = y2 - y1
    d = math.hypot(dx, dy)
    theta = math.atan2(dy, dx)
    if d < time_step:
        return x2, y2
    else:
        return x1 + time_step * math.cos(theta), y1 + time_step * math.sin(theta)


if __name__ == '__main__':
    basename = "CluttedEnvironment_15_2"
    # read paths from yaml
    with open(f'solutions/{basename}_solutions.yaml', 'r') as f:
        paths = yaml.load(f, Loader=yaml.FullLoader)

    with open(f'configs/{basename}.yaml', 'r') as f:
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

    config["robot_radii"] = [radius - 0.05 for radius in config["robot_radii"]]
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
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(0, config["width"])
    ax.set_ylim(0, config["height"])

    # 장애물 시각화 객체 생성
    for x, y, r in circle_obstacles:
        obstacle = Circle((x, y), r, fc='gray', alpha=0.5)
        ax.add_patch(obstacle)

    for x, y, width, height in rectangle_obstacles:
        obstacle = Rectangle((x - width / 2, y - height / 2), width, height, fc='gray', alpha=0.5)
        ax.add_patch(obstacle)

    # 로봇 시각화 객체 생성
    robots = []
    color_list = [
        'b',  # blue
        'g',  # green
        'r',  # red
        'c',  # cyan
        'm',  # magenta
        'y',  # yellow
        'k',  # black
        '#FF6347',  # tomato
        '#8B0000',  # darkred
        '#FFD700',  # gold
        '#ADFF2F',  # greenyellow
        '#483D8B',  # darkslateblue
        '#FF4500',  # orangered
        '#7CFC00',  # lawngreen
        '#D8BFD8',  # thistle
        '#D2B48C',  # tan
        '#FA8072',  # salmon
        '#F0E68C',  # khaki
        '#90EE90',  # lightgreen
        '#87CEEB'   # skyblue
    ]
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

    # 로봇 번호 텍스트
    robot_num = len(x_list)
    robot_text = []
    for i in range(robot_num):
        robot_text.append(ax.text(0.05, 0.9 - i * 0.05, f'Robot {i + 1}', transform=ax.transAxes, fontsize=12, color=color_list[i]))

    def check_collision(robot_pos, obstacles, robot_radius):
        for obs_x, obs_y, obs_r in obstacles:
            distance = euclidean_distance(robot_pos, (obs_x, obs_y))
            if distance <= obs_r + robot_radius:
                return True
        return False
    x_poses = [x_list[i][0] for i in range(robot_num)]
    y_poses = [y_list[i][0] for i in range(robot_num)]
    time_step = 5 / 100
    def update(frame):
        current_time = frame
        # 시작 위치와 도착 위치 표시
        for i, (start_point, end_point) in enumerate(zip(start_points, end_points)):
            start_point.set_center((x_list[i][0], y_list[i][0]))
            end_point.set_xy((x_list[i][-1] - 0.2, y_list[i][-1] - 0.2))
        # 각 로봇의 위치 갱신
        for i, (robot, path_line, x, y, t, text) in enumerate(zip(robots, path_lines, x_list, y_list, t_list, robot_text)):
            current_position = np.searchsorted(t, current_time, side='right') - 1
            if current_position >= 0:
                if current_position < len(t) - 1:
                    x_poses[i], y_poses[i] = interpolate(x_poses[i], x[current_position + 1],
                                               y_poses[i], y[current_position + 1],
                                               time_step)
                else:
                    x_poses[i], y_poses[i] = x[-1], y[-1]

                # 로봇 번호 텍스트 위치 변경
                text.set_position((x_poses[i] + 0.2, y_poses[i] - 0.2))

                robot.set_center((x_poses[i], y_poses[i]))
                path_line.set_data(x[:current_position + 1] + (x_poses[i],), y[:current_position + 1] + (y_poses[i],))

            # 로봇과 장애물 간 충돌 감지
            if check_collision((x_poses[i], y_poses[i]), circle_obstacles, config['robot_radii'][i]):
                print(f"Robot {i} collided with an obstacle!")

        # 각 로봇들간의 충돌 감지
        for i, j in combinations(range(len(robots)), 2):
            x1, y1 = robots[i].center
            x2, y2 = robots[j].center
            distance = euclidean_distance((x1, y1), (x2, y2))
            if distance < config['robot_radii'][i] + config['robot_radii'][j]:
                print(f"Robot {i} collided with robot {j}!")

        # 시간 텍스트 갱신
        time_text.set_text(f'Time: {current_time:.2f}')

        all_text = [time_text] + robot_text
        return robots + path_lines + all_text

    # 애니메이션 설정
    max_time = max(max(t) for t in t_list)
    ani = FuncAnimation(fig, update, frames=np.arange(0, max_time + 0.01, 0.01), blit=True, interval=10, repeat=False)

    plt.show()
