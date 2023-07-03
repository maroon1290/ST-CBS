import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from matplotlib.patches import Rectangle, RegularPolygon
import yaml

basename = "NarrowEnv_2"
config_file = f"../configs/{basename}.yaml"
solution_file = f"../solutions/{basename}_solution.yaml"

# configs 파일
with open(config_file, 'r') as stream:
    config = yaml.load(stream, Loader=yaml.FullLoader)
# 로봇의 경로
with open(solution_file, 'r') as stream:
    solution = yaml.load(stream, Loader=yaml.FullLoader)


# 선형 보간을 위한 함수
def interpolate_path(path, num_points):
    new_path = []
    for i in range(len(path) - 1):
        for t in np.linspace(0, 1, num_points):
            x = (1 - t) * path[i][0] + t * path[i + 1][0]
            y = (1 - t) * path[i][1] + t * path[i + 1][1]
            new_path.append([x, y, t + path[i][2]])
    new_path.append(path[-1])
    return new_path


# 충돌 검사를 위한 함수
def check_collision_robots(robot, other_robots):
    # Check collision with other robots
    for other_robot in other_robots:
        if other_robot == robot:
            continue
        dist = np.hypot(robot.center[0] - other_robot.center[0], robot.center[1] - other_robot.center[1])
        if dist <= config["robotRadii"][0] * 2:  # Both robots' radii combined
            return True


def check_collision_obstacles(robot, obstacles):
    for obstacle in obstacles:
        # Check collision with rect obstacle
        width = obstacle.get_width()
        height = obstacle.get_height()
        center_x = obstacle.get_x() + width / 2
        center_y = obstacle.get_y() + height / 2

        x = robot.center[0]
        y = robot.center[1]
        r = robot.radius

        # Define the rectangle's bounds
        rect_left = center_x - width / 2
        rect_right = center_x + width / 2
        rect_top = center_y - height / 2
        rect_bottom = center_y + height / 2

        # If the circle's center is inside the rectangle, return True
        if rect_left < x < rect_right and rect_top < y < rect_bottom:
            return True

        # Calculate the closest point on the rectangle's edge
        closest_x = min(max(x, rect_left), rect_right)
        closest_y = min(max(y, rect_top), rect_bottom)

        # Calculate the distance from the circle's center to the closest point
        dist_x = x - closest_x
        dist_y = y - closest_y

        # Return whether the distance is less than or equal to the circle's radius
        if (dist_x ** 2 + dist_y ** 2) <= (r ** 2):
            return True

    return False


fig, ax = plt.subplots()

# 로봇과 궤적을 저장할 리스트들
robots = []
lines = []
starts = []
ends = []

# 임의의 색상
colors = ['blue', 'green', 'purple', 'orange', 'yellow', 'brown', 'pink', 'cyan', 'magenta', 'lime', 'teal', 'lavender', 'turquoise', 'darkgreen', 'tan', 'salmon', 'gold', 'purple', 'darkred', 'darkblue']

# 로봇, 궤적, 시작점, 종료점 생성 및 초기화
for i, path in enumerate(solution):
    robot = Circle((0, 0), config['robotRadii'][i], fill=False, color=colors[i])
    ax.add_patch(robot)
    robots.append(robot)

    ln, = ax.plot([], [], color=colors[i])
    lines.append(ln)

    start = Rectangle((0, 0), 0.5, 0.5, color=colors[i], alpha=0.5)  # small square with transparency
    start.xy = path[0][0] - 0.25, path[0][1] - 0.25  # set bottom left corner
    ax.add_patch(start)
    starts.append(start)

    end = RegularPolygon((0, 0), 3, 0.5, color=colors[i], alpha=0.5)  # triangle with transparency
    end.xy = path[-1][0], path[-1][1]  # set bottom left corner
    ax.add_patch(end)
    ends.append(end)

# Obstacle 생성
obstacles = []
for rect_obstacle in config["rectangleObstacles"]:
    obstacle = Rectangle((rect_obstacle[0] - rect_obstacle[2] / 2, rect_obstacle[1] - rect_obstacle[3] / 2), rect_obstacle[2], rect_obstacle[3], color="black",
                         alpha=0.5)
    ax.add_patch(obstacle)
    obstacles.append(obstacle)


def init():
    ax.set_xlim(0, config['spaceLimits'][0])
    ax.set_ylim(0, config['spaceLimits'][1])

    for i, line in enumerate(lines):
        line.set_data([], [])
        robots[i].center = solution[i][0][0], solution[i][0][1]  # Set the robot's initial position

    return lines + robots + starts + ends + obstacles


def update(frame):
    for i, robot in enumerate(robots):
        x, y, _ = frame[i]
        robot.center = x, y

        old_x, old_y = lines[i].get_data()
        new_x, new_y = np.append(old_x, x), np.append(old_y, y)
        lines[i].set_data(new_x, new_y)

        # Check for collisions
        if check_collision_robots(robot, robots):
            robot.set_edgecolor('red')
            print(f"Robot {i + 1} has collided.")
        elif check_collision_obstacles(robot, obstacles):
            robot.set_edgecolor('red')
            print(f"Robot {i + 1} has collided with the obstacle.")
        else:
            robot.set_edgecolor(colors[i])

    return lines + robots + starts + ends + obstacles


# 보간된 경로 데이터
interpolated_solution = [interpolate_path(path, 100) for path in solution]

ani = animation.FuncAnimation(fig, update, frames=zip(*interpolated_solution), interval=10, init_func=init, blit=True,
                              repeat=True)

plt.show()
