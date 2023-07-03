import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import yaml


with open("../BaseName.yaml", 'r') as stream:
    basename_yaml = yaml.load(stream, Loader=yaml.FullLoader)
    basename = basename_yaml["basename"]
config_file = f"../configs/{basename}.yaml"
solution_file = f"../solutions/{basename}_solution.yaml"

# configs 파일
with open(config_file, 'r') as stream:
    config = yaml.load(stream, Loader=yaml.FullLoader)
# 로봇의 경로
with open(solution_file, 'r') as stream:
    solution = yaml.load(stream, Loader=yaml.FullLoader)

# 3D 그래프 설정
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for path in solution:
    # x, y, time 좌표 추출
    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]
    time_coords = [point[2] for point in path]

    # 로봇의 경로 표시
    ax.plot(x_coords, y_coords, time_coords, marker='o')

# 그래프의 축 설정
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Time')
ax.set_xlim(0, config['spaceLimits'][0])
ax.set_ylim(0, config['spaceLimits'][1])

# 원기둥 그리기
# circle_center = (5, 5)
# circle_radius = 2
# z = np.linspace(0, 5, 100)
# theta = np.linspace(0, 2.*np.pi, 100)
# theta_grid, z_grid=np.meshgrid(theta, z)
# x_grid = circle_radius * np.cos(theta_grid) + circle_center[0]
# y_grid = circle_radius * np.sin(theta_grid) + circle_center[1]
#
# ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.5, rstride=100, cstride=100)

# cube 그리기
def create_cube(center_x, center_y, width, height, depth):
    x, y = center_x - width / 2, center_y - height / 2

    vertices = np.array([
        [x, y, 0],
        [x + width, y, 0],
        [x + width, y + height, 0],
        [x, y + height, 0],
        [x, y, depth],
        [x + width, y, depth],
        [x + width, y + height, depth],
        [x, y + height, depth]
    ])

    faces = [
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[7], vertices[6], vertices[2], vertices[3]],
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[7], vertices[6], vertices[5], vertices[4]],
        [vertices[7], vertices[3], vertices[0], vertices[4]],
        [vertices[1], vertices[2], vertices[6], vertices[5]]
    ]

    return faces


# find max time in solution
max_time = max(solution[0], key=lambda x: x[2])[2]
for rect_obstacle in config["rectangleObstacles"]:
    cube_faces = create_cube(rect_obstacle[0], rect_obstacle[1], rect_obstacle[2], rect_obstacle[3], max_time)
    face_collection = Poly3DCollection(cube_faces, facecolor='b', alpha=0.1, linewidths=1, edgecolors='k')
    ax.add_collection3d(face_collection)

# 그래프 출력
plt.show()
