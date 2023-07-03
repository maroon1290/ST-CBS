import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import yaml

with open("../BaseName.yaml", 'r') as stream:
    basename_yaml = yaml.load(stream, Loader=yaml.FullLoader)
    basename = basename_yaml["basename"]

# 로봇의 경로
with open("paths/path.yaml", 'r') as stream:
    path = yaml.load(stream, Loader=yaml.FullLoader)

print(path)
# x, y, time 좌표 추출
x_coords = [point["x"] for point in path]
y_coords = [point["y"] for point in path]
time_coords = [point["time"] for point in path]

# 3D 그래프 설정
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 로봇의 경로 표시
ax.plot(x_coords, y_coords, time_coords, marker='o')

# 그래프의 축 설정
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Time')

# 원의 중심 좌표와 반지름
circle_center = (5, 5)
circle_radius = 2

# 원기둥 그리기
z = np.linspace(0, 5, 100)
theta = np.linspace(0, 2.*np.pi, 100)
theta_grid, z_grid=np.meshgrid(theta, z)
x_grid = circle_radius * np.cos(theta_grid) + circle_center[0]
y_grid = circle_radius * np.sin(theta_grid) + circle_center[1]

ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.5, rstride=100, cstride=100)

# 그래프 출력
plt.show()
