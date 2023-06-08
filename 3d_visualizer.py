import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import yaml
from matplotlib.animation import FuncAnimation

# Your multi agent plan

class RectangleObstacle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height


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

with open('configs/CluttedEnvironment_20_1.yaml') as f:
    config = yaml.safe_load(f)

with open('solutions/CluttedEnvironment_20_1_solutions.yaml') as f:
    plan = yaml.safe_load(f)

# Number of agents
num_agents = len(plan)

# Generate colors for each agent
colors = [
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
    '#87CEEB'  # skyblue
]

# Create 3D plot
# fig size
plt.rc('legend', fontsize=15)
plt.rc('axes', labelsize=20)
fig = plt.figure(figsize=(9, 9))
ax = fig.add_subplot(111, projection='3d')

obstacles = [RectangleObstacle(obstacle["x"], obstacle["y"], obstacle["width"], obstacle["height"]) for obstacle in config['obstacles']]
for obstacle in obstacles:
    cube_faces = create_cube(obstacle.x, obstacle.y, obstacle.width, obstacle.height, 10)
    face_collection = Poly3DCollection(cube_faces, facecolor='b', alpha=0.1, linewidths=1, edgecolors='k')
    ax.add_collection3d(face_collection)

# Plot each agent's plan
for i, agent_plan in enumerate(plan):
    # Unpack coordinates and time
    x_coords, y_coords, time_steps = zip(*agent_plan)

    # Plot this agent's plan
    ax.scatter(x_coords, y_coords, time_steps, color=colors[i], alpha=0.5, s=250)
    ax.plot(x_coords, y_coords, time_steps, color=colors[i], label=f'Agent {i + 1}')

# Configure plot
# camera angle
ax.view_init(90, 0)

# ax.set_title('Space-Time Path of Multiple Agents')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('T')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])
# set width
ax.set_xlim(0, 30)
# set height
ax.set_ylim(0, 30)
# Show plot

plt.show()