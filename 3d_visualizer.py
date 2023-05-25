import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import numpy as np
import yaml

# Your multi agent plan
with open('solutions/OpenEnvironment_15_3_solutions.yaml') as f:
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
fig = plt.figure(figsize=(9, 9))
ax = fig.add_subplot(111, projection='3d')

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

ax.set_title('Space-Time Path of Multiple Agents')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('T')
# set width
ax.set_xlim(0, 30)
# set height
ax.set_ylim(0, 30)
ax.legend()

# Show plot
plt.show()