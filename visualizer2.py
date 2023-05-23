import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import numpy as np
import yaml

# Your multi agent plan
with open('solutions.yaml') as f:
    plan = yaml.safe_load(f)

plan = [
    [
        [2, 10, 0],
        [3.862739511932436, 9.864534288214493, 1],
        [6.859982905003279, 9.993111279520008, 2],
        [9.95750455413588, 8.655653797567123, 3],
        [13.001829859898413, 8.04018242929156, 4],
        [15.180992094079212, 10.428977073065692, 5],
        [18.0, 10.0, 6]
    ],
    [
        [18, 10, 0],
        [15.054033053631137, 10.56681456482885, 1],
        [12.410298635096854, 11.984794932907757, 2],
        [12.061034153530466, 12.060124093421331, 3],
        [10.815171312586347, 12.126637534252186, 4],
        [8.158759774555861, 12.667727857325106, 5],
        [3.540466811127201, 11.450951640827746, 6],
        [3.5838723150692364, 11.28912558996654, 7],
        [2.0, 10.0, 8]
    ]
]

# Number of agents
num_agents = len(plan)

# Generate colors for each agent
colors = cm.rainbow(np.linspace(1, 0, num_agents))

# Create 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot each agent's plan
for i, agent_plan in enumerate(plan):
    # Unpack coordinates and time
    x_coords, y_coords, time_steps = zip(*agent_plan)

    # Plot this agent's plan
    ax.scatter(x_coords, y_coords, time_steps, color=colors[i], s=500, label=f'Agent {i+1}')

# Configure plot
# camera angle
ax.view_init(20, 65)

ax.set_title('Space-Time Path of Multiple Agents')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('T')
# set width
ax.set_xlim(0, 20)
# set height
ax.set_ylim(0, 20)
ax.legend()

# Show plot
plt.show()