import matplotlib.pyplot as plt
import matplotlib.patches as patches
import yaml

data = "configs/OpenEnvironment_20_6.yaml"
# your yaml data goes here

# load the yaml data
with open(data, 'r') as data:
    data_dict = yaml.safe_load(data)

fig, ax = plt.subplots()

# set the limits of the plot to the grid dimensions
ax.set_xlim([0, int(data_dict['width'])])
ax.set_ylim([0, int(data_dict['height'])])

# plot the starting and goal positions for each robot
for start, goal, radius in zip(data_dict['starts'], data_dict['goals'], data_dict['robot_radii']):
    start_x, start_y = start
    goal_x, goal_y = goal

    # plot starting positions as red dots
    ax.plot(start_x, start_y, 'ro')

    # plot goal positions as green dots
    ax.plot(goal_x, goal_y, 'go')

    # plot robot as circle with given radius
    robot = patches.Circle((start_x, start_y), radius, fill=False)
    ax.add_patch(robot)

# show the plot
plt.show()