import math

import yaml
import random

from shapely.geometry import Point, box

if __name__ == '__main__':
    count = 10
    for i in range(count):
        basename = "CluttedEnvironment_20"
        with open(f'configs/{basename}.yaml', 'r') as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        width = config["width"]
        height = config["height"]
        obstacles = config["obstacles"]
        radius = config["robot_radii"][0]
        robot_num = config["robot_num"]

        search_min_width = radius
        search_min_height = radius
        search_max_width = width - radius
        search_max_height = height - radius

        # generate random starts and goals for each robot
        starts = []
        goals = []
        for _ in range(robot_num):
            def generate_start_and_goal():
                while True:
                    start = [random.uniform(search_min_width, search_max_width),
                             random.uniform(search_min_height, search_max_height)]
                    goal = [random.uniform(search_min_width, search_max_width),
                            random.uniform(search_min_height, search_max_height)]
                    conflict = False
                    for previous_start, previous_goal in zip(starts, goals):
                        if math.hypot(previous_start[0] - start[0], previous_start[1] - start[1]) < radius + radius or \
                                math.hypot(previous_goal[0] - start[0], previous_goal[1] - start[1]) < radius + radius:
                            conflict = True
                        if math.hypot(previous_start[0] - goal[0], previous_start[1] - goal[1]) < radius + radius or \
                                math.hypot(previous_goal[0] - goal[0], previous_goal[1] - goal[1]) < radius + radius:
                            conflict = True

                    for obstacle in obstacles:
                        start_circle = Point(start[0], start[1]).buffer(radius)
                        goal_circle = Point(goal[0], goal[1]).buffer(radius)
                        obstacle_rectangle = box(obstacle['x'] - obstacle["width"] / 2.0, obstacle['y'] - obstacle["height"] / 2.0,
                                        obstacle['x'] + obstacle["width"] / 2.0,
                                        obstacle['y'] + obstacle["height"] / 2.0)

                        if obstacle_rectangle.intersects(start_circle) or obstacle_rectangle.intersects(goal_circle):
                            conflict = True

                    if not conflict:
                        return start, goal

            start, goal = generate_start_and_goal()
            starts.append(start)
            goals.append(goal)

        config["starts"] = starts
        config["goals"] = goals

        # dump config
        with open(f'configs/{basename}_{i}.yaml', 'w') as f:
            yaml.dump(config, f)
            print(f"Dumped {basename}_{i}.yaml")