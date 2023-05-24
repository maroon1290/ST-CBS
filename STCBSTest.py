from STCBS import *
import yaml
import os
import time
import multiprocessing


if __name__ == '__main__':
    count = 10
    makespan_list = []
    sum_of_costs_list = []
    compute_time_list = []
    for i in range(count):
        config_name = f"OpenEnvironment_15_{i}"
        # read config.yaml
        with open(os.path.join("configs", config_name + ".yaml"), "r") as file:
            config = yaml.safe_load(file)

        # make obstacles
        obstacles = []
        for config_obstacle in config["obstacles"]:
            if config_obstacle["type"] == "CircleObstacle":
                obstacle = CircleObstacle(config_obstacle["x"], config_obstacle["y"], config_obstacle["radius"])
            elif config_obstacle["type"] == "RectangleObstacle":
                obstacle = RectangleObstacle(config_obstacle["x"], config_obstacle["y"], config_obstacle["width"], config_obstacle["height"])
            else:
                raise ValueError("invalid obstacle type")
            obstacles.append(obstacle)

        # run MARRF
        st_cbs = STCBS(
            config["starts"],
            config["goals"],
            config["width"],
            config["height"],
            config["robot_radii"],
            config["lambda_factor"],
            config["expand_distances"],
            obstacles,
            config["robot_num"],
            config["neighbor_radius"],
            config["max_iter"]
        )

        makespan = None
        sum_of_costs = None
        solutions = None
        start_time = time.time()
        pool = multiprocessing.Pool(processes=1)
        result = pool.apply_async(st_cbs.planning)
        try:
            makespan, sum_of_costs, solutions = result.get(timeout=300)
        except multiprocessing.TimeoutError:
            print(f"{config_name} didn't finish within 5 minutes. Terminating...")
            pool.terminate()
        else:
            pool.close()
        pool.join()
        end_time = time.time()
        if solutions:
            print(f"Time: {end_time - start_time}")
            print("Makespan: ", makespan)
            print("Sum of costs: ", sum_of_costs)
            for solution in solutions:
                for node in solution:
                    print(f"[{node.x}, {node.y}, {node.t}],")
                print("--------------------------------")

            # save solutions to yaml
            with open(f"solutions/{config_name}_solutions.yaml", "w") as file:
                solutions_list = []
                for solution in solutions:
                    solution_list = []
                    for node in solution:
                        solution_list.append([node.x, node.y, node.t])
                    solutions_list.append(solution_list)
                yaml.dump(solutions_list, file)

            makespan_list.append(makespan)
            sum_of_costs_list.append(sum_of_costs)
            compute_time_list.append(end_time - start_time)
        else:
            makespan_list.append(None)
            sum_of_costs_list.append(None)
            compute_time_list.append(None)

    # save makespan and sum of costs to yaml
    with open(f"solutions/{config_name}_raw_data.csv", "w") as file:
        for i in range(count):
            file.write(f"{makespan_list[i]}, {sum_of_costs_list[i]}, {compute_time_list[i]} \n")
