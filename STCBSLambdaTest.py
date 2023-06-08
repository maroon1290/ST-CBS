from STCBS import *
import yaml
import os
import time
import multiprocessing


if __name__ == '__main__':
    name_list = ["lambda_0", "lambda_25", "lambda_50", "lambda_75", "lambda_100"]
    for name in name_list:
        count = 1
        space_makespan_list = []
        sum_of_space_costs_list = []
        time_makespan_list = []
        sum_of_time_costs_list = []
        space_time_makespan_list = []
        sum_of_space_time_costs_list = []
        solutions_list = []
        compute_time_list = []
        for i in range(count):
            config_name = "OpenEnvironment_5_2"
            # read config.yaml
            with open(os.path.join("configs", config_name + ".yaml"), "r") as file:
                config = yaml.safe_load(file)

            if name == "lambda_0":
                config["lambda_factor"] = 0
            elif name == "lambda_25":
                config["lambda_factor"] = 0.25
            elif name == "lambda_50":
                config["lambda_factor"] = 0.5
            elif name == "lambda_75":
                config["lambda_factor"] = 0.75
            elif name == "lambda_100":
                config["lambda_factor"] = 1

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
            print(id(st_cbs))
            space_makespan = None
            sum_of_space_costs = None
            time_makespan = None
            sum_of_time_costs = None
            space_time_makespan = None
            sum_of_space_time_costs = None
            solutions = None
            start_time = time.time()
            pool = multiprocessing.Pool(processes=1)
            result = pool.apply_async(st_cbs.planning)
            try:
                space_makespan, sum_of_space_costs, time_makespan, sum_of_time_costs, space_time_makespan, sum_of_space_time_costs, solutions = result.get(timeout=300)
            except multiprocessing.TimeoutError:
                print(f"{config_name} didn't finish within 5 minutes. Terminating...")
                pool.terminate()
            else:
                pool.close()
            pool.join()
            end_time = time.time()
            compute_time = end_time - start_time
            if solutions:
                print(f"Lambda Test: {config['lambda_factor']}")
                print(f"{config_name} finished in {compute_time} seconds")
                print(f"space_makespan: {space_makespan}")
                print(f"sum_of_space_costs: {sum_of_space_costs}")
                print(f"time_makespan: {time_makespan}")
                print(f"sum_of_time_costs: {sum_of_time_costs}")
                print(f"space_time_makespan: {space_time_makespan}")
                print(f"sum_of_space_time_costs: {sum_of_space_time_costs}")

                for solution in solutions:
                    for node in solution:
                        print(f"[{node.x}, {node.y}, {node.t}],")
                    print("--------------------------------")

                # save solutions to yaml
                with open(f"solutions/{config_name}_{name}_solutions.yaml", "w") as file:
                    solutions_list = []
                    for solution in solutions:
                        solution_list = []
                        for node in solution:
                            solution_list.append([node.x, node.y, node.t])
                        solutions_list.append(solution_list)
                    yaml.dump(solutions_list, file)

                space_makespan_list.append(space_makespan)
                sum_of_space_costs_list.append(sum_of_space_costs)
                time_makespan_list.append(time_makespan)
                sum_of_time_costs_list.append(sum_of_time_costs)
                space_time_makespan_list.append(space_time_makespan)
                sum_of_space_time_costs_list.append(sum_of_space_time_costs)
                compute_time_list.append(compute_time)
            else:
                space_makespan_list.append(None)
                sum_of_space_costs_list.append(None)
                time_makespan_list.append(None)
                sum_of_time_costs_list.append(None)
                space_time_makespan_list.append(None)
                sum_of_space_time_costs_list.append(None)
                compute_time_list.append(None)

        # save makespan and sum of costs to yaml
        with open(f"solutions/{config_name}_{name}_raw_data.csv", "a") as file:
            for space_makespan, sum_of_space_costs, time_makespan, sum_of_time_costs, space_time_makespan, sum_of_space_time_costs, compute_time in zip(space_makespan_list, sum_of_space_costs_list, time_makespan_list, sum_of_time_costs_list, space_time_makespan_list, sum_of_space_time_costs_list, compute_time_list):
                file.write(f"{space_makespan}, {sum_of_space_costs}, {time_makespan}, {sum_of_time_costs}, {space_time_makespan}, {sum_of_space_time_costs}, {compute_time}\n")
