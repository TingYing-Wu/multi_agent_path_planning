"""

Extension of SIPP to multi-robot scenarios

author: Ashwin Bose (@atb033)

See the article: 10.1109/ICRA.2011.5980306

"""
import sys, os
sys.path.append( os.path.dirname(os.path.abspath(__file__)) )
print(os.path.dirname(os.path.abspath(__file__)))

import argparse
import yaml
from sipp import SippPlanner

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map and dynamic obstacles")
    parser.add_argument("output", help="output file with the schedule")

    args = parser.parse_args()

    # Read Map
    with open(args.map, 'r') as map_file:
        try:
            cell_map = yaml.load(map_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    # Output file
    output = {"schedule": {}}
    for agent_id in range(len(cell_map["agents"])):
        sipp_planner = SippPlanner(cell_map, agent_id)

        if sipp_planner.compute_plan():
            plan = sipp_planner.get_plan()
            output["schedule"].update(plan)
            # Make the cell as a dynamic obstacle as the agent has been on the cell
            cell_map["dynamic_obstacles"].update(plan)

            with open(args.output, 'w') as output_yaml:
                yaml.safe_dump(output, output_yaml)  
        else: 
            print("Plan not found")


if __name__ == "__main__":
    main()
