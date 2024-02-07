"""

SIPP implementation  

author: Ashwin Bose (@atb033)

See the article: DOI: 10.1109/ICRA.2011.5980306

"""

import argparse
import yaml
from math import fabs
from graph_generation import SippGraph, State

# SippPlanner class inherits from SippGraph
class SippPlanner(SippGraph):
    def __init__(self, map, agent_id):
        """
        Args:
            map (dict): The map configuration containing the map dimensions, obstacles, etc.
            agent_id (int): The identifier for the specific agent
        """
        # Initialize the base class (SippGraph)
        SippGraph.__init__(self, map)
        # Set the starting position, goal, and name for the agent defined in input.yaml
        self.start = tuple(map["agents"][agent_id]["start"])
        self.goal = tuple(map["agents"][agent_id]["goal"])
        self.name = map["agents"][agent_id]["name"]
        # Initialize the open list for the A* algorithm
        self.open = []

    def get_successors(self, state: State) -> list:
        """ Get successors (list) for the current state (State)
        """
        successors = []
        m_time = 1  # Assuming each move takes 1 time unit
        neighbour_list = self.get_valid_neighbours(state.position)

        # Iterate over each valid neighbour and check for valid intervals
        for neighbour in neighbour_list:
            start_t = state.time + m_time
            end_t = state.interval[1] + m_time
            for i in self.sipp_graph[neighbour].interval_list:
                # TODO: Why not filter out the node existed in the open or close list?
                # checks if the robot can move into the neighboring position during its safe interval without colliding.
                if i[0] > end_t or i[1] < start_t:
                    continue
                time = max(start_t, i[0]) 
                s = State(neighbour, time, i)
                successors.append(s)
        return successors

    def get_heuristic(self, position: tuple) -> float:
        """ Heuristic function for A* algorithm (Manhattan distance) """
        return fabs(position[0] - self.goal[0]) + fabs(position[1]-self.goal[1])

    def compute_plan(self):
        """compute the optimal path
        
        Returns: 1 if a plan is found, 0 otherwise
        """
        self.open = []
        goal_reached = False
        cost = 1    # Assuming uniform cost for simplicity

        # Initialize the start state and set its g and f values
        s_start = State(self.start, 0) 

        self.sipp_graph[self.start].g = 0.
        f_start = self.get_heuristic(self.start)
        self.sipp_graph[self.start].f = f_start

        # Add the start state to the open list
        self.open.append((f_start, s_start))

        while (not goal_reached):
            if self.open == {}: 
                # Plan not found
                return 0
            
            # Pop the node with the lowest f value
            s = self.open.pop(0)[1] # [1] accesses the second element of the tuple returned by pop(0). The tuple is likely structured as (f-value, state), so [1] gets the state.
            successors = self.get_successors(s)
    
            for successor in successors:
                # If new cost is lower, update node's cost and parent
                if self.sipp_graph[successor.position].g > self.sipp_graph[s.position].g + cost: # Filter out the passed, lower node
                    self.sipp_graph[successor.position].g = self.sipp_graph[s.position].g + cost
                    self.sipp_graph[successor.position].parent_state = s

                    if successor.position == self.goal:
                        print("Plan successfully calculated!!")
                        goal_reached = True
                        break
                    
                    # Update f value (= g + h) and reinsert node into open list
                    self.sipp_graph[successor.position].f = self.sipp_graph[successor.position].g + self.get_heuristic(successor.position)
                    self.open.append((self.sipp_graph[successor.position].f, successor))

        # Backtrack to construct the plan from goal to start
        start_reached = False
        self.plan = []
        current = successor
        while not start_reached:
            self.plan.insert(0,current)
            if current.position == self.start:
                start_reached = True
            current = self.sipp_graph[current.position].parent_state
        return 1    # Return 1 indicating successful plan computation
            
    
    def get_plan(self) -> dict:
        """ Retrieve the computed plan as a list of setpoints
        """
        path_list = []

        # first setpoint
        setpoint = self.plan[0]
        temp_dict = {"x":setpoint.position[0], "y":setpoint.position[1], "t":setpoint.time}
        path_list.append(temp_dict)

        for i in range(len(self.plan)-1):
            for j in range(self.plan[i+1].time - self.plan[i].time-1):
                x = self.plan[i].position[0]
                y = self.plan[i].position[1]
                t = self.plan[i].time
                setpoint = self.plan[i]
                temp_dict = {"x":x, "y":y, "t":t+j+1}
                path_list.append(temp_dict)

            setpoint = self.plan[i+1]
            temp_dict = {"x":setpoint.position[0], "y":setpoint.position[1], "t":setpoint.time}
            path_list.append(temp_dict)

        return {self.name:path_list}



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map and dynamic obstacles")
    parser.add_argument("output", help="output file with the schedule")

    args = parser.parse_args()

    with open(args.map, 'r') as map_file:
        try:
            map = yaml.load(map_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)


    # compute first plan
    sipp_planner = SippPlanner(map,0)

    if sipp_planner.compute_plan():
        plan = sipp_planner.get_plan()
        output = {"schedule": {}}
        output["schedule"].update(plan)
        with open(args.output, 'w') as output_yaml:
            yaml.safe_dump(output, output_yaml)
    else: 
        print("Plan not found")


if __name__ == "__main__":
    main()