"""

Python implementation of Conflict-based search

author: Ashwin Bose (@atb033)

"""
import sys
sys.path.insert(0, '../')
import argparse
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy

from a_star import AStar

class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
        
    def __eq__(self, other) -> bool:
        # Check equality with another location
        return self.x == other.x and self.y == other.y
    
    def __str__(self) -> str:
        # String representation of location
        return str((self.x, self.y))

class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location
    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    
    def __hash__(self) -> int:
        # Hash function for state
        return hash(str(self.time) + str(self.location.x) + str(self.location.y))
    
    def is_equal_except_time(self, state) -> bool:
        # Check if two states are equal except for time
        return self.location == state.location
    
    def __str__(self):
        return str((self.time, self.location.x, self.location.y))


class Conflict(object):
    VERTEX = 1
    EDGE = 2
    def __init__(self, time=-1, conflict_type=-1, agent_1='', agent_2='', location_1=Location(), location_2=Location()):
        self.time = time
        self.type = conflict_type
        self.agent_1 = agent_1
        self.agent_2 = agent_2
        self.location_1 = location_1
        self.location_2 = location_2

    def __str__(self):
        return f'({str(self.time)}, {self.agent_1}, {self.agent_2}, {str(self.location_1)}, {str(self.location_2)})'


class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location))
    def __str__(self):
        return f'({str(self.time)}, {str(self.location)})'


class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2
    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2
    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))
    def __str__(self):
        return f'({str(self.time)}, {str(self.location_1)}, {str(self.location_2)})'

class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set() # Set of VertexConstraint
        self.edge_constraints = set()   # Set of EdgeConstraint

    def add_constraint(self, other: 'Constraints'):
        # Add constraints from another constraint object
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return f"VC: {[str(vc) for vc in self.vertex_constraints]}EC: {[str(ec) for ec in self.edge_constraints]}"


class Environment(object):
    def __init__(self, dimension: tuple, agents: list, obstacles: list):
        self.dimension = dimension  # Dimensions of the grid, e.g. [3, 3]
        self.obstacles = obstacles  # List of obstacle locations

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

    def get_neighbors(self, state) -> list:
        """ Get valid neighboring states of a given state """
        neighbors = []

        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        return neighbors


    def get_first_conflict(self, solution: dict):
        """ Examine the paths of all agents to find any instance where two agents
            1. occupy the same location at the same time (vertex conflict)
            2. cross the same edge in opposite directions at the same time (edge conflict) 
        """
        # Get the entire duration of the plan
        max_t = max(len(plan) for plan in solution.values())
        result = Conflict()
        # Identify the first conflict in the solution by traversing the entire plan
        for t in range(max_t):
            # Check vertex conflict
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                # Fetch the State(time, position) of each agent at time t
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                # If the locations of both states are equal, that is the agents in the same location at the same time
                if state_1.is_equal_except_time(state_2):
                    result = Conflict(time = t, conflict_type = Conflict.VERTEX, 
                                      agent_1 = agent_1, agent_2 = agent_2, 
                                      location_1 = state_1.location)
                    return result

            # Check edge conflict
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)
                # Edge conflict, if the agents swap their locations between these time steps.
                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result = Conflict(time = t, conflict_type = Conflict.EDGE, 
                                      agent_1 = agent_1, agent_2 = agent_2, 
                                      location_1 = state_1a.location, location_2 = state_1b.location)
                    return result
        return False

    def create_constraints_from_conflict(self, conflict: Conflict) -> dict:
        # Create constraints from a given conflict
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            # Create for the time and location of the conflict
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            # Both agents involved in the conflict are given the same constraint to avoid the conflicting location at the specific time
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1 # 1 -> 2
            constraint_dict[conflict.agent_2] = constraint2 # 2 -> 1

        return constraint_dict

    def get_state(self, agent_name: str, solution: dict, t: int) -> State:
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state: State) -> bool:
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.obstacles

    def transition_valid(self, state_1: State, state_2: State) -> bool:
        # Check if a transition between two states is valid (not violating edge constraints)
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state: State, agent_name: str) -> float:
        # Calculate admissible heuristic for A* search
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)

    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state}})

    def compute_solution(self) -> dict:
        """ Compute the solution for all agents using A* search algorithm """
        solution = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            if local_solution := self.a_star.search(agent):
                solution[agent] = local_solution
            else:
                return False
        return solution

    def compute_solution_cost(self, solution: dict) -> int:
        return sum(len(path) for path in solution.values())


class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost


class CBS(object):
    def __init__(self, environment: Environment):
        self.env = environment  # Initialize CBS algorithm with an environment
        self.open_set = set()   # Open set for the high-level search
        self.closed_set = set() # Closed set for the high-level search
        
    def search(self) -> dict:
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        while self.open_set:
            # Find the HighLevelNode in the open_set with the minimum cost
            P = min(self.open_set)  # P is a HighLevelNode with the minimum cost
            self.open_set -= {P}
            self.closed_set |= {P}

            self.env.constraint_dict = P.constraint_dict    # Include vertex and edge constraints
            conflict_dict = self.env.get_first_conflict(P.solution)
            if not conflict_dict:
                # No conflict found, the plan based on A* search is the optimal solution
                print("solution found")
                return self.generate_plan(P.solution)

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    def generate_plan(self, solution: dict) -> dict:
        """ Generate the final plan from the solution """
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("param", help="input file containing map and obstacles, e.g. input.yaml")
    parser.add_argument("output", help="output file with the schedule, e.g. output.yaml")
    args = parser.parse_args()

    # Read parameters including map and agents from the input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    obstacles = param["map"]["obstacles"]
    agents = param['agents']
    
    # Initialize the environment based on the map, obstacles and agents
    env = Environment(dimension, agents, obstacles)

    # Searching
    cbs = CBS(env)
    solution = cbs.search()
    if not solution:
        print(" Solution not found" )
        return

    # Write to output file
    output = {"schedule": solution, "cost": env.compute_solution_cost(solution)}
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)


if __name__ == "__main__":
    main()
