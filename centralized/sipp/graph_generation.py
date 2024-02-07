"""

Graph generation for sipp 

author: Ashwin Bose (@atb033)

See the article: 10.1109/ICRA.2011.5980306

"""

import argparse
import yaml
from bisect import bisect

# State of an agent storing the position, time and interval
class State(object):
    def __init__(self, position: tuple =(-1,-1), t: int = 0, interval: tuple = (0,float('inf'))):
        self.position = tuple(position)
        self.time = t
        self.interval = interval

# Represent a grid cell in SIPP
class SippGrid(object):
    def __init__(self):
        # self.position = ()
        self.interval_list = [(0, float('inf'))]
        self.f = float('inf')   # Initialize f-value (= g + h) for A* algorithm
        self.g = float('inf')
        self.parent_state = State()

    def split_interval(self, t: int, last_t: bool = False):
        """ Generates safe intervals by splitting existing intervals based on the time step.
        """
        for interval in self.interval_list:
            if last_t:
                if t<=interval[0]:
                    self.interval_list.remove(interval)
                elif t>interval[1]:
                    continue
                else:
                    self.interval_list.remove(interval)
                    self.interval_list.append((interval[0], t-1))
            elif t == interval[0]:
                self.interval_list.remove(interval)
                if t+1 <= interval[1]:
                    self.interval_list.append((t+1, interval[1]))
            elif t == interval[1]:
                self.interval_list.remove(interval)
                if t-1 <= interval[0]:
                    self.interval_list.append((interval[0],t-1))
            elif bisect(interval,t) == 1:
                self.interval_list.remove(interval)
                self.interval_list.append((interval[0], t-1))
                self.interval_list.append((t+1, interval[1]))
            self.interval_list.sort()

class SippGraph(object):
    def __init__(self, map: dict):
        self.map = map
        self.dimensions = map["map"]["dimensions"]
        # Extract static obstacles from the map and store as tuples
        self.obstacles = [tuple(v) for v in map["map"]["obstacles"]]        
        self.dyn_obstacles = map["dynamic_obstacles"]

        self.sipp_graph: dict[tuple, SippGrid] = {}  # e.g. {(0, 0):SippGrid()}, in which the class of SippGrid(interval_list, f, g, State) 
        self.init_graph()
        self.init_intervals()

    def init_graph(self):
        """Iterate through the dimensions to create grid cells"""
        for i in range(self.dimensions[0]): # e.g. dimensions: [3, 3]
            for j in range(self.dimensions[1]):
                grid_dict = {(i, j):SippGrid()} # dictionary with tuple keys. E.g. {(0, 0):SippGrid()}
                # update() is a standard function for Python dictionaries. It adds key-value pairs from grid_dict to self.sipp_graph. 
                # If a key from grid_dict already exists in self.sipp_graph, its value is updated; otherwise, the key-value pair is added.
                self.sipp_graph.update(grid_dict)

    def init_intervals(self):
        if not self.dyn_obstacles: return
        # Process dynamic obstacles to update intervals
        for schedule in self.dyn_obstacles.values():
            # for location in schedule:
            for i in range(len(schedule)):
                location = schedule[i]
                last_t = i == len(schedule)-1   # Check if last time step (true or false)

                position = (location["x"],location["y"])
                t = location["t"]

                self.sipp_graph[position].split_interval(t, last_t)
                # print(str(position) + str(self.sipp_graph[position].interval_list))     

    def is_valid_position(self, position: tuple[int, int]) -> bool:
        """ Check if a position is valid (within dimensions and not an obstacle)
        Arguments:
            position (tuple[int, int]): The position to check
        """
        dim_check = position[0] in range(self.dimensions[0]) and  position[1] in range(self.dimensions[1])
        obs_check = position not in self.obstacles
        # print(dim_check)
        return dim_check and obs_check

    def get_valid_neighbours(self, position: tuple[int, int]) -> list[tuple[int, int]]:
        """ Get valid neighbours for a position
        Arguments:
            position (tuple[int, int]): The position to check
        Returns: list[tuple[int, int]]: List of valid neighbouring positions
        """
        neighbour_list = []
        
        # Check each direction (up, down, left, right) for valid neighbors
        up = (position[0], position[1]+1)
        if self.is_valid_position(up): neighbour_list.append(up)

        down = (position[0], position[1]-1)
        if self.is_valid_position(down): neighbour_list.append(down)

        left = (position[0]-1, position[1])
        if self.is_valid_position(left): neighbour_list.append(left)

        right = (position[0]+1, position[1])
        if self.is_valid_position(right): neighbour_list.append(right)

        # Replace the above with the following to check 4 directions (or diagonals appended as well)
        # directions = [(0, 1), (0, -1), (-1, 0), (1, 0)]
        # for dx, dy in directions:
        #     neighbour = (position[0] + dx, position[1] + dy)
        #     if self.is_valid_position(neighbour):
        #         neighbour_list.append(neighbour)

        return neighbour_list


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map and dynamic dyn_obstacles")
    args = parser.parse_args()
    
    with open(args.map, 'r') as map_file:
        try:
            cell_map = yaml.load(map_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    graph = SippGraph(cell_map)
    # print(graph.get_valid_neighbours((0,0)))
    # print(graph.sipp_graph[(1,1)].interval_list)
    # print(graph.get_valid_neighbours((1,2)))

if __name__ == "__main__":
    main()

