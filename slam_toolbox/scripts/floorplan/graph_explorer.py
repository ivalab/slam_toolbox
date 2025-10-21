from typing import Any, List, Tuple, Union

# from run_autonomy.exploration import WaypointsExplorer

import networkx as nx
import numpy as np

from graph_utils import read_graph_from_file


# TODO: Make this class smarter.
# It should keep track of its own set of goals, and handle them like
#     the base WaypointsExplorer.
# Should we reroute if new goals are added? Annoying statefulness that
#     I am currently too lazy to deal with.
class GraphExplorer:
    """Given a Networkx graph, solve for an exploration route using TSP and follow it.

    In no small part ripped from: https://github.gatech.edu/ivabots/floor_nav/blob/main/src/floor_nav/tsp_calculator.py
    """

    def __init__(self):
        super().__init__()

    def set_graph(self, G: nx.Graph, targets: List[Any], start: Union[Any, Tuple[float, float]], loop=True):
        """Given a graph and a set of waypoints, compute a tour starting and
        ending at the given node index, that hits at least the targeted nodes.

        Uses nx TSP solver.

        @param G            nx graph, requires attributes:
                                edge: weight
                                node: position (x, y)
        @param targets      List of node IDs that must be hit.
                                Type of Node ID depends on the graph.
        @param start        One of:
                                Node ID to start at. Type depends on the graph.
                                (x, y) pos to start at. Tuple(float, float)

                            In the latter case, the closest node is picked as the start node.
        """
        path = nx.approximation.traveling_salesman_problem(G, weight="weight", nodes=targets)
        try:
            # If start is a pair...
            len(start)

            # Identify closest node to the start.
            start = np.array(start)
            best_idx = path[0]
            best_dist = np.inf
            for i, node_label in enumerate(path):
                pos = G.nodes[node_label]["position"]
                dist = np.linalg.norm(start - pos)
                if dist < best_dist:
                    best_idx = i
                    best_dist = dist

            start_idx = best_idx
        except:
            start_idx = None
            for i, node_label in enumerate(path):
                if node_label == start:
                    start_idx = i
                    break

        path_pos = []
        for i in range(len(path)):
            pos = [*G.nodes[path[start_idx]]["position"], 0]
            path_pos.append(pos)
            start_idx += 1
            if start_idx == len(path):
                start_idx = 0

        if len(path_pos) > 1 and loop:
            path_pos.append(path_pos[0])

        return path_pos

    #     self.set_goals(path_pos)

    # def explore(self) -> None:
    #     """@see WaypointsExplorer.explore()"""
    #     return super().explore()

    # def is_done(self) -> bool:
    #     """@see WaypointsExplorer.is_done()"""
    #     return super().is_done()

    # def interrupt(self) -> None:
    #     """@see WaypointsExplorer.interrupt()"""
    #     return super().interrupt()


def main():
    filename = "/mnt/IVALAB/dropbox/GaTech Dropbox/Yanwei Du/Data/Data_RoboSLAM/Research/MappingProject/TSRB_Maps/Floorplan/fourth_floor_realworld.pgm.json"

    G = read_graph_from_file(filename)
    explore = GraphExplorer()
    targets = G.nodes()
    start = list(G.nodes().keys())[0]
    path_pos = explore.set_graph(G, targets, start)
    print(path_pos)


if __name__ == "__main__":
    main()
