"""Utility functions for dealing with Networkx graphs."""
import os

import networkx as nx
import numpy as np

def plot_graph(G: nx.Graph):
    """Draw a graph into matplotlib.
    Draws Edges as straight lines.

    Assumes the nodes have attributes "position": (x, y), and that this is not a multigraph.
    """
    pos = nx.get_node_attributes(G, 'position')

    segments = []
    pos_arr = np.array([pos[n] for n in G.nodes])
    for a, b in G.edges:
        segments.append([pos[a], pos[b]])

    import matplotlib.pyplot as plt # Delayed import, in case your file doesn't care about plotting.
    #https://stackoverflow.com/questions/21352580/plotting-numerous-disconnected-line-segments-with-different-colors
    from matplotlib import collections as mc
    plt.figure()
    plt.scatter(pos_arr[:, 0], pos_arr[:, 1])
    lc = mc.LineCollection(segments, linewidths=2)
    ax = plt.gca()
    ax.add_collection(lc)
    ax.set_aspect('equal')

def read_graph_from_file(fname: str, ftype:str = "auto") -> nx.Graph:
    """Reads graph data from a file.

    Currently supported types:
        - Networkx JSON
        - HDF5 (custom format output by matlab)

    @param fname    Filename to read
    @param ftype    File type (default: "auto", infers based on file extension.)
                    One of: [ json, hdf5 ]

    @return nx.Graph representing parsed data.
    """
    if ftype == "auto":
        if fname.endswith('.json'):
            ftype = "json"
        elif fname.endswith('.h5'):
            ftype = "hdf5"

    fname = os.path.expanduser(fname)

    if ftype == "json":
        import json
        from networkx.readwrite import json_graph
        with open(fname) as f:
            data = json.load(f)
        return json_graph.node_link_graph(data)
    if ftype == "hdf5":
        # Somewhat inspired by: https://github.gatech.edu/ivabots/floor_nav/blob/main/src/floor_nav/hdf5_parser.py
        import h5py
        dataset = h5py.File(fname, 'r')['roadmap']
        vertices = dataset['vertices']
        edges = dataset['edges']
        G = nx.Graph()

        # NOTE: We explicitly convert all types to python types from numpy types, for JSON serialization.
        for v in vertices:
            vert_data = vertices[v].attrs
            vert_xy = vertices[v][0]
            G.add_node(int(vert_data['id'][0]), position=(float(vert_xy[0]), float(vert_xy[1])))
        for e in edges:
            edge_data = edges[e].attrs
            G.add_edge(
                int(edge_data['startVertexId'][0]),
                int(edge_data['endVertexId'][0]),
                weight=float(edge_data['value'][0])
            )
        return G

    raise ValueError(f"Invalid file type for graph reading: {ftype}")

def write_graph_to_file(fname: str, G: nx.Graph, ftype:str = "auto"):
    """Writes graph data to a file.

    Currently supported types:
        - Networkx JSON

    @param fname    Filename to write.
    @param G        nx.Graph representing data to write.
    @param ftype    File type (default: "auto", infers based on file extension.)
                    One of: [ json, ]
    """
    if ftype == "auto":
        if fname.endswith('.json'):
            ftype = "json"

    if ftype == "json":
        import json
        from networkx.readwrite import json_graph
        with open(fname, 'w') as f:
            json.dump(json_graph.node_link_data(G), f)
        return

    raise ValueError(f"Invalid file type for graph writing: {ftype}")
