#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file generate_route_graph.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 10-20-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""


import math, numpy as np, networkx as nx
from collections import defaultdict


# ------------ 1) Cluster poses with a coarse grid (no sklearn) ------------
def cluster_poses_grid(poses, cell_size=1.0, rep="medoid"):
    """
    poses: list[(x,y,yaw)]
    cell_size: meters; larger -> fewer clusters
    rep: 'medoid' | 'first'
    Returns:
      reps: list[(x,y,yaw)]  representative poses
      clusters: list[list[int]]  indices of original poses per cluster
      rep_indices: list[int]     index in original poses chosen as rep
    """
    bins = defaultdict(list)
    for i, (x, y, yaw) in enumerate(poses):
        key = (int(math.floor(x / cell_size)), int(math.floor(y / cell_size)))
        bins[key].append(i)

    reps, clusters, rep_indices = [], [], []
    for members in bins.values():
        clusters.append(members)
        if rep == "first" or len(members) == 1:
            ri = members[0]
        else:
            # medoid = member with min sum of distances to others in the cluster
            P = np.array([[poses[i][0], poses[i][1]] for i in members])
            D = np.linalg.norm(P[:, None, :] - P[None, :, :], axis=-1).sum(axis=1)
            ri = members[int(np.argmin(D))]
        rep_indices.append(ri)
        reps.append(poses[ri])
    return reps, clusters, rep_indices


# ------------ 2) Build a simple graph among reps ------------
def build_graph(reps, k=3, r_max=None):
    """
    reps: list[(x,y,yaw)]
    k: connect up to k nearest neighbors
    r_max: max connection distance (meters) or None
    Returns: NetworkX Graph with node attrs x,y,yaw and edge weight w
    """
    P = np.array([[p[0], p[1]] for p in reps])
    N = len(reps)
    G = nx.Graph()
    for i, (x, y, yaw) in enumerate(reps):
        G.add_node(i, x=x, y=y, yaw=yaw)

    for i in range(N):
        d = np.linalg.norm(P - P[i], axis=1)
        order = np.argsort(d)[1:]  # skip self
        added = 0
        for j in order:
            if r_max is not None and d[j] > r_max:
                break
            G.add_edge(i, j, w=float(d[j]))
            added += 1
            if added >= k:
                break
    return G


# (Alternative) minimal spanning graph if you prefer a tree:
def build_mst_graph(reps):
    P = np.array([[p[0], p[1]] for p in reps])
    N = len(reps)
    full = nx.Graph()
    start_id = -1
    min_dist = 1000.0
    ref_pt = np.array([0, 0])
    for i in range(N):
        full.add_node(i, x=reps[i][0], y=reps[i][1], yaw=reps[i][2])
        dist = np.linalg.norm(reps[i][:2] - ref_pt)
        if dist < min_dist:
            min_dist = dist
            start_id = i
    # fully connect by distance (O(N^2)); fine for a few hundred nodes
    for i in range(N):
        for j in range(i + 1, N):
            w = float(np.linalg.norm(P[i] - P[j]))
            # TODO: covisibility???
            if w > 5:
                continue
            full.add_edge(i, j, w=w)
    T = nx.minimum_spanning_tree(full, weight="w")
    return T, start_id


# ------------ 3) (Optional) make a traversal order ------------
def traversal_order_mst(G, start=0):
    """DFS over MST edges to get a simple visiting order."""
    if not nx.is_tree(G):
        G = nx.minimum_spanning_tree(G, weight="w")
    return list(nx.dfs_preorder_nodes(G, source=start))


def set_headings_along_path(reps, order):
    """Orient each rep toward the next node."""
    reps2 = [list(reps[i]) for i in range(len(reps))]
    for k, i in enumerate(order[:-1]):
        j = order[k + 1]
        dx = reps[j][0] - reps[i][0]
        dy = reps[j][1] - reps[i][1]
        reps2[i][2] = math.atan2(dy, dx)
    return [tuple(p) for p in reps2]


def run(poses):
    # poses: your dense grid-sampled poses [(x,y,yaw), ...]
    reps, clusters, rep_ids = cluster_poses_grid(poses, cell_size=3, rep="medoid")

    # Option A: sparse k-NN graph
    # G = build_graph(reps, k=3, r_max=4.0)

    # Option B: a single tree (minimal wires)
    G, start_id = build_mst_graph(reps)
    if start_id < 0:
        start_id = 0
    print(start_id)

    # Make a simple route (order of visiting)
    order = traversal_order_mst(G, start=start_id)

    # Point headings along the route (nice for sim scans)
    reps_oriented = set_headings_along_path(reps, order)

    # If your simulator expects a sequence to publish:
    poses_to_play = [reps_oriented[i] for i in order]
    return poses_to_play
