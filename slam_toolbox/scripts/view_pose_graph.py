#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file view_pose_graph.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 10-13-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""


import json, csv, networkx as nx
import matplotlib.pyplot as plt
from typing import Optional
import numpy as np
from sklearn.cluster import DBSCAN
import math
from collections import defaultdict


def attach_images_to_nodes(G, id2stamp, stamp2id, image_csv, dt=0.5):
    im_stamps = np.loadtxt(image_csv, ndmin=2, dtype="str")
    node_stamps = np.array(list(id2stamp.values()))
    node_ids = np.array(list(id2stamp.keys()))
    for name, stamp_str in im_stamps:
        stamp = float(stamp_str)
        timediff = np.abs(node_stamps - stamp)
        minval = np.min(timediff)
        if minval > dt:
            continue
        idx = np.argmin(timediff)
        nid = node_ids[idx]
        G.nodes[nid]["images"].append((name, stamp))


def load_posegraph_json(json_path, images_csv=None):
    with open(json_path, "r") as f:
        J = json.load(f)

    id2stamp = {}
    stamp2id = {}

    G = nx.Graph()
    for n in J["nodes"]:
        G.add_node(
            n["id"],
            x=n["x"],
            y=n["y"],
            theta=n["theta"],
            stamp=n.get("stamp", None),
            fixed=n.get("fixed", False),
            images=[],
        )

        id2stamp[n["id"]] = n["stamp"]
        stamp2id[n["stamp"]] = n["id"]

    it = iter(J["edges"])
    for e in it:
        i, j = e["i"], e["j"]
        G.add_edge(
            i, j, type=e.get("type", "odom"), obs=tuple(e["obs"]), cov=tuple(e["cov"])  # (dx,dy,dtheta)
        )  # 9 numbers row-major
    if images_csv:
        attach_images_to_nodes(G, id2stamp, stamp2id, images_csv)

    return G, id2stamp, stamp2id


# def _node_xy(node):
#     # Support either (x,y,theta) or T_wc 4x4
#     if "x" in node and "y" in node:
#         return float(node["x"]), float(node["y"])
#     if "T_wc" in node:
#         T = node["T_wc"]
#         # assume [[...],[...],[...],[...]] row-major
#         return float(T[0][3]), float(T[1][3])
#     raise KeyError("Node missing (x,y) or T_wc.")


def plot_posegraph_2d(G, show_images_count: bool = False):
    nodes = {nid: (G.nodes[nid]["x"], G.nodes[nid]["y"]) for nid in G.nodes()}
    edges = [(i, j) for i, j in G.edges()]

    plt.figure()
    # Draw edges
    for i, j in edges:
        x1, y1 = nodes[i]
        x2, y2 = nodes[j]
        plt.plot([x1, x2], [y1, y2], linewidth=2.0, color="blue")

    # Draw nodes
    xys = np.array([[node[0], node[1]] for node in nodes.values()])
    plt.plot(xys[:, 0], xys[:, 1], "o", markersize=5, color="red")
    if show_images_count:
        for nid in G.nodes:
            k = len(G.nodes[nid]["images"])
            if k > 0:
                x, y = nodes[nid]
                plt.text(x, y, str(k), fontsize=15)

    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Pose Graph (XY)")
    plt.tight_layout()
    plt.show()


def _get_xy(G, n):
    nd = G.nodes[n]
    # if "x" in nd and "y" in nd:
    return float(nd["x"]), float(nd["y"])
    # if "T_wc" in nd:  # fallback
    # T = nd["T_wc"]
    # return float(T[0][3]), float(T[1][3])
    # raise KeyError(f"Node {n} missing (x,y) or T_wc")


def _choose_rep(G, members, strategy="medoid"):
    """Return one member node id as cluster representative (original pose node)."""
    if strategy == "max_degree":
        return max(members, key=lambda n: G.degree(n))
    if strategy == "earliest":
        # fallback to medoid if no stamps
        with_stamps = [n for n in members if "stamp" in G.nodes[n] and G.nodes[n]["stamp"] is not None]
        if with_stamps:
            return min(with_stamps, key=lambda n: float(G.nodes[n]["stamp"]))
        # else medoid
    # "medoid": closest to cluster mean (x,y)
    P = np.array([_get_xy(G, n) for n in members])
    mu = P.mean(axis=0)
    d2 = ((P - mu) ** 2).sum(axis=1)
    return members[int(np.argmin(d2))]


def build_navigation_graph_with_reps(
    G_pose: nx.Graph,
    method: str = "dbscan",  # "dbscan" or "grid"
    eps: float = 1.0,
    min_samples: int = 3,
    grid_size: Optional[float] = None,
    min_cluster_size: int = 5,
    connect_via: str = "edges",  # "edges" or "proximity"
    proximity_thresh: float = 2.0,
    rep_strategy: str = "medoid",  # "medoid" | "max_degree" | "earliest"
):
    """
    Build a sparse navigation graph whose nodes are ORIGINAL POSE NODE IDs
    (one representative per cluster).
    Returns:
      NavG: nx.Graph (nodes = representative pose IDs)
      mapping: dict pose_node -> cluster_id (int)
      reps: dict cluster_id -> representative pose node id
      clusters: dict cluster_id -> list of member pose node ids
    """
    nodes = list(G_pose.nodes())
    if not nodes:
        raise ValueError("Empty pose-graph.")

    # --- 1) Cluster pose nodes
    if method == "dbscan":
        P = np.array([_get_xy(G_pose, n) for n in nodes])
        labels = DBSCAN(eps=eps, min_samples=min_samples).fit(P).labels_
    elif method == "grid":
        if not grid_size or grid_size <= 0:
            raise ValueError("grid_size must be > 0 for method='grid'.")
        labels, bins = [], {}
        for n in nodes:
            x, y = _get_xy(G_pose, n)
            key = (int(math.floor(x / grid_size)), int(math.floor(y / grid_size)))
            if key not in bins:
                bins[key] = len(bins)
            labels.append(bins[key])
        labels = np.array(labels, dtype=int)
    else:
        raise ValueError("method must be 'dbscan' or 'grid'")

    cluster_members = defaultdict(list)
    for n, c in zip(nodes, labels):
        if c == -1:  # DBSCAN noise → own micro cluster
            c = f"noise_{n}"
        cluster_members[c].append(n)

    # prune tiny clusters if requested
    clusters = {cid: m for cid, m in cluster_members.items() if len(m) >= min_cluster_size} or cluster_members

    # compact integer cluster ids
    old_to_new = {old: k for k, old in enumerate(clusters.keys())}
    mapping = {}
    for old, members in clusters.items():
        for n in members:
            mapping[n] = old_to_new[old]

    # --- 2) Pick representative original node per cluster
    reps = {}
    for old, members in clusters.items():
        rep = _choose_rep(G_pose, members, strategy=rep_strategy)
        reps[old_to_new[old]] = rep

    # --- 3) Build Nav graph with rep nodes
    NavG = nx.Graph()
    # add nav nodes = representative pose ids with some attributes copied
    for cid, rep in reps.items():
        x, y = _get_xy(G_pose, rep)[0], _get_xy(G_pose, rep)[1]
        attrs = dict(G_pose.nodes[rep])  # copy pose-node attrs if you want
        # annotate cluster info
        attrs.update(
            {
                "cluster_id": cid,
                "cluster_size": len(clusters[list(clusters.keys())[cid]]),
                "members": clusters[list(clusters.keys())[cid]],
                "x": x,
                "y": y,
            }
        )
        NavG.add_node(rep, **attrs)

    # --- 4) Connect reps
    if connect_via == "edges":
        # connect two rep nodes if any pose-graph edge crosses their clusters
        seen = set()
        for i, j in G_pose.edges():
            if i not in mapping or j not in mapping:
                continue
            ci, cj = mapping[i], mapping[j]
            if ci == cj:
                continue
            ri, rj = reps[ci], reps[cj]
            a = (min(ri, rj), max(ri, rj))
            if a in seen:
                continue
            seen.add(a)
            xi, yi = _get_xy(G_pose, ri)
            xj, yj = _get_xy(G_pose, rj)
            w = float(math.hypot(xi - xj, yi - yj))
            NavG.add_edge(ri, rj, weight=w, via="pose_edges")
    elif connect_via == "proximity":
        rep_ids = list(NavG.nodes())
        R = np.array([_get_xy(G_pose, r) for r in rep_ids])
        for a in range(len(rep_ids)):
            for b in range(a + 1, len(rep_ids)):
                d = float(np.linalg.norm(R[a] - R[b]))
                if d <= proximity_thresh:
                    NavG.add_edge(rep_ids[a], rep_ids[b], weight=d, via="proximity")
    else:
        raise ValueError("connect_via must be 'edges' or 'proximity'")

    return NavG, mapping, reps, clusters


def plot_nav_graph_reps(G_pose: nx.Graph, NavG: nx.Graph, alpha_pose=0.25, show_images_count: bool = False):
    xs = [G_pose.nodes[n]["x"] for n in G_pose.nodes() if "x" in G_pose.nodes[n]]
    ys = [G_pose.nodes[n]["y"] for n in G_pose.nodes() if "y" in G_pose.nodes[n]]
    plt.figure()
    if xs and ys:
        plt.scatter(xs, ys, s=4, alpha=alpha_pose, label="pose nodes")

    # nav reps
    rx = [NavG.nodes[r]["x"] for r in NavG.nodes()]
    ry = [NavG.nodes[r]["y"] for r in NavG.nodes()]
    plt.scatter(rx, ry, s=40, label="nav nodes")

    # nav edges (between reps)
    for u, v, d in NavG.edges(data=True):
        x1, y1 = NavG.nodes[u]["x"], NavG.nodes[u]["y"]
        x2, y2 = NavG.nodes[v]["x"], NavG.nodes[v]["y"]
        plt.plot([x1, x2], [y1, y2], linewidth=1.6, alpha=0.9)

    # Draw image count
    if show_images_count:
        for nid in NavG.nodes():
            k = len(NavG.nodes[nid]["images"])
            if k > 0:
                x, y = _get_xy(NavG, nid)
                plt.text(x, y, str(k), fontsize=15)

    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Navigation Graph")
    plt.legend()
    plt.tight_layout()
    plt.show()


def main():
    pg_filename = "/home/yanwei/Desktop/mapping-pg/real_3/slam_toolbox_posegraph.json"
    im_filename = "/home/yanwei/Desktop/mapping-pg/real_3/images.txt"
    G, _, _ = load_posegraph_json(pg_filename, images_csv=im_filename)
    plot_posegraph_2d(G, show_images_count=True)
    NavG = build_navigation_graph_with_reps(G, method="grid", grid_size=3.0, connect_via="edges", min_cluster_size=0)
    plot_nav_graph_reps(G, NavG[0], show_images_count=False)


if __name__ == "__main__":
    main()
