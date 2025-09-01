#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file run_mapping.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 09-01-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""

import subprocess
import glob
from pathlib import Path
import os
import time

DATA_ROOT = "/mnt/IVALAB/rosbags/tsrb/"
SEQUENCES = [
    # "20241012",
    "20250330",
    "20250331",
    "20250530",
    "20250619",
    "new/msf/two_loops",
    "new/msf/one_big_loop",
    "new/msf/inspection",
]

ROUND = 1
SPEED = 0.5

DEPRECATED_TOPICS = [
    "/map",
    "/map_metadata",
    "/map_updates",
    "/slam_toolbox/karto_graph_visualization",
    "/slam_toolbox/odom",
    "/slam_toolbox/pose",
    "/slam_toolbox/update",
    "/slam_toolbox/update_full",
    "/slam_map",
    "/slam_map_updates",
    "/slam/pose",
]


def remap_topics(topics):
    out = [f"{name}:=/deprecated{name}" for name in topics]
    return " ".join(out)


for seq_index, seq_dir in enumerate(SEQUENCES):

    # Run SLAM_Toolbox.
    output_dir = os.path.join(DATA_ROOT, seq_dir, "slam_toolbox")
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    cmd_slam = f"roslaunch slam_toolbox offline.launch output_dir:={output_dir}"
    print(cmd_slam)
    subprocess.Popen(cmd_slam, shell=True)
    time.sleep(5)

    # Run bag.
    bagfiles = sorted(glob.glob(os.path.join(DATA_ROOT, seq_dir, "*.bag")))
    # print(bagfiles)
    bagstr = " ".join(bagfiles)
    cmd_bag = f"rosbag play {bagstr} -r {SPEED} {remap_topics(DEPRECATED_TOPICS)} -u 10.0"
    print(cmd_bag)
    subprocess.call(cmd_bag, shell=True)
    time.sleep(1)

    # Kill SLAM_Toolbox
    for node in ["/slam_toolbox"]:
        cmd_kill = f"rosnode kill {node}"
        subprocess.call(cmd_kill, shell=True)
        time.sleep(2)
    time.sleep(5)

    break
