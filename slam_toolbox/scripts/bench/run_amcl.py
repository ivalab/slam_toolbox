#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file run_amcl.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 12-07-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""


import subprocess
import glob
from pathlib import Path
import os
import time

DATA_ROOT = "/mnt/IVALAB/rosbags/tsrb/GW_CL_SEQS"
# RESULT_ROOT = DATA_ROOT
RESULT_ROOT = "/tmp"
SEQUENCES = [
    # "20241012",
    # "20250330",
    # "20250331",
    # "20250530",
    # "20250619",
    # "new/msf/two_loops",
    # "new/msf/one_big_loop",
    # "new/msf/inspection",
    # "cl_path/path1",
    "20250912_1",
    # "20250912_2",
    # "20250912_3",
    # "cl_path/path3",
    # "floorplan_test",
]

init_poses = {
    "20250912_1": (0.0, 0.0, 1.5707),
    "20250912_2": (-41.070, -11.039, 0.0),
    "20250912_3": (-39.339, -3.549, 1.5707),
}

ROUND = 1
SPEED = 1.0
SAVE_OCC_MAP = True

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
    "/move_base",
    # "/scan",
    "/tf",
]


def remap_topics(topics):
    out = [f"{name}:=/deprecated{name}" for name in topics]
    return " ".join(out)


for seq_index, seq_dir in enumerate(SEQUENCES):

    init_x, init_y, init_a = init_poses[seq_dir]

    # Run SLAM_Toolbox.
    output_dir = os.path.join(RESULT_ROOT, seq_dir, "amcl")
    if not os.path.exists(output_dir):
        Path(output_dir).mkdir(exist_ok=True, parents=True)
    cmd_slam = f"roslaunch slam_toolbox amcl.launch initial_pose_x:={init_x} initial_pose_y:={init_y}  initial_pose_a:={init_a}"
    print(cmd_slam)
    subprocess.Popen(cmd_slam, shell=True)
    time.sleep(5)

    # Run bag.
    bagfiles = sorted(glob.glob(os.path.join(DATA_ROOT, seq_dir, "*.bag")))
    # print(bagfiles)
    bagstr = " ".join(bagfiles)
    cmd_bag = f"rosbag play {bagstr} -r {SPEED} {remap_topics(DEPRECATED_TOPICS)}"  # -u 735
    print(cmd_bag)
    subprocess.call(cmd_bag, shell=True)
    time.sleep(1)

    # Kill SLAM_Toolbox
    for node in [
        "/odom_to_tf",
        "/slam_map_server",
        "/amcl",
        "/slam_toolbox",
        "/rviz",
    ]:
        cmd_kill = f"rosnode kill {node}"
        subprocess.call(cmd_kill, shell=True)
        time.sleep(2)
    time.sleep(5)

    break
