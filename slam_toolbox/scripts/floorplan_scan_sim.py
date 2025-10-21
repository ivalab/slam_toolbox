#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file floorplan_scan_sim.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 10-20-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""

import math
import numpy as np
import rospy
import tf
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TransformStamped, PoseArray
from sensor_msgs.msg import LaserScan

from generate_route_graph import run

# -------------------------
# Helpers
# -------------------------


def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def world_to_map(grid, xw, yw):
    res = grid.info.resolution
    mx = int((xw - grid.info.origin.position.x) / res)
    my = int((yw - grid.info.origin.position.y) / res)
    if 0 <= mx < grid.info.width and 0 <= my < grid.info.height:
        return mx, my
    return None


def is_free_with_clearance(grid, xw, yw, clearance_m, free_val=0):
    res = grid.info.resolution
    r = max(0, int(round(clearance_m / res)))
    mp = world_to_map(grid, xw, yw)
    if mp is None:
        return False
    mx, my = mp
    W, H = grid.info.width, grid.info.height
    data = grid.data
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            nx, ny = mx + dx, my + dy
            if 0 <= nx < W and 0 <= ny < H:
                # if data[ny * W + nx] != free_val:  # Free [0], Unknown [-1], OCC [100]
                if data[ny * W + nx] > 10:
                    return False
            else:
                return False
    return True


def sample_grid_poses(grid, stride_m, clearance_m, init_yaw=0.0, refine_yaw=True):
    res = grid.info.resolution
    xmin = grid.info.origin.position.x
    ymin = grid.info.origin.position.y
    xmax = xmin + grid.info.width * res
    ymax = ymin + grid.info.height * res
    poses = []
    y = ymin + stride_m
    while y < ymax:
        x = xmin + stride_m
        while x < xmax:
            if is_free_with_clearance(grid, x, y, clearance_m):
                yaw = init_yaw
                if refine_yaw and len(poses) > 0:
                    yaw = np.arctan2(y - poses[-1][1], x - poses[-1][0])
                poses.append((x, y, yaw))
            x += stride_m
        y += stride_m
    return poses


def make_pose_array_msg(frame_id, poses):
    arr = PoseArray()
    arr.header.frame_id = frame_id
    arr.header.stamp = rospy.Time.now()
    for i, (x, y, yaw) in enumerate(poses):
        ps = Pose()
        ps.position.x = x
        ps.position.y = y
        ps.orientation = yaw_to_quat(yaw)
        arr.poses.append(ps)
    return arr


def make_path_msg(frame_id, poses):
    msg = Path()
    msg.header.frame_id = frame_id
    t0 = rospy.Time.now()
    msg.header.stamp = t0
    for i, (x, y, yaw) in enumerate(poses):
        ps = PoseStamped()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = yaw_to_quat(yaw)
        ps.header.stamp = t0 + rospy.Duration(i * 0.1)  # 10hz???
        msg.poses.append(ps)
    return msg


def bresenham(p0, p1):
    (y0, x0), (y1, x1) = p0, p1
    dy, dx = abs(y1 - y0), abs(x1 - x0)
    sy, sx = (1 if y0 < y1 else -1), (1 if x0 < x1 else -1)
    err = dx + dy
    y, x = y0, x0
    while True:
        yield (y, x)
        if y == y1 and x == x1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def simulate_scan(grid, x, y, yaw, angle_min, angle_max, angle_inc, range_min, range_max):
    """Raycast on OccupancyGrid. Returns LaserScan ranges array."""
    W = grid.info.width
    H = grid.info.height
    res = grid.info.resolution
    origin_x = grid.info.origin.position.x
    origin_y = grid.info.origin.position.y
    data = grid.data

    angles = np.arange(angle_min, angle_max + 1e-9, angle_inc, dtype=np.float32)
    ranges = np.empty_like(angles, dtype=np.float32)
    # start cell
    start = world_to_map(grid, x, y)
    if start is None:
        ranges.fill(np.inf)
        return ranges

    y0, x0 = start[1], start[0]  # (row, col)
    for k, a in enumerate(angles):
        th = yaw + a
        ex = x + range_max * math.cos(th)
        ey = y + range_max * math.sin(th)
        end = world_to_map(grid, ex, ey)
        if end is None:
            # cast to map edge
            # approximate by clipping to map bounds
            gx = max(0, min(W - 1, int(round((ex - origin_x) / res))))
            gy = max(0, min(H - 1, int(round((ey - origin_y) / res))))
            end = (gx, gy)
        y1, x1 = end[1], end[0]

        hit = np.inf
        for gy, gx in bresenham((y0, x0), (y1, x1)):
            if gy < 0 or gy >= H or gx < 0 or gx >= W:
                break
            # occupied = False
            # for gy_step in range(-1, 2):
            #     for gx_step in range(-1, 2):
            #         gyv = gy + gy_step
            #         gxv = gx + gx_step
            #         if gyv < 0 or gyv >= H or gxv < 0 or gxv >= W:
            #             continue
            v = data[gy * W + gx]
            if v > 10:  # occupied
                # occupied = True
                # break
                # if occupied:
                # break
                # if occupied:
                wx = origin_x + (gx + 0.5) * res
                wy = origin_y + (gy + 0.5) * res
                d = math.hypot(wx - x, wy - y)
                # if d >= range_min:
                # hit = min(d, range_max)
                if d > range_max:
                    hit = np.inf
                elif d < range_min:
                    hit = np.nan
                else:
                    hit = d
                break
        ranges[k] = hit
    return ranges


def simulate_scan2(grid, x, y, yaw, angle_min, angle_max, angle_inc, range_min, range_max):
    """Raycast on OccupancyGrid. Returns LaserScan ranges array."""
    W = grid.info.width
    H = grid.info.height
    res = grid.info.resolution
    origin_x = grid.info.origin.position.x
    origin_y = grid.info.origin.position.y
    data = grid.data

    angles = np.arange(angle_min, angle_max + 1e-9, angle_inc, dtype=np.float32)
    ranges = np.empty_like(angles, dtype=np.float32)
    # start cell
    start = world_to_map(grid, x, y)
    if start is None:
        ranges.fill(np.inf)
        return ranges
    y0, x0 = start[1], start[0]  # (row, col)
    for k, a in enumerate(angles):
        th = yaw + a
        hit = np.inf
        for dist in np.arange(range_min, range_max + 1e-9, res):
            ex = x + dist * math.cos(th)
            ey = y + dist * math.sin(th)
            pt = world_to_map(grid, ex, ey)
            if pt is None:
                break
            if pt[1] < 0 or pt[1] >= H or pt[0] < 0 or pt[0] >= W:
                break
            v = data[pt[1] * W + pt[0]]
            if v > 10:  # occupied
                # if v != 0:  # occupied
                hit = dist
                break
        ranges[k] = hit
    return ranges


def main():
    rospy.init_node("floorplan_scan_sim")

    # Frames & topics
    # map_frame = rospy.get_param("~map_frame", "map")
    odom_frame = rospy.get_param("~odom_frame", "odom")
    base_frame = rospy.get_param("~base_frame", "base_footprint")
    laser_frame = rospy.get_param("~laser_frame", "laser_frame")
    scan_topic = rospy.get_param("~scan_topic", "/scan")
    odom_topic = rospy.get_param("~odom_topic", "/odom")
    pose_array_topic = rospy.get_param("~pose_array_topic", "/pose_array")
    path_topic = rospy.get_param("~pose_graph_topic", "/pose_graph")

    # Lidar params (your defaults)
    fov_deg = rospy.get_param("~fov_deg", 360.0)
    ang_res_deg = rospy.get_param("~angular_resolution_deg", 0.12)
    max_range = rospy.get_param("~range_max", 30.0)
    min_range = rospy.get_param("~range_min", 0.05)
    scan_rate = rospy.get_param("~scan_rate_hz", 10.0)

    # Pose sequence generation
    stride_m = rospy.get_param("~grid_stride_m", 0.5)
    clearance_m = rospy.get_param("~clearance_m", 0.50)
    fixed_yaw_deg = rospy.get_param("~fixed_yaw_deg", 0.0)
    loop_through = rospy.get_param("~loop_through", True)  # loop over poses

    # Optionally load poses from YAML/JSON (list of [x,y,yaw]) instead of sampling
    load_poses_path = rospy.get_param("~load_poses", "")

    # Wait for map
    rospy.loginfo("Waiting for /map ...")
    grid = rospy.wait_for_message("/map", OccupancyGrid)
    rospy.loginfo("Got /map: %dx%d @ %.3fm/px", grid.info.width, grid.info.height, grid.info.resolution)

    # Build pose list
    if load_poses_path:
        import yaml, json, os

        if load_poses_path.endswith((".yaml", ".yml")):
            with open(load_poses_path, "r") as f:
                poses = yaml.safe_load(f)
        else:
            with open(load_poses_path, "r") as f:
                poses = json.load(f)
        poses = [(float(p[0]), float(p[1]), float(p[2])) for p in poses]
        rospy.loginfo("Loaded %d poses from %s", len(poses), load_poses_path)
    else:
        raw_poses = sample_grid_poses(
            grid, stride_m=stride_m, clearance_m=clearance_m, init_yaw=math.radians(fixed_yaw_deg), refine_yaw=False
        )
        poses = run(raw_poses)
        # poses = raw_poses
        rospy.loginfo("Sampled %d poses (stride=%.2fm, clearance=%.2fm).", len(poses), stride_m, clearance_m)

    if not poses:
        rospy.logerr("No poses found. Adjust stride/clearance or provide ~load_poses.")
        return

    # Publish full path for visualization (in odom frame; we’ll publish TF map->odom identity for simplicity)
    path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
    path_msg = make_path_msg(odom_frame, poses)
    path_pub.publish(path_msg)
    rospy.loginfo("Published /gt_path with %d poses.", len(poses))
    pose_array_pub = rospy.Publisher(pose_array_topic, PoseArray, queue_size=1, latch=True)
    pose_array_msg = make_pose_array_msg(odom_frame, poses)
    pose_array_pub.publish(pose_array_msg)

    pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=1)

    # Publishers
    scan_pub = rospy.Publisher(scan_topic, LaserScan, queue_size=1)
    odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=1)
    br = tf.TransformBroadcaster()
    static_br = tf2_ros.StaticTransformBroadcaster()

    # Static TF: base_footprint -> laser_frame = identity
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = base_frame
    t.child_frame_id = laser_frame
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
    static_br.sendTransform(t)

    # Also publish map->odom as identity (so slam_toolbox can work in map; optional)
    # t2 = TransformStamped()
    # t2.header.stamp = rospy.Time.now()
    # t2.header.frame_id = map_frame
    # t2.child_frame_id = odom_frame
    # t2.transform.translation.x = 0.0
    # t2.transform.translation.y = 0.0
    # t2.transform.translation.z = 0.0
    # t2.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
    # static_br.sendTransform(t2)

    # Laser angles
    angle_min = -math.radians(fov_deg) / 2.0
    angle_max = math.radians(fov_deg) / 2.0
    angle_inc = math.radians(ang_res_deg)
    beam_count = int(round((angle_max - angle_min) / angle_inc)) + 1
    rospy.loginfo(
        "Laser: FOV=%.1f deg, res=%.3f deg -> %d beams, range=[%.2f, %.2f]m, rate=%.1f Hz",
        fov_deg,
        ang_res_deg,
        beam_count,
        min_range,
        max_range,
        scan_rate,
    )

    rate = rospy.Rate(scan_rate)
    idx = 0
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        x, y, yaw = poses[idx]

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = odom_frame
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation = yaw_to_quat(yaw)
        pose_pub.publish(pose_msg)

        # 1) Simulate scan at current pose
        ranges = simulate_scan2(grid, x, y, yaw, angle_min, angle_max, angle_inc, min_range, max_range)

        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = laser_frame
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / scan_rate
        scan.range_min = min_range
        scan.range_max = max_range
        scan.ranges = ranges.tolist()
        scan.intensities = []

        scan_pub.publish(scan)

        # 2) Publish odom (pose in odom) and TF odom->base
        odom = Odometry()
        odom.header.stamp = scan.header.stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = yaw_to_quat(yaw)
        odom_pub.publish(odom)

        # TF: odom -> base_footprint
        br.sendTransform(
            (x, y, 0.0), (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)), scan.header.stamp, base_frame, odom_frame
        )

        # advance pose index
        idx += 1
        if idx >= len(poses):
            if loop_through:
                idx = 0
            else:
                rospy.loginfo("Finished publishing all poses.")
                break

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
