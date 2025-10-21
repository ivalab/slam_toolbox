#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file floormap_to_scan.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 11-05-2025
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
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TransformStamped, PoseArray, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from bresenham_lidar_sim import simulate_scan_bresenham
from nav_msgs.srv import GetPlan


from graph_explorer import GraphExplorer
from graph_utils import read_graph_from_file

import os


class FloorplanToScan:

    def __init__(self):

        rospy.init_node("floorplan_to_scan")

        self.init_params()

        rospy.wait_for_service("/move_base/make_plan")
        self.make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        # Subscribers
        self.init_pos = None
        self.int_pos_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_pos_cb)

        # Publishers
        self.wpts_pub = rospy.Publisher("/wpts", PoseArray, queue_size=1)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=1)
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()

    def init_params(self):
        self.odom_frame = "odom"
        self.map_frame = "map"
        self.lidar_frame = "laser"
        self.base_frame = "virtual_base_link"

    def init_pos_cb(self, msg):
        """_summary_

        Args:
            msg (_type_): _description_
        """
        self.init_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ]

    def convert_xya_to_pose(self, xya):
        msg = Pose()
        msg.position.x = xya[0]
        msg.position.y = xya[1]
        msg.orientation.w = np.cos(xya[2] / 2.0)
        msg.orientation.z = np.sin(xya[2] / 2.0)
        return msg

    def convert_wpts_to_path_msg(self, wpts):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()
        for xya in wpts:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = self.convert_xya_to_pose(xya)
            msg.poses.append(pose_stamped)
        return msg

    def convert_vertices_to_wpts(self, vertices):
        wpts = [vertices[0]]
        for i in range(1, len(vertices)):
            x0, y0, a0 = vertices[i - 1]
            x1, y1, a1 = vertices[i]
            yaw = np.arctan2(y1 - y0, x1 - x0)
            wpts.append([x1, y1, yaw])
        return wpts

    def convert_to_pose_array_msg(self, wpts):
        arr = PoseArray()
        arr.header.stamp = rospy.Time.now()
        arr.header.frame_id = "map"
        for xya in wpts:
            arr.poses.append(self.convert_xya_to_pose(xya))
        return arr

    def yaw_to_quat(self, yaw):
        return Quaternion(0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

    def world_to_map(self, grid, xw, yw):
        """No boundary check.

        Args:
            grid (_type_): _description_
            xw (_type_): _description_
            yw (_type_): _description_

        Returns:
            _type_: _description_
        """
        res = grid.info.resolution
        mx = int((xw - grid.info.origin.position.x) / res)
        my = int((yw - grid.info.origin.position.y) / res)
        return mx, my

    def is_occupied(self, grid, mx, my):
        """Check if a map cell is occupied (obstacle)"""
        if mx < 0 or mx >= grid.info.width or my < 0 or my >= grid.info.height:
            return True  # Out of bounds treated as occupied

        # OccupancyGrid: -1=unknown, 0=free, 100=occupied
        # Treat unknown and occupied as obstacles
        # cell_value = grid.data[my, mx]
        cell_value = grid.data[my * grid.info.width + mx]
        return cell_value > 10 or cell_value == -1

    def raycast(self, x, y, angle, grid, range_min, range_max):
        """
        Perform raycasting from position (x, y) at given angle
        Returns the distance to the first obstacle or max_range
        """
        step_size = grid.info.resolution * 0.5  # Step in world coordinates

        for distance in np.arange(range_min, range_max, step_size):
            # Calculate point along the ray
            ray_x = x + distance * math.cos(angle)
            ray_y = y + distance * math.sin(angle)

            # Convert to map coordinates
            mx, my = self.world_to_map(grid, ray_x, ray_y)
            # Check if occupied
            if self.is_occupied(grid, mx, my):
                return distance

        return np.inf

    def generate_scan(self, grid, x, y, yaw, angle_min, angle_max, angle_inc, range_min, range_max):
        # Generate ranges by raycasting
        angles = np.arange(angle_min, angle_max + 1e-9, angle_inc, dtype=np.float32)
        ranges = np.empty_like(angles, dtype=np.float32)
        for k, a in enumerate(angles):
            angle = yaw + a
            distance = self.raycast(x, y, angle, grid, range_min, range_max)
            ranges[k] = distance
        return ranges

    def bresenham(self, p0, p1):
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

    def simulate_scan(self, grid, x, y, yaw, angle_min, angle_max, angle_inc, range_min, range_max):
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
        start = self.world_to_map(grid, x, y)
        if start is None:
            ranges.fill(np.inf)
            return ranges

        y0, x0 = start[1], start[0]  # (row, col)
        for k, a in enumerate(angles):
            th = yaw + a
            ex = x + range_max * math.cos(th)
            ey = y + range_max * math.sin(th)
            end = self.world_to_map(grid, ex, ey)
            if end is None:
                continue
                # cast to map edge
                # approximate by clipping to map bounds
                # gx = max(0, min(W - 1, int(round((ex - origin_x) / res))))
                # gy = max(0, min(H - 1, int(round((ey - origin_y) / res))))
                # end = (gx, gy)
                # gx = end[0]
            y1, x1 = end[1], end[0]

            hit = np.inf
            for gy, gx in self.bresenham((y0, x0), (y1, x1)):
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

    def densify_vertices(self, vertices, tolerance=0.05, min_dist=1.0):
        init = []
        num = len(vertices)
        for i in range(num - 1):
            start = PoseStamped()
            start.header.stamp = rospy.Time.now()
            start.header.frame_id = self.map_frame
            start.pose.position.x = vertices[i][0]
            start.pose.position.y = vertices[i][1]
            start.pose.orientation.w = 1.0

            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self.map_frame
            goal.pose.position.x = vertices[i + 1][0]
            goal.pose.position.y = vertices[i + 1][1]
            goal.pose.orientation.w = 1.0
            req = dict(start=start, goal=goal, tolerance=tolerance)
            resp = self.make_plan(**req)

            if len(resp.plan.poses) == 0:
                rospy.logerr("Global planner returned an empty path. Check map, footprint, and planner params.")
                continue

            for p in resp.plan.poses:
                x0 = p.pose.position.x
                y0 = p.pose.position.y
                init.append([x0, y0, 0.0])

        num = len(init)
        last = init[0]
        out = [last]
        for i in range(1, num):
            cur = init[i]
            dist = np.linalg.norm(np.array(cur) - np.array(last))
            if dist < min_dist:
                continue
            out.append(cur)
            last = cur
        out.append(init[-1])
        return out

    def run(self):

        # Wait for map
        rospy.loginfo("Waiting for /map ...")
        grid = rospy.wait_for_message("/map", OccupancyGrid)
        rospy.loginfo("Got /map: %dx%d @ %.3fm/px", grid.info.width, grid.info.height, grid.info.resolution)

        # Load route graph
        route_graph_filename = rospy.get_param("~route_graph", "")
        if len(route_graph_filename) == 0 or not os.path.exists(route_graph_filename):
            rospy.logerr("Route graph file does NOT exist!")
            return

        # Waiting for the user to input the robot init pos
        rospy.loginfo("Waiting for user click robot init pose ... ")
        while not rospy.is_shutdown() and self.init_pos is None:
            rospy.sleep(1)

        if self.init_pos is None:
            rospy.logerr("Robot init pose is NOT defined!")
            return

        # Process the route graph to get path
        rospy.loginfo("Loading route graph ...")
        G = read_graph_from_file(route_graph_filename)
        explorer = GraphExplorer()
        rospy.loginfo(f"User defined init position: {self.init_pos[0], self.init_pos[1]}")
        init_vertices = explorer.set_graph(G, G.nodes(), self.init_pos)
        if len(init_vertices) < 1:
            rospy.logerr("No feasible vertices are Found from the Route Graph!!!")
            return
        # Compute wpts from the vertices.
        rospy.loginfo(f"Init vertices num = {len(init_vertices)}")
        rospy.loginfo("Densifying the vertices ...")
        vertices = self.densify_vertices(init_vertices)
        rospy.loginfo(f"Final vertices num = {len(vertices)}")
        wpts = self.convert_vertices_to_wpts(vertices)
        path_msg = self.convert_wpts_to_path_msg(wpts)
        self.path_pub.publish(path_msg)

        # Publish static tf
        # Static TF: base_footprint -> laser_frame = identity
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.lidar_frame
        # Turtlebot params.
        t.transform.translation.x = 0.04
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.4
        t.transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.static_br.sendTransform(t)

        # Load lidar params
        fov_deg = rospy.get_param("~fov_deg", 360.0)
        ang_res_deg = rospy.get_param("~angular_resolution_deg", 0.12)
        range_max = rospy.get_param("~range_max", 30.0)
        range_min = rospy.get_param("~range_min", 0.05)
        scan_rate = rospy.get_param("~scan_rate", 10.0)
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
            range_min,
            range_max,
            scan_rate,
        )
        loop_through = rospy.get_param("~loop_through", True)
        loop_num = 2
        loop = 0
        rate = rospy.Rate(scan_rate)
        idx = 0
        rospy.loginfo("Publising the waypoints ... ")
        while not rospy.is_shutdown():
            x, y, yaw = wpts[idx]
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.map_frame
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.orientation = self.yaw_to_quat(yaw)
            self.pose_pub.publish(pose_msg)

            rospy.loginfo(f"Processing {idx} out of {len(wpts)} ... ")

            # 1) Simulate scan at current pose
            ranges = self.simulate_scan(grid, x, y, yaw, angle_min, angle_max, angle_inc, range_min, range_max)
            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.header.frame_id = self.lidar_frame
            scan.angle_min = angle_min
            scan.angle_max = angle_max
            scan.angle_increment = angle_inc
            scan.time_increment = 0.0
            scan.scan_time = 1.0 / scan_rate
            scan.range_min = range_min
            scan.range_max = range_max
            scan.ranges = ranges.tolist()
            scan.intensities = []

            self.scan_pub.publish(scan)

            # 2) Publish odom (pose in odom) and TF odom->base
            odom = Odometry()
            odom.header.stamp = scan.header.stamp
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation = self.yaw_to_quat(yaw)
            self.odom_pub.publish(odom)

            # TF: odom -> base_footprint
            self.br.sendTransform(
                (x, y, 0.0),
                (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)),
                scan.header.stamp,
                self.base_frame,
                self.odom_frame,
            )

            # advance pose index
            idx += 1
            if idx >= len(wpts):
                loop += 1
                if loop_through:
                    if loop >= loop_num:
                        break
                    else:
                        idx = 0
                else:
                    rospy.loginfo("Finished publishing all poses.")
                    break

            rate.sleep()


def main():

    fs = FloorplanToScan()
    fs.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
