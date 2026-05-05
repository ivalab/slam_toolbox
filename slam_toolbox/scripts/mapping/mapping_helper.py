#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file mapping_helper.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 05-05-2026
@version 1.0
@license Copyright (c) 2026
@desc Mapping helper for waypoint buffering and map saving on shutdown.
"""

import subprocess
from pathlib import Path

import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped


class MappingHelper(object):
    def __init__(self):
        rospy.init_node("mapping_helper", anonymous=True)

        self._output_dir = Path(rospy.get_param("~output_dir"))
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._map_saver_timeout = float(rospy.get_param("~map_saver_timeout", 30.0))

        self._waypoints = []
        self._path_indices = []
        self._saved = False

        self._goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback, queue_size=10)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("Subscribed to /move_base_simple/goal")
        rospy.loginfo("Output directory: %s", self._output_dir)

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        quat = msg.pose.orientation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(quat_list)

        index = len(self._waypoints)
        self._waypoints.append([index, x, y, yaw])
        self._path_indices.append(index)

        rospy.loginfo("Buffered waypoint %d: x=%.3f y=%.3f yaw=%.3f", index, x, y, yaw)

    def save_waypoints(self):
        if self._waypoints:
            waypoints = np.asarray(self._waypoints, dtype=float)
        else:
            waypoints = np.empty((0, 4), dtype=float)

        waypoints_file = self._output_dir / "waypoints.txt"
        np.savetxt(waypoints_file, waypoints, fmt="%.6f", header="index x y yaw")
        rospy.loginfo("Saved waypoints to %s", waypoints_file)

        path_file = self._output_dir / "path0.txt"
        path_indices = np.asarray(self._path_indices, dtype=int)
        np.savetxt(path_file, path_indices, fmt="%d", header="index")
        rospy.loginfo("Saved path indices to %s", path_file)

    def save_map(self):
        map_prefix = self._output_dir / "map"
        cmd = ["rosrun", "map_server", "map_saver", "-f", str(map_prefix)]
        rospy.loginfo("Running: %s", " ".join(cmd))
        try:
            proc = subprocess.Popen(cmd)
            try:
                returncode = proc.wait(timeout=self._map_saver_timeout)
            except subprocess.TimeoutExpired:
                rospy.logerr("map_saver exceeded %.1f sec, terminating it.", self._map_saver_timeout)
                proc.terminate()
                try:
                    proc.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    rospy.logerr("map_saver did not terminate cleanly, killing it.")
                    proc.kill()
                    proc.wait()
                return

            if returncode != 0:
                rospy.logerr("map_saver failed with return code %d", returncode)
            else:
                rospy.loginfo("Saved occupancy map to %s.*", map_prefix)
        except Exception as exc:
            rospy.logerr("Failed to run map_saver: %s", exc)

    def on_shutdown(self):
        if self._saved:
            return
        self._saved = True
        self.save_waypoints()
        self.save_map()


def main():
    helper = MappingHelper()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
