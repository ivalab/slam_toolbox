#!/usr/bin/env python3
"""
Extract inspection waypoints from overhead AprilTag detections and SLAM Pose-Graph.

Features:
- Load keyframes from TUM format or subscribe to a PoseArray topic (KeyFrame).
- Subscribe to image color and camera_info topics and detect AprilTags.
- Save each tag detection (timestamp, tag id, four corners, center).
- Cluster detections within a time window (default 5s) and keep the
  detection closest to image center per cluster.
- Match filtered detections to nearest keyframe by timestamp within
  a threshold and save final inspection points in (x,y,theta) and TUM format.
- Optional interpolation of keyframes to a target rate (e.g., 30Hz).

Dependencies: numpy, rospy, cv_bridge, message_filters, pupil_apriltags (or apriltag), scipy (optional)

Usage (ROS node):
  rosrun <pkg> extract_inspection_waypoints.py --image_topic /camera/image_raw \
      --camera_info /camera/camera_info --tum_file /path/to/keyframes.tum --output_dir ./out

"""
import argparse
import csv
import math
import os
from pathlib import Path
import sys
import threading
import time
from collections import defaultdict

import numpy as np

try:
    import rospy
    from sensor_msgs.msg import Image, CameraInfo
    from sensor_msgs.msg import CompressedImage
    from geometry_msgs.msg import PoseArray
    import message_filters
    from cv_bridge import CvBridge
except Exception:
    rospy = None

import cv2
try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None

try:
    # prefer pupil_apriltags if available
    from pupil_apriltags import Detector as AprilTagDetector
except Exception:
    try:
        import apriltag

        class _LegacyDetector:
            def __init__(self, family='tag36h11'):
                self.detector = apriltag.Detector()

            def detect(self, img_gray):
                return self.detector.detect(img_gray)

        AprilTagDetector = _LegacyDetector
    except Exception:
        AprilTagDetector = None

try:
    from scipy import interpolate
except Exception:
    interpolate = None


def quaternion_to_yaw(qx, qy, qz, qw):
    # yaw from quaternion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ExtractionNode(object):
    def __init__(self, args):
        self.args = args
        self.args.output_dir = os.path.join(self.args.output_dir, (Path(self.args.tum_file).parent.name))
        self.bridge = CvBridge() if rospy else None
        self.detector = AprilTagDetector(family=args.apriltag_family) if AprilTagDetector else None

        self.lock = threading.Lock()
        self.detections = []  # list of dicts: {ts, tag_id, corners:[(x,y)*4], center:(x,y), dist_to_center}

        # keyframes as list of dict: {ts, tx, ty, tz, qx,qy,qz,qw}
        self.keyframes = []

        if args.tum_file:
            self.load_tum(args.tum_file)

        self.posearray_buffer = []

        if rospy:
            rospy.init_node('extract_inspection_waypoints', anonymous=True)

            # subscribe camera topics (support compressed or raw image types)
            img_type = Image if args.raw_image else CompressedImage
            img_sub = message_filters.Subscriber(args.image_topic, img_type)
            caminfo_sub = message_filters.Subscriber(args.camera_info, CameraInfo)
            ts = message_filters.ApproximateTimeSynchronizer([img_sub, caminfo_sub], 10, 0.1)
            ts.registerCallback(self.image_cb)

            # if args.pose_array_topic:
                # rospy.Subscriber(args.pose_array_topic, PoseArray, self.posearray_cb)

            rospy.loginfo('extract_inspection_waypoints node started')
        else:
            print('ROS not available; this script can still process images if adapted.')

    def load_tum(self, path):
        data = np.loadtxt(path, ndmin=2)
        # TUM format: timestamp tx ty tz qx qy qz qw (or similar)
        if data.ndim == 1:
            data = data[np.newaxis, :]
        for row in data:
            ts = float(row[0])
            tx, ty, tz = map(float, row[1:4])
            qx, qy, qz, qw = map(float, row[4:8])
            self.keyframes.append({'ts': ts, 'tx': tx, 'ty': ty, 'tz': tz, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw})

    # def posearray_cb(self, msg):
    #     # store PoseArray with header stamp; associate each pose with the same timestamp
    #     with self.lock:
    #         ts = msg.header.stamp.to_sec()
    #         for p in msg.poses:
    #             # convert quaternion and position
    #             tx = p.position.x
    #             ty = p.position.y
    #             tz = p.position.z
    #             qx = p.orientation.x
    #             qy = p.orientation.y
    #             qz = p.orientation.z
    #             qw = p.orientation.w
    #             self.keyframes.append({'ts': ts, 'tx': tx, 'ty': ty, 'tz': tz, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw})

    def image_cb(self, img_msg, caminfo_msg):
        if self.detector is None:
            rospy.logerr('No AprilTag detector available; please install pupil_apriltags or apriltag python package')
            return

        try:
            if isinstance(img_msg, CompressedImage) or self.args.image_compressed:
                # use CvBridge if it supports compressed conversion, else decode with OpenCV
                try:
                    cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
                except Exception:
                    arr = np.frombuffer(img_msg.data, dtype=np.uint8)
                    cv_image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr('CvBridge conversion failed: %s', e)
            return

        gray = cv_image[:, :, 0] if cv_image.ndim == 3 else cv_image

        tags = self.detector.detect(gray)

        h, w = gray.shape[:2]
        img_center = (w / 2.0, h / 2.0)
        ts = img_msg.header.stamp.to_sec()

        # with self.lock:
        for t in tags:
            # pupil_apriltags returns dict-like with 'id','corners','center'
            try:
                tag_id = int(t['id'])
                corners = [(float(c[0]), float(c[1])) for c in t['corners']]
                center = (float(t['center'][0]), float(t['center'][1]))
            except Exception:
                # legacy apriltag python bindings
                tag_id = int(t.tag_id)
                corners = [(float(c[0]), float(c[1])) for c in t.corners]
                center = (float(t.center[0]), float(t.center[1]))

            dx = center[0] - img_center[0]
            dy = center[1] - img_center[1]
            dist = math.hypot(dx, dy)

            self.detections.append({'ts': ts, 'tag_id': tag_id, 'corners': corners, 'center': center, 'dist': dist})

    def cluster_detections(self, window_s=None, window_m = None):
        if window_s is None:
            window_s = self.args.cluster_max_time
        if window_m is None:
            window_m = self.args.cluster_max_distance
        filtered = []
        # with self.lock:
        # group by tag id
        by_id = defaultdict(list)
        for d in self.detections:
            by_id[d['tag_id']].append(d)

        for tag_id, dets in by_id.items():
            dets.sort(key=lambda x: x['ts'])
            # simple sequential clustering: start cluster, include items while within window
            i = 0
            n = len(dets)
            while i < n:
                cluster = [dets[i]]
                start_ts = dets[i]['ts']
                j = i + 1
                while j < n and dets[j]['ts'] - start_ts <= window_s:
                    cluster.append(dets[j])
                    j += 1
                # pick detection closest to image center
                best = min(cluster, key=lambda x: x['dist'])
                filtered.append(best)
                i = j

        return sorted(filtered, key=lambda x: x['ts'])

    def match_to_keyframes(self, filtered, match_thresh=None):
        if match_thresh is None:
            match_thresh = self.args.match_threshold

        if len(self.keyframes) == 0:
            rospy.logwarn('No keyframes available to match; output will be empty') if rospy else print('No keyframes available')
            return []

        # build array of keyframe timestamps
        k_ts = np.array([kf['ts'] for kf in self.keyframes])

        matched = []
        for d in filtered:
            ts = d['ts']
            # find index of nearest keyframe in time
            idx = int(np.argmin(np.abs(k_ts - ts)))
            dt = abs(k_ts[idx] - ts)
            if dt <= match_thresh:
                kf = self.keyframes[idx]
                # extract x,y,theta
                theta = quaternion_to_yaw(kf['qx'], kf['qy'], kf['qz'], kf['qw'])
                matched.append({'detection': d, 'keyframe': kf, 'x': kf['tx'], 'y': kf['ty'], 'theta': theta, 'kf_idx': idx, 'dt': dt})

        return matched

    def save_outputs(self, matched, output_dir):
        if len(matched) < 1:
            print("no waypoints have been found !!!")
            return
#        os.makedirs(os.path.dirname(output_dir) or '.', exist_ok=True)
        Path(output_dir).mkdir(exist_ok=True, parents=True)
        detections_csv = output_dir + '/tag_detections.csv'
        filtered_csv = output_dir + '/tag_filtered.csv'
        inspections_xytheta = output_dir + '/waypoints.txt'
        inspections_tum = output_dir + '/goals.txt'
        np.savetxt(output_dir + '/path.txt', np.arange(len(matched)).reshape(-1, 1), fmt="%.1f")

        # save raw detections
        with open(detections_csv, 'w', newline='') as f:
            w = csv.writer(f)
            header = ['ts', 'tag_id', 'center_x', 'center_y'] + [f'corner{i}_{c}' for i in range(4) for c in ('x', 'y')]
            w.writerow(header)
            with self.lock:
                for d in self.detections:
                    row = [d['ts'], d['tag_id'], d['center'][0], d['center'][1]]
                    for (cx, cy) in d['corners']:
                        row += [cx, cy]
                    w.writerow(row)

        # save filtered
        filtered = self.cluster_detections()
        with open(filtered_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['ts', 'tag_id', 'center_x', 'center_y', 'dist'])
            for d in filtered:
                w.writerow([d['ts'], d['tag_id'], d['center'][0], d['center'][1], d['dist']])

        # save inspections
        with open(inspections_xytheta, 'w') as f_xy, open(inspections_tum, 'w') as f_tum:
            for m in matched:
                ts = m['keyframe']['ts']
                x = m['x']
                y = m['y']
                theta = m['theta']
                f_xy.write(f"{x} {y} {theta}\n")
                # TUM line
                kf = m['keyframe']
                f_tum.write(f"{ts:.6f} {kf['tx']} {kf['ty']} {kf['tz']} {kf['qx']} {kf['qy']} {kf['qz']} {kf['qw']}\n")

        if rospy:
            rospy.loginfo('Saved outputs: %s*, %s*, %s, %s', output_dir, output_dir, inspections_xytheta, inspections_tum)
        else:
            print('Saved outputs:', detections_csv, filtered_csv, inspections_xytheta, inspections_tum)

        self.visualize(matched, save_path=(self.args.output_dir + '/trajectory.png'))

    def interpolate_keyframes(self, rate_hz=30.0):
        # Interpolate self.keyframes to a fixed rate. Returns new list of keyframes.
        if len(self.keyframes) < 2:
            return self.keyframes

        ts = np.array([kf['ts'] for kf in self.keyframes])
        xs = np.array([kf['tx'] for kf in self.keyframes])
        ys = np.array([kf['ty'] for kf in self.keyframes])
        thetas = np.array([quaternion_to_yaw(kf['qx'], kf['qy'], kf['qz'], kf['qw']) for kf in self.keyframes])

        t_min, t_max = ts[0], ts[-1]
        new_ts = np.arange(t_min, t_max, 1.0 / rate_hz)

        if interpolate:
            fx = interpolate.interp1d(ts, xs, kind='cubic', fill_value='extrapolate')
            fy = interpolate.interp1d(ts, ys, kind='cubic', fill_value='extrapolate')
            fth = interpolate.interp1d(ts, thetas, kind='cubic', fill_value='extrapolate')
            new_x = fx(new_ts)
            new_y = fy(new_ts)
            new_th = fth(new_ts)
        else:
            new_x = np.interp(new_ts, ts, xs)
            new_y = np.interp(new_ts, ts, ys)
            new_th = np.interp(new_ts, ts, thetas)

        new_kfs = []
        for t, x, y, th in zip(new_ts, new_x, new_y, new_th):
            # approximate quaternion from yaw only
            qw = math.cos(th / 2.0)
            qz = math.sin(th / 2.0)
            new_kfs.append({'ts': float(t), 'tx': float(x), 'ty': float(y), 'tz': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': float(qz), 'qw': float(qw)})

        return new_kfs

    def run(self):
        if rospy:
            rospy.loginfo('Running extraction; press Ctrl-C when done to process and save')
            try:
                rospy.spin()
            except KeyboardInterrupt:
                pass
        else:
            print('Non-ROS mode: nothing to run.')

        # After finishing, post-process
        filtered = self.cluster_detections()
        matched = self.match_to_keyframes(filtered)

        # optionally interpolate keyframes and rematch
        if self.args.interpolate:
            new_kfs = self.interpolate_keyframes(rate_hz=self.args.interpolate_rate)
            self.keyframes = new_kfs
            matched = self.match_to_keyframes(filtered)

        # visualize before saving if requested
#        if self.args.show_plot or self.args.save_plot:
        self.save_outputs(matched, self.args.output_dir)

    def visualize(self, matched, save_path=None):
        """Draw 2D trajectory of keyframes and inspection points."""
        if plt is None:
            print('matplotlib not available; skipping visualization')
            return

        if len(self.keyframes) == 0:
            print('No keyframes to visualize')
            return

        kx = [kf['tx'] for kf in self.keyframes]
        ky = [kf['ty'] for kf in self.keyframes]

        ix = [m['x'] for m in matched]
        iy = [m['y'] for m in matched]

        fig, ax = plt.subplots()
        ax.plot(kx, ky, '-k', label='keyframe trajectory')
        ax.scatter(kx, ky, s=10, c='gray')
        if ix:
            ax.scatter(ix, iy, s=50, c='red', marker='x', label='inspection points')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.legend()
        ax.set_title('Keyframe trajectory and inspection points')

        if save_path:
            fig.savefig(save_path)
            print('Saved plot to', save_path)

        plt.show()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--tum_file', type=str, default='', help='Path to TUM-format keyframes file', required=True)
    p.add_argument('--pose_array_topic', type=str, default='', help='Topic for geometry_msgs/PoseArray with keyframes')
    p.add_argument('--image_topic', type=str, default='/pointgrey/image_rect_color/compressed', help='Image topic (color)')
    p.add_argument('--camera_info', dest='camera_info', type=str, default='/pointgrey/camera_info', help='CameraInfo topic')
    p.add_argument('--output_dir', type=str, default='/home/roboslam/slam_ws/data/inspection/', help='Output dir')
    p.add_argument('--cluster_max_time', type=float, default=60.0, help='Time window (s) to cluster detections')
    p.add_argument('--cluster_max_distance', type=float, default=1.0, help='Radius circle (m) to cluster detections')
    p.add_argument('--match_threshold', type=float, default=0.5, help='Max time diff (s) to match detection to keyframe')
    p.add_argument('--center_threshold', type=float, default=1e9, help='(unused) pixel threshold around image center to require')
    p.add_argument('--apriltag_family', type=str, default='tag36h11', help='AprilTag family')
    p.add_argument('--interpolate', action='store_true', help='Interpolate keyframes to uniform rate before matching')
    p.add_argument('--interpolate_rate', type=float, default=30.0, help='Target rate Hz when interpolating')
    p.add_argument('--raw_image', action='store_true', help='Whether the image is raw (default: compressed type)')
    p.add_argument('--show_plot', action='store_true', help='Whether show the inspection points')
    p.add_argument('--save_plot', action='store_true', help='Whether save the inspection points plot')
    return p.parse_args()


def main():
    args = parse_args()
    node = ExtractionNode(args)
    node.run()


if __name__ == '__main__':
    main()

