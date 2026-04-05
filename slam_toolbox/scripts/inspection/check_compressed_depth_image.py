#!/usr/bin/env python3
"""
ROS node that subscribes to three depth topics:
 - /camera/depth/image_raw (sensor_msgs/Image)
 - /camera/depth/compressed (sensor_msgs/CompressedImage)
 - /camera/depth/compressedDepth (sensor_msgs/CompressedImage)

It decodes compressed images, aligns messages by timestamp, and reports
whether the compressed images match the raw image (exact for integers,
or within tolerance for floats).
"""
import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage

from image_transport_codecs import decode
from cv_bridge import CvBridge, CvBridgeError
import message_filters


class DepthComparator(object):
    def __init__(self):
        self.bridge = CvBridge()

        # parameters (allow remapping via rosparam or topic remapping)
        raw_topic = rospy.get_param('~raw_topic', '/camera/aligned_depth_to_color/image_raw')
        comp_topic = rospy.get_param('~compressed_topic', '/camera/aligned_depth_to_color/image_raw/compressed')
        compd_topic = rospy.get_param('~compressed_depth_topic', '/camera/aligned_depth_to_color/image_raw/compressedDepth/decompressed')

        rospy.loginfo('Subscribing to: %s, %s, %s', raw_topic, comp_topic, compd_topic)

        sub_raw = message_filters.Subscriber(raw_topic, Image)
        sub_comp = message_filters.Subscriber(comp_topic, CompressedImage)
        sub_compd = message_filters.Subscriber(compd_topic, Image)

        # approximate sync to allow slight time differences
        ats = message_filters.ApproximateTimeSynchronizer([sub_raw, sub_comp, sub_compd], queue_size=20, slop=0.1)
        ats.registerCallback(self.callback)

        # counters
        self.msg_count = 0

    def decode_compressed(self, comp_msg):
        """Decode a sensor_msgs/CompressedImage to an OpenCV image (numpy array).

        Returns the decoded numpy array (unchanged channels/dtype) or raises an exception.
        """
        try:
            arr = np.frombuffer(comp_msg.data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
            if img is None:
                raise ValueError('cv2.imdecode returned None')
            return img
        except Exception as e:
            raise

    def to_single_channel_if_needed(self, img, raw_shape):
        """If img has multiple channels but raw image is single channel,
        convert to single channel via grayscale.
        """
        if img is None:
            return img
        if len(raw_shape) == 2 and img.ndim == 3:
            # convert color to gray
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return img

    def compare_arrays(self, raw_cv, comp_cv, compd_cv):
        # convert compressed decoded images to match raw_cv shape and dtype as best effort
        # keep copies
        comp = comp_cv
        compd = compd_cv

        # If color images were decoded but raw is single-channel, convert
        comp = self.to_single_channel_if_needed(comp, raw_cv.shape)
        compd = self.to_single_channel_if_needed(compd, raw_cv.shape)

        # Cast comp/compd to raw dtype for comparison (best-effort)
        try:
            # If raw is float32, convert both comps to float32
            if raw_cv.dtype == np.float32 or raw_cv.dtype == np.float64:
                compf = comp.astype(np.float32)
                compdf = compd.astype(np.float32)
                rawf = raw_cv.astype(np.float32)
                # compute differences
                diff_c = np.abs(rawf - compf)
                diff_cd = np.abs(rawf - compdf)
                tol = rospy.get_param('~float_tolerance', 1e-3)
                eq_c = diff_c <= tol
                eq_cd = diff_cd <= tol
                maxdiff_c = float(np.max(diff_c)) if diff_c.size else 0.0
                maxdiff_cd = float(np.max(diff_cd)) if diff_cd.size else 0.0
                meandiff_c = float(np.mean(diff_c)) if diff_c.size else 0.0
                meandiff_cd = float(np.mean(diff_cd)) if diff_cd.size else 0.0
                return eq_c, eq_cd, maxdiff_c, maxdiff_cd, meandiff_c, meandiff_cd
            else:
                # integer types: compare equality after casting comp to raw dtype
                comp_cast = comp.astype(raw_cv.dtype)
                compd_cast = compd.astype(raw_cv.dtype)
                eq_c = (raw_cv == comp_cast)
                eq_cd = (raw_cv == compd_cast)
                # produce simple diffs statistics
                diff_c = (raw_cv.astype(np.float32) - comp_cast.astype(np.float32))
                diff_cd = (raw_cv.astype(np.float32) - compd_cast.astype(np.float32))
                maxdiff_c = float(np.max(np.abs(diff_c))) if diff_c.size else 0.0
                maxdiff_cd = float(np.max(np.abs(diff_cd))) if diff_cd.size else 0.0
                meandiff_c = float(np.mean(np.abs(diff_c))) if diff_c.size else 0.0
                meandiff_cd = float(np.mean(np.abs(diff_cd))) if diff_cd.size else 0.0
                return eq_c, eq_cd, maxdiff_c, maxdiff_cd, meandiff_c, meandiff_cd
        except Exception as e:
            rospy.logwarn('Comparison failed: %s', str(e))
            return None

    def callback(self, raw_msg, comp_msg, compd_msg):
        self.msg_count += 1
        try:
            # decode raw
            raw_cv = self.bridge.imgmsg_to_cv2(raw_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr('CvBridge error decoding raw image: %s', str(e))
            return

        # decode compressed images
        try:
            comp_cv = self.decode_compressed(comp_msg)
        except Exception as e:
            rospy.logwarn('Failed to decode compressed image: %s', str(e))
            return

        try:
            # compd_cv = self.decode_compressed(compd_msg)
#            compd_cv, error = decode(compd_msg)
            compd_cv = self.bridge.imgmsg_to_cv2(compd_msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logwarn('Failed to decode compressedDepth image: %s', str(e))
            return

        # If compressed decode yields 3 channels and raw is single-channel, convert above
        # If shapes mismatch, attempt simple resize/back-conversion if needed
        if comp_cv.shape != raw_cv.shape:
            print("Decompressed compressed image shape mismatch")
            # # try resizing compressed to raw shape (width/height). This is best-effort.
            # try:
            #     if comp_cv.ndim == 2 and raw_cv.ndim == 2:
            #         comp_cv = cv2.resize(comp_cv, (raw_cv.shape[1], raw_cv.shape[0]), interpolation=cv2.INTER_NEAREST)
            #     elif comp_cv.ndim == 3 and raw_cv.ndim == 2:
            #         comp_cv = cv2.resize(comp_cv, (raw_cv.shape[1], raw_cv.shape[0]), interpolation=cv2.INTER_NEAREST)
            #         comp_cv = cv2.cvtColor(comp_cv, cv2.COLOR_BGR2GRAY)
            #     elif comp_cv.ndim == 3 and raw_cv.ndim == 3:
            #         comp_cv = cv2.resize(comp_cv, (raw_cv.shape[1], raw_cv.shape[0]), interpolation=cv2.INTER_NEAREST)
            #     else:
            #         # leave it
            #         pass
            # except Exception:
            #     pass
            comp_cv = None

        if compd_cv.shape != raw_cv.shape:
            print("Decompressed compressedDepth image shape mismatch")
            compd_cv = None
            # try:
            #     if compd_cv.ndim == 2 and raw_cv.ndim == 2:
            #         compd_cv = cv2.resize(compd_cv, (raw_cv.shape[1], raw_cv.shape[0]), interpolation=cv2.INTER_NEAREST)
            #     elif compd_cv.ndim == 3 and raw_cv.ndim == 2:
            #         compd_cv = cv2.resize(compd_cv, (raw_cv.shape[1], raw_cv.shape[0]), interpolation=cv2.INTER_NEAREST)
            #         compd_cv = cv2.cvtColor(compd_cv, cv2.COLOR_BGR2GRAY)
            #     elif compd_cv.ndim == 3 and raw_cv.ndim == 3:
            #         compd_cv = cv2.resize(compd_cv, (raw_cv.shape[1], raw_cv.shape[0]), interpolation=cv2.INTER_NEAREST)
            # except Exception:
            #     pass
        if comp_cv is None and compd_cv is None:
            print("No available images to compare")
            return

        result = self.compare_arrays(raw_cv, comp_cv, compd_cv)
        if result is None:
            rospy.loginfo('Could not compare this triplet (msg #%d).', self.msg_count)
            return

        eq_c, eq_cd, maxdiff_c, maxdiff_cd, meandiff_c, meandiff_cd = result

        # compute percentages
        total = raw_cv.size
        pct_eq_c = 100.0 * float(np.count_nonzero(eq_c)) / float(total) if total else 0.0
        pct_eq_cd = 100.0 * float(np.count_nonzero(eq_cd)) / float(total) if total else 0.0

        rospy.loginfo('Msg #%d: comp==raw: %.2f%% (max %.6g mean %.6g), compDepth==raw: %.2f%% (max %.6g mean %.6g)',
                      self.msg_count, pct_eq_c, maxdiff_c, meandiff_c, pct_eq_cd, maxdiff_cd, meandiff_cd)


def main(argv):
    rospy.init_node('compare_depth_images')
    dc = DepthComparator()
    rospy.loginfo('compare_depth_images node started')
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

