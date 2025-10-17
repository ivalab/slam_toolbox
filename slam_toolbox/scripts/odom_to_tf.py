#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file odom_to_tf.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 10-17-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""

#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry


class OdomToTF:
    def __init__(self):
        rospy.init_node("odom_to_tf_broadcaster")
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.br = tf.TransformBroadcaster()
        rospy.loginfo("Started broadcasting TF from /odom")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        self.br.sendTransform(
            (pos.x, pos.y, pos.z),
            (ori.x, ori.y, ori.z, ori.w),
            msg.header.stamp,
            msg.child_frame_id,  # usually "base_link"
            msg.header.frame_id,  # usually "odom"
        )


if __name__ == "__main__":
    try:
        OdomToTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
