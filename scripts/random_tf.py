#!/usr/bin/env python

"""
node to generate a tf used for testing
"""

import rospy
import tf2_ros
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('random_tf2_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = "map"

    phase_drift = 1.13

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        x = 5 * math.sin(rospy.Time.now().to_sec() * math.pi * 0.01)
        y = 5 * math.sin(phase_drift*(rospy.Time.now().to_sec()) * math.pi * 0.01 )

        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        t.child_frame_id = "RadEye"
        br.sendTransform(t)

        t.child_frame_id = "epd"
        br.sendTransform(t)
        rate.sleep()
