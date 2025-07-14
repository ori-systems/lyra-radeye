#!/usr/bin/env python

"""
node to generate a tf used for testing
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
import math
from tf_transformations import quaternion_from_euler

class RandomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('random_tf_broadcaster')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.phase_drift = 1.13

    def timer_callback(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "map"
        t.header.stamp = self.get_clock().now().to_msg()

        time_sec = self.get_clock().now().nanoseconds / 1e9
        x = 5 * math.sin(time_sec * math.pi * 0.01)
        y = 5 * math.sin(self.phase_drift * time_sec * math.pi * 0.01)

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t.child_frame_id = "RadEye"
        self.br.sendTransform(t)

        t.child_frame_id = "epd"
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RandomTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
