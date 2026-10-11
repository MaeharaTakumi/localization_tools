#!/usr/bin/env python3
"""Gazebo の車椅子を決まった速度指令で走らせる（前進 → その場で 180° 旋回 → 戻る → 停止）."""
import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

PHASES = [  # (秒, v [m/s], omega(t) [rad/s])
    (28.0, 0.5, lambda t: 0.15 * math.sin(2 * math.pi * t / 10.0)),
    (math.pi / 0.4, 0.0, lambda t: 0.4),
    (24.0, 0.5, lambda t: 0.10 * math.sin(2 * math.pi * t / 8.0)),
    (2.0, 0.0, lambda t: 0.0),
]


def main():
    rclpy.init()
    node = Node('drive_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
    pub = node.create_publisher(TwistStamped, '/wheelchair/diff_drive_controller/cmd_vel', 10)
    time.sleep(1.0)
    for duration, v, omega in PHASES:
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < duration:
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = v
            msg.twist.angular.z = omega(t)
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
    print('drive finished')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
