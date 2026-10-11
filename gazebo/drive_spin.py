#!/usr/bin/env python3
"""Gazebo の車椅子を「左に 1 周 → 右に 1 周 → 3 秒前進」で走らせる（動作の間に停止を入れない）.

回転は、真値 /wheelchair/pose の yaw が動作の開始時から ±360° 変わるまで続ける（指令は ±1 rad/s）。
前進は sim time で 3 秒（1.5 m/s）。各動作の開始時刻（sim time）を /tmp/drive_phases.txt に書く（グラフ用）。
"""
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

W = 1.0   # 角速度 [rad/s]
V = 1.5   # 並進速度 [m/s]
FORWARD_TIME = 3.0
TIMEOUT = 20.0   # 回転が終わらないときの打ち切り [s]


class Driver(Node):

    def __init__(self):
        super().__init__('drive_spin', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(TwistStamped, '/wheelchair/diff_drive_controller/cmd_vel', 10)
        self.yaw = None      # 真値の yaw（連続になるように足し合わせたもの）
        self.create_subscription(Pose, '/wheelchair/pose', self.on_pose, qos_profile_sensor_data)

    def on_pose(self, p):
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.yaw is None:
            self.yaw = yaw
        else:
            self.yaw += math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = v, w
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Driver()
    while node.now() == 0.0 or node.yaw is None:   # /clock と真値を待つ
        rclpy.spin_once(node, timeout_sec=0.1)

    lines = []
    for name, w, target in (('left turn', W, 2 * math.pi), ('right turn', -W, -2 * math.pi)):
        t0, yaw0 = node.now(), node.yaw
        lines.append(f'{name} {t0:.3f}')
        while abs(node.yaw - yaw0) < abs(target) or (node.yaw - yaw0) * target < 0:
            if node.now() - t0 > TIMEOUT:
                print(f'{name}: timeout ({math.degrees(node.yaw - yaw0):.1f} deg)')
                break
            node.send(0.0, w)
            rclpy.spin_once(node, timeout_sec=0.02)
        print(f'{name}: {math.degrees(node.yaw - yaw0):.1f} deg in {node.now() - t0:.2f} s')
    t0 = node.now()
    lines.append(f'forward {t0:.3f}')
    while node.now() - t0 < FORWARD_TIME:
        node.send(V, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
    lines.append(f'stop {node.now():.3f}')
    for _ in range(20):
        node.send(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
    with open('/tmp/drive_phases.txt', 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
