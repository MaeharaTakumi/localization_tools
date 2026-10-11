#!/usr/bin/env python3
"""Gazebo の車椅子を、向きと位置を変えながら何度も止める（NDT の観測ノイズを止まった状態で測るため）.

  初期位置で 45° ずつ 8 方向（各 STOP 秒止まる）→ 1.5 m 前進 → 90° ずつ 4 方向 + 最後にもう一度止まる

回転と前進は真値 /wheelchair/pose を見て目標まで動かす（回転 0.5 rad/s、前進 0.5 m/s）。
止まっていた区間（sim time）を /tmp/drive_phases.txt に「stop_<名前>_start t」「stop_<名前>_end t」で書く。
"""
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

W = 0.5          # 回転 [rad/s]
V = 0.5          # 前進 [m/s]
STOP = 5.0       # 止まる時間 [s]
FORWARD = 1.5    # 前進する距離 [m]
TIMEOUT = 20.0


class Driver(Node):

    def __init__(self):
        super().__init__('drive_stops', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(TwistStamped, '/wheelchair/diff_drive_controller/cmd_vel', 10)
        self.yaw = None   # 真値の yaw（連続になるように足し合わせたもの）
        self.xy = None
        self.create_subscription(Pose, '/wheelchair/pose', self.on_pose, qos_profile_sensor_data)

    def on_pose(self, p):
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.yaw = yaw if self.yaw is None else self.yaw + math.atan2(math.sin(yaw - self.yaw),
                                                                      math.cos(yaw - self.yaw))
        self.xy = (p.position.x, p.position.y)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = v, w
        self.pub.publish(msg)

    def run(self, v, w, done):
        t0 = self.now()
        while not done() and self.now() - t0 < TIMEOUT:
            self.send(v, w)
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self, name, lines):
        t0 = self.now()
        lines.append(f'stop_{name}_start {t0:.3f}')
        while self.now() - t0 < STOP:
            self.send(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
        lines.append(f'stop_{name}_end {self.now():.3f}')
        print(f'stop {name}: yaw {math.degrees(self.yaw):.1f} deg, xy {self.xy[0]:.2f}, {self.xy[1]:.2f}')

    def rotate(self, angle):
        yaw0 = self.yaw
        self.run(0.0, math.copysign(W, angle), lambda: abs(self.yaw - yaw0) >= abs(angle))

    def forward(self, dist):
        x0, y0 = self.xy
        self.run(V, 0.0, lambda: math.hypot(self.xy[0] - x0, self.xy[1] - y0) >= dist)


def main():
    rclpy.init()
    node = Driver()
    while node.now() == 0.0 or node.yaw is None:   # /clock と真値を待つ
        rclpy.spin_once(node, timeout_sec=0.1)
    lines = []
    for i in range(8):
        node.stop(f'A{i}', lines)
        node.rotate(math.radians(45.0))
    node.forward(FORWARD)
    for i in range(4):
        node.stop(f'B{i}', lines)
        node.rotate(math.radians(90.0))
    node.stop('B4', lines)
    with open('/tmp/drive_phases.txt', 'w') as f:
        f.write('\n'.join(lines) + '\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
