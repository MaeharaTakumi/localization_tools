#!/usr/bin/env python3
"""robot_localization 経路の合成データテスト（Gazebo なし。ROS_DOMAIN_ID を分けて実行する）.

真値の軌跡（等速円運動）から /clock・/wheelchair/odom・/velodyne_points（空）・/ndt_pose（LiDAR）を作って流し、
/odometry/global（map -> base）と /ekf_pose（map -> LiDAR、点群の stamp）を真値と比べる。
途中で /initialpose を送り、2 つの EKF がその姿勢にリセットされることを確かめる。
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, '/root/ros2_ws/src/localization_bringup/scripts')
from robot_localization_adapter import (  # noqa: E402
    fill_pose_msg, matrix_to_vec, pose_msg_to_matrix, vec_to_matrix, wrap)

T0 = 100.0
X0, Y0, Z0, YAW0 = 80.037, 7.072, 12.332, 3.110
V, W = 0.5, 0.1
# 取付 base_footprint -> velodyne（URDF。make_test_env.py の static_tfs を合成）
T_BL = vec_to_matrix([-0.17, 0.0, 0.10 + 1.0 + 0.0377, 0, 0, 0])
NDT_DELAY = 0.05
RESET_AT = 12.0
END_AT = 20.0


def truth_base(t):
    s = t - T0
    yaw = YAW0 + W * s
    x = X0 + V / W * (math.sin(yaw) - math.sin(YAW0))
    y = Y0 - V / W * (math.cos(yaw) - math.cos(YAW0))
    return vec_to_matrix([x, y, Z0, 0.0, 0.0, yaw])


def stamp(t):
    ns = int(round(t * 1e9))
    return ns // 1_000_000_000, ns % 1_000_000_000


def to_time(t, msg_stamp):
    msg_stamp.sec, msg_stamp.nanosec = stamp(t)


def from_stamp(s):
    return s.sec + s.nanosec * 1e-9


def err(t_est, t_true):
    d = matrix_to_vec(np.linalg.inv(t_true) @ t_est)
    return math.hypot(d[0], d[1]), abs(d[2]), abs(wrap(d[5]))


class Tester(Node):

    def __init__(self):
        super().__init__('rl_synthetic_test')
        self.rng = np.random.default_rng(0)
        self.wall0 = time.monotonic()
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheelchair/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/velodyne_points', 10)
        self.ndt_pub = self.create_publisher(PoseWithCovarianceStamped, '/ndt_pose', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        rel = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/odometry/global', self.on_global, rel)
        self.create_subscription(Odometry, '/odometry/local', self.on_local, rel)
        self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.on_ekf_pose, rel)
        self.global_log, self.local_log, self.ekf_pose_log = [], [], []
        self.cloud_stamps = set()
        self.pending_ndt = []
        self.next_odom = self.next_cloud = T0
        self.reset_sent = None
        self.create_timer(0.005, self.tick)

    def now_sim(self):
        return T0 + (time.monotonic() - self.wall0)

    def tick(self):
        t = self.now_sim()
        c = Clock()
        to_time(t, c.clock)
        self.clock_pub.publish(c)
        while self.next_odom <= t:
            o = Odometry()
            to_time(self.next_odom, o.header.stamp)
            o.header.frame_id, o.child_frame_id = 'wheelchair/odom', 'wheelchair/base_link'  # Gazebo と同じ
            o.twist.twist.linear.x, o.twist.twist.angular.z = V, W
            self.odom_pub.publish(o)
            self.next_odom += 0.02
        while self.next_cloud <= t:
            pc = PointCloud2()
            to_time(self.next_cloud, pc.header.stamp)
            pc.header.frame_id = 'velodyne'
            self.cloud_pub.publish(pc)
            self.cloud_stamps.add(stamp(self.next_cloud))
            self.pending_ndt.append(self.next_cloud)
            self.next_cloud += 0.1
        while self.pending_ndt and self.pending_ndt[0] + NDT_DELAY <= t:
            ts = self.pending_ndt.pop(0)
            noise = np.concatenate([self.rng.normal(0, 0.02, 3), self.rng.normal(0, 0.002, 3)])
            p = PoseWithCovarianceStamped()
            to_time(ts, p.header.stamp)
            p.header.frame_id = 'map'
            fill_pose_msg(p.pose.pose, truth_base(ts) @ T_BL @ vec_to_matrix(noise))
            self.ndt_pub.publish(p)
        if self.reset_sent is None and t >= T0 + RESET_AT:
            p = PoseWithCovarianceStamped()
            to_time(t, p.header.stamp)
            p.header.frame_id = 'map'
            fill_pose_msg(p.pose.pose, truth_base(t) @ T_BL)   # /initialpose は LiDAR の姿勢
            self.init_pub.publish(p)
            self.reset_sent = t

    def on_global(self, m):
        self.global_log.append((from_stamp(m.header.stamp), pose_msg_to_matrix(m.pose.pose)))

    def on_local(self, m):
        self.local_log.append((from_stamp(m.header.stamp), pose_msg_to_matrix(m.pose.pose)))

    def on_ekf_pose(self, m):
        self.ekf_pose_log.append(
            (from_stamp(m.header.stamp), (m.header.stamp.sec, m.header.stamp.nanosec),
             pose_msg_to_matrix(m.pose.pose), np.array(m.pose.covariance).reshape(6, 6)))


def summarize(name, rows):
    if not rows:
        print(f'{name}: no data')
        return
    a = np.array(rows)
    print(f'{name}: n={len(a)}  xy max {a[:, 0].max():.3f} m, mean {a[:, 0].mean():.3f} m | '
          f'z max {a[:, 1].max():.3f} m | yaw max {math.degrees(a[:, 2].max()):.2f} deg')


def main():
    rclpy.init()
    node = Tester()
    while node.now_sim() < T0 + END_AT:
        rclpy.spin_once(node, timeout_sec=0.001)
    rs = node.reset_sent
    settle = 3.0  # 初期化・リセット直後は除く
    g = [err(T, truth_base(t)) for t, T in node.global_log
         if T0 + settle < t < rs or t > rs + settle]
    summarize('/odometry/global vs truth (base)', g)
    e = [err(T, truth_base(t) @ T_BL) for t, _, T, _ in node.ekf_pose_log
         if T0 + settle < t < rs or t > rs + settle]
    summarize('/ekf_pose vs truth (LiDAR)', e)
    on_cloud = sum(1 for _, s, _, _ in node.ekf_pose_log if s in node.cloud_stamps)
    # adapter の publish_trigger が cloud なら全部、timer（既定）なら 0 になる
    rate = len(node.ekf_pose_log) / max(1e-9, node.ekf_pose_log[-1][0] - node.ekf_pose_log[0][0]) if len(node.ekf_pose_log) > 1 else 0
    print(f'/ekf_pose stamps equal to cloud stamps: {on_cloud}/{len(node.ekf_pose_log)}'
          f' (clouds sent: {len(node.cloud_stamps)}; all for publish_trigger cloud, none for timer), {rate:.0f} Hz')
    if node.ekf_pose_log:
        c = node.ekf_pose_log[-1][3]
        print('/ekf_pose last covariance diag:', np.round(np.diag(c), 5))
    # リセット後の odom（ekf_filter_node_odom）は初期姿勢から動くので、真値と比べる（map -> odom がほぼ恒等になる）
    before = [(t, T) for t, T in node.local_log if t < rs]
    after = [(t, T) for t, T in node.local_log if rs + 0.3 <= t < rs + 1.0]
    if before and after:
        print('/odometry/local before reset xyz:', np.round(before[-1][1][:3, 3], 3))
        t, T = after[-1]
        print(f'/odometry/local {t - rs:.2f} s after reset xyz:', np.round(T[:3, 3], 3),
              ' truth base xyz:', np.round(truth_base(t)[:3, 3], 3))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
