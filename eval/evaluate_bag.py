#!/usr/bin/env python3
"""rosbag の /ekf_pose（map -> LiDAR）と base の推定を、Gazebo の真値 /wheelchair/pose と比べる.

  python3 evaluate_bag.py <bag> [base の推定トピック（/odometry/global か /ekf_odom）]

/wheelchair/pose（base_footprint、header なし）の時刻は、bag の受信時刻を /clock で sim time に直して使う。
真値の LiDAR は URDF の取付 base_footprint -> velodyne = (-0.17, 0, 1.1377)、回転 0。
"""
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, '/root/ros2_ws/src/localization_bringup/scripts')
from robot_localization_adapter import matrix_to_vec, pose_msg_to_matrix, vec_to_matrix  # noqa: E402

URDF_MOUNT = vec_to_matrix([-0.17, 0.0, 0.10 + 1.0 + 0.0377, 0.0, 0.0, 0.0])
SKIP = 10.0  # 最初の出力から除く時間 [s]（NDT の収束待ち）


def read(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        out.setdefault(topic, []).append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    path = sys.argv[1]
    base_topic = sys.argv[2] if len(sys.argv) > 2 else '/odometry/global'
    d = read(path)
    clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d['/clock']])
    truth_t = np.interp([t for t, _ in d['/wheelchair/pose']], clk[:, 0], clk[:, 1])
    truth_v = np.array([matrix_to_vec(pose_msg_to_matrix(m)) for _, m in d['/wheelchair/pose']])
    truth_v[:, 5] = np.unwrap(truth_v[:, 5])
    order = np.argsort(truth_t)
    truth_t, truth_v = truth_t[order], truth_v[order]

    def truth_base(t):
        if t < truth_t[0] or t > truth_t[-1]:
            return None
        return vec_to_matrix([np.interp(t, truth_t, truth_v[:, i]) for i in range(6)])

    def evaluate(name, samples, mount):
        if not samples:
            print(f'{name}: no data')
            return
        t_first = samples[0][0]
        rows = []
        for t, est in samples:
            tb = truth_base(t)
            if tb is None or t < t_first + SKIP:
                continue
            ref = tb @ mount
            e = est[:3, 3] - ref[:3, 3]
            dyaw = math.atan2(math.sin(matrix_to_vec(est)[5] - matrix_to_vec(ref)[5]),
                              math.cos(matrix_to_vec(est)[5] - matrix_to_vec(ref)[5]))
            k = np.searchsorted(truth_t, t)
            k = min(max(k, 1), len(truth_t) - 1)
            speed = np.linalg.norm(truth_v[k, :2] - truth_v[k - 1, :2]) / max(truth_t[k] - truth_t[k - 1], 1e-6)
            rows.append((math.hypot(e[0], e[1]), e[2], math.degrees(dyaw), speed))
        a = np.array(rows)
        still, moving = a[a[:, 3] < 0.05], a[a[:, 3] > 0.3]
        print(f'{name}: n={len(a)}  xy mean {a[:, 0].mean():.3f} / rms {math.sqrt((a[:, 0] ** 2).mean()):.3f}'
              f' / max {a[:, 0].max():.3f} m | z mean {a[:, 1].mean():+.3f} m'
              f' | yaw mean {a[:, 2].mean():+.2f} / max |{np.abs(a[:, 2]).max():.2f}| deg')
        if len(still) and len(moving):
            print(f'    xy mean: stopped {still[:, 0].mean():.3f} m (n={len(still)}), '
                  f'moving > 0.3 m/s {moving[:, 0].mean():.3f} m (n={len(moving)})')

    ekf_pose = [(stamp(m), pose_msg_to_matrix(m.pose.pose)) for _, m in d.get('/ekf_pose', [])]
    evaluate('/ekf_pose (LiDAR) vs truth LiDAR (URDF mount)', ekf_pose, URDF_MOUNT)
    base = [(stamp(m), pose_msg_to_matrix(m.pose.pose)) for _, m in d.get(base_topic, [])]
    evaluate(f'{base_topic} (base) vs truth base_footprint', base, np.eye(4))
    dist = np.sum(np.linalg.norm(np.diff(truth_v[:, :2], axis=0), axis=1))
    print(f'truth: {truth_t[-1] - truth_t[0]:.1f} s, path length {dist:.1f} m')


if __name__ == '__main__':
    main()
