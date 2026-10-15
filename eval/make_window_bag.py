#!/usr/bin/env python3
"""録画の /wheelchair/odom の速度を、diff_drive_controller の移動平均 n 個のものに置き換えた bag を作る（コンテナ test 内で実行）.

  python3 make_window_bag.py <入力 bag> <出力 bag> <n>

同じ走行で、Gazebo の velocity_rolling_window_size だけを変えた場合の入力を再現する（推定器の比較用）。
速度は /wheelchair/joint_states の車輪の回転角から diff_drive_controller（position_feedback: true）と同じ計算で求め、
stamp が同じ /wheelchair/odom の twist.linear.x・angular.z を書き換える。tune_run.py が再生するトピックだけを写す。
<入力 bag>_phases.txt も写す。n に録画時と同じ値を与えると元と同じ値になる（確認用）。
"""
import os
import shutil
import sys

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_odom_filter などを import する
from plot_odom_filter import B, R  # noqa: E402

TOPICS = ['/clock', '/velodyne_points', '/wheelchair/odom', '/wheelchair/joint_states', '/wheelchair/pose']


def open_reader(path, topics=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def stamp_ns(m):
    return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec


def rolling(in_bag, n):
    """joint_states の stamp [ns] -> (v, omega)（窓 n 個の平均。最初の n 個は無し）."""
    reader = open_reader(in_bag, ['/wheelchair/joint_states'])
    msg_type = get_message('sensor_msgs/msg/JointState')
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, msg_type)
        il, ir = m.name.index('left_wheel_joint'), m.name.index('right_wheel_joint')
        rows.append((stamp_ns(m), m.position[il], m.position[ir]))
    rows.sort()
    t = np.array([r[0] for r in rows])
    pl, pr = np.array([r[1] for r in rows]), np.array([r[2] for r in rows])
    dt = np.diff(t) * 1e-9
    dl, dr = np.diff(pl) * R, np.diff(pr) * R
    k = np.ones(n) / n
    v = np.convolve((dl + dr) / 2 / dt, k)[:len(dt)]
    w = np.convolve((dr - dl) / B / dt, k)[:len(dt)]
    return {int(t[i + 1]): (v[i], w[i]) for i in range(n - 1, len(dt))}


def main():
    in_bag, out_bag, n = sys.argv[1], sys.argv[2], int(sys.argv[3])
    table = rolling(in_bag, n)
    reader = open_reader(in_bag, TOPICS)
    shutil.rmtree(out_bag, ignore_errors=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out_bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for meta in reader.get_all_topics_and_types():
        if meta.name in TOPICS:
            writer.create_topic(meta)
    odom_type = get_message('nav_msgs/msg/Odometry')
    replaced = missing = 0
    diff = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/wheelchair/odom':
            m = deserialize_message(data, odom_type)
            vw = table.get(stamp_ns(m))
            if vw is None:
                missing += 1
            else:
                diff.append((m.twist.twist.linear.x - vw[0], m.twist.twist.angular.z - vw[1]))
                m.twist.twist.linear.x, m.twist.twist.angular.z = float(vw[0]), float(vw[1])
                data = serialize_message(m)
                replaced += 1
        writer.write(topic, data, t)
    del writer
    shutil.copy(in_bag.rstrip('/') + '_phases.txt', out_bag.rstrip('/') + '_phases.txt')
    d = np.array(diff)
    print(f'/wheelchair/odom: replaced {replaced}, no matching joint_states {missing}; '
          f'change RMS v {np.sqrt((d[:, 0] ** 2).mean()):.4f} m/s, omega {np.sqrt((d[:, 1] ** 2).mean()):.4f} rad/s')


if __name__ == '__main__':
    main()
