"""録画（rosbag2）の読み込みと、録画に付けているファイル・メッセージの時刻の共通部品."""
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from pose_math import transform_msg_to_matrix


def open_reader(path, topics=None):
    """bag を開いた SequentialReader。topics を渡すとそのトピックだけ読む."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader


def read_bag(path, topics=None):
    """bag を {トピック: [(受信時刻 [s], メッセージ)]} で読む。topics を渡すとそのトピックだけ（点群を読まずに速い）."""
    reader = open_reader(path, topics)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        out.setdefault(topic, []).append((t * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def phases_of(bag):
    """<bag>_phases.txt（動作の名前と開始時刻 [s]。drive/run_test.sh が書く）を {名前: 時刻} で読む."""
    out = {}
    with open(bag.rstrip('/') + '_phases.txt') as f:
        for line in f:
            name, t = line.rsplit(' ', 1)
            out[name] = float(t)
    return out


def stamp_sec(msg):
    """header のあるメッセージの stamp [s]."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def stamp_ns(stamp):
    """stamp（builtin_interfaces/Time）[ns]（整数）."""
    return stamp.sec * 10**9 + stamp.nanosec


def mount_from_tf_static(tf_static, base, lidar, required=False):
    """/tf_static のメッセージ [(受信時刻, メッセージ)] をたどった base -> lidar の変換（4x4）.

    Gazebo は base_footprint -> base_link -> velodyne_base_link -> velodyne。
    見つからなければ None（required=True ならエラー）。
    """
    parent = {tr.child_frame_id: (tr.header.frame_id, transform_msg_to_matrix(tr.transform))
              for _, m in tf_static for tr in m.transforms}
    t, frame = np.eye(4), lidar
    while frame != base:
        if frame not in parent:
            if required:
                raise RuntimeError(f'no TF from {base} to {lidar} in /tf_static')
            return None
        frame, tf = parent[frame]
        t = tf @ t
    return t
