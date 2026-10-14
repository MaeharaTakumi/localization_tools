"""録画（bag）の置き場所（test_maehara 内）.

  /root/ros2_ws/bags/gazebo/<名前>          Gazebo の録画（<名前>_phases.txt：動作の開始時刻、<名前>_drive.log：走行のログ、
                                            <名前>_info.yaml：録画の情報）
  /root/ros2_ws/bags/gazebo/tune/<試行名>   その再生の出力（tune_run.py・compare_run.py）
  /root/ros2_ws/bags/real/<名前>            実機の録画（同じく _info.yaml を付ける）
  /root/ros2_ws/bags/real/tune/<試行名>     その再生の出力
"""
import os

BAGS = '/root/ros2_ws/bags'
SOURCES = ('gazebo', 'real')


def source_dir(source):
    if source not in SOURCES:
        raise ValueError(f'source must be one of {SOURCES}: {source}')
    return os.path.join(BAGS, source)


def source_of(bag):
    """bag のパスから gazebo / real を返す（どちらの下でもなければ None）."""
    parts = os.path.normpath(os.path.abspath(bag)).split(os.sep)
    for s in SOURCES:
        if s in parts and parts[parts.index(s) - 1] == 'bags':
            return s
    return None


def tune_dir(in_bag):
    """入力の録画に対する再生の出力の置き場所（録画と同じ source の tune/）."""
    return os.path.join(source_dir(source_of(in_bag) or 'gazebo'), 'tune')


def tune_bag(row_or_name, source='gazebo'):
    """結果の 1 行（out_bag があればそれ）か試行名から、再生の出力の bag のパス."""
    if isinstance(row_or_name, dict):
        if row_or_name.get('out_bag'):
            return row_or_name['out_bag']
        row_or_name = row_or_name['name']
    return os.path.join(source_dir(source), 'tune', row_or_name)
