#!/usr/bin/env python3
"""録画（drive/run_test.sh の bag）を再生して、候補のパラメータで推定器を動かし、指標を出す（コンテナ test 内で実行）.

  python3 tune_run.py <入力 bag> <試行名> '<parameters の JSON>' ['<launch の追加引数>']

- ROS_DOMAIN_ID=99 で動かす（Gazebo・MATLAB とは混ざらない）
- 入力 bag は最初の動作（<bag>_phases.txt の最も早い時刻。左右 1 周なら「左回転」）の LEAD 秒前から、最後（stop）の TAIL 秒後まで再生する。
- 環境は localization_bringup の param/gazebo/ を /tmp/tune_env_<試行名>/ に写して使う（common/env_files.py）。/tf_static が流れないので、
  その env.yaml にだけ URDF と同じ取付（base_footprint -> base_link -> velodyne_base_link -> velodyne）を static_tfs で書く
- parameters の JSON（{ノード名: {パラメータ: 値}}。ノードは ekf_localizer・pcl_localization・whill_odometry）は、写した <ノード名>.yaml に
  上書きする。キー "__env__" の中身は env.yaml の最上位を上書きする（例 {"__env__": {"use_sim_time": true}}。null の項目は消す）
- 推定器は ekf_localizer（launch の追加引数 use_ekf:=false なら NDT 単体。結果の estimator は ndt_only）。
  launch の追加引数 matcher:=lidarslam なら NDT の代わりに lidarslam（scan_matcher）。EKF なしの指標は /slam_pose（estimator は slam_only）
- 出力：入力と同じ側の tune/<試行名>（/root/ros2_ws/bags/gazebo/tune か /root/ros2_ws/bags/real/tune。bag_paths.py）と、
  指標の 1 行 JSON（標準出力の最後の行。out_bag に出力の bag のパス）
"""
import json
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'plot', 'eval', 'calibration')]   # 共通部品（common/）などを import する
from env_files import make_env  # noqa: E402
from gazebo_model import URDF_TFS  # noqa: E402
from bag_io import phases_of  # noqa: E402
from plot_spin import load_run  # noqa: E402
from transition_eval import evaluate as transition_metrics  # noqa: E402
from bag_paths import tune_dir  # noqa: E402

LEAD = 8.0
TAIL = 4.0
OUT_TOPICS = ['/clock', '/wheelchair/pose', '/ekf_pose', '/ndt_pose', '/slam_pose', '/localization_debug', '/ekf_odom']
IN_TOPICS = ['/clock', '/velodyne_points', '/wheelchair/odom', '/wheelchair/joint_states', '/wheelchair/pose']


def play_window(bag, phases):
    """再生の開始オフセット [s]（bag の先頭から）と長さ [s]。最初の動作の LEAD 秒前から最後の動作の TAIL 秒後まで."""
    first_phase, last_phase = min(phases.values()), max(phases.values())
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/clock']))
    msg_type = get_message('rosgraph_msgs/msg/Clock')
    first = None
    start = end = None
    while reader.has_next():
        _, data, t = reader.read_next()
        first = t if first is None else first
        c = deserialize_message(data, msg_type).clock
        sim = c.sec + c.nanosec * 1e-9
        if start is None and sim >= first_phase - LEAD:
            start = t
        if sim >= last_phase + TAIL:
            end = t
            break
    return (start - first) * 1e-9, (end - start) * 1e-9


def bag_duration(bag):
    """録画の長さ [s]."""
    return rosbag2_py.Info().read_metadata(bag, 'sqlite3').duration.nanoseconds / 1e9


def stop(proc):
    if proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def _mean(a):
    return float(a.mean()) if a.size else float('nan')


def _max(a):
    return float(a.max()) if a.size else float('nan')


def metrics(bag, pose_topic='/ekf_pose'):
    """指標。angle_diff（/localization_debug）は NDT だけが出す（lidarslam では NaN、n_ndt は 0）."""
    r = load_run(bag, pose_topic)
    ph = r['phases']
    t, e = r['t'], r['err']
    at, ang = r['ang_t'], r['ang']
    sw0, sw1 = ph['right turn'], ph['right turn'] + 1.0       # 回転の切り替えの窓
    motion = (t >= 0) & (t <= ph['stop'] + 2.0)
    rot = (t >= 0) & (t < ph['forward']) & ~((t >= sw0) & (t <= sw1))
    amotion = (at >= 0) & (at <= ph['stop'] + 2.0)
    aswitch = (at >= sw0 - 0.2) & (at <= sw1)
    switch = (t >= sw0 - 0.2) & (t <= sw1)
    return {
        **transition_metrics(bag, pose_topic),
        'yaw_rot': float(np.abs(e[rot, 2]).mean()),
        'ang_mean': _mean(ang[amotion]),
        'ang_max': _max(ang[amotion]),
        'xy_mean': float(e[motion, 0].mean()),
        'xy_max': float(e[motion, 0].max()),
        'spike_yaw': float(np.abs(e[switch, 2]).max()),
        'spike_ang': _max(ang[aswitch]),
        'yaw_max': float(np.abs(e[motion, 2]).max()),
        'n_pose': int(motion.sum()), 'n_ndt': int(amotion.sum()),
    }


def launch_and_play(name, env_path, in_bag, out_bag, record_topics, play, launch_args=(), tag='tune'):
    """環境のフォルダ env_path で自己位置推定を起動し、in_bag を再生して record_topics を out_bag に録る。ログのパスを返す.

    ROS_DOMAIN_ID=99 で動かす（動いている推定・Gazebo・MATLAB とは混ざらない）。play() は地図の読み込みを待ったあとに呼ばれ、
    (ros2 bag play の追加の引数, 再生にかかる時間 [s]) を返す。record_topics が空なら録らない。ログは /tmp/<tag>_<name>.log。
    """
    penv = dict(os.environ, ROS_DOMAIN_ID='99')
    log_path = f'/tmp/{tag}_{name}.log'
    log = open(log_path, 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'localization_bringup', 'localization.launch.py', f'env:={env_path}',
         'rviz:=false', *launch_args],
        env=penv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    record = None
    if record_topics:
        record = subprocess.Popen(['ros2', 'bag', 'record', '-o', out_bag, *record_topics],
                                  env=penv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(10.0)   # 地図の読み込み
        play_args, duration = play()
        subprocess.run(['ros2', 'bag', 'play', in_bag, *play_args], env=penv, stdout=log, stderr=subprocess.STDOUT,
                       timeout=duration + 30)
        time.sleep(2.0)
    finally:
        stop(launch)
        if record is not None:
            stop(record)
        log.close()
    return log_path


def replay(in_bag, name, params, extra=(), out_topics=OUT_TOPICS, whole_bag=False):
    """Gazebo の録画 in_bag を候補のパラメータで再生して推定器を動かし、出力の bag（入力と同じ側の tune/<name>）を返す.

    再生するのは最初の動作の LEAD 秒前から最後の動作の TAIL 秒後まで（whole_bag=True なら録画全体）。録るのは out_topics。
    """
    params = dict(params)
    out_bag = os.path.join(tune_dir(in_bag), name)
    os.makedirs(tune_dir(in_bag), exist_ok=True)
    subprocess.run(['rm', '-rf', out_bag])
    phases = phases_of(in_bag)
    with open(out_bag + '_phases.txt', 'w') as f:
        f.writelines(f'{k} {v:.3f}\n' for k, v in phases.items())

    env_values = {'static_tfs': URDF_TFS, **(params.pop('__env__', None) or {})}
    env = make_env('gazebo', f'/tmp/tune_env_{name}', env_values, params)

    def play():
        offset, duration = (0.0, bag_duration(in_bag)) if whole_bag else play_window(in_bag, phases)
        return ['--start-offset', f'{offset:.3f}', '--topics', *IN_TOPICS], duration

    log_path = launch_and_play(name, env, in_bag, out_bag, out_topics, play, launch_args=extra)
    # ノードが再生の途中で落ちた回は使わない（例：パラメータを読めずに起動しなかった）。止める合図（launch の
    # 'user interrupted'）より後に落ちたものは除く（止めるときの SIGINT や、終了時のまれなセグメンテーション違反）
    died = []
    with open(log_path) as f:
        for line in f:
            if 'user interrupted' in line:
                break
            m = re.search(r'\[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)', line)
            if m:
                died.append(f'{m.group(1)} (exit code {m.group(2)})')
    if died:
        raise RuntimeError(f'process died during the replay: {", ".join(died)} (see {log_path})')
    return out_bag


def main():
    in_bag, name, params_json = sys.argv[1:4]
    extra = sys.argv[4].split() if len(sys.argv) > 4 else []
    out_bag = replay(in_bag, name, json.loads(params_json), extra)
    slam = 'matcher:=lidarslam' in extra
    if 'use_ekf:=false' in extra:   # 点群の位置合わせ単体（NDT か lidarslam）
        estimator, topic = ('slam_only', '/slam_pose') if slam else ('ndt_only', '/ndt_pose')
    else:
        estimator, topic = ('ekf_localizer_slam' if slam else 'ekf_localizer'), '/ekf_pose'
    print(json.dumps({'name': name, 'estimator': estimator, 'out_bag': out_bag, **metrics(out_bag, topic)}))


if __name__ == '__main__':
    main()
