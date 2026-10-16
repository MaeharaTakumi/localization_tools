#!/usr/bin/env python3
"""録画（drive/run_test.sh の bag）を再生して、候補のパラメータで推定器を動かし、指標を出す（コンテナ test 内で実行）.

  python3 tune_run.py <入力 bag> <試行名> '<parameters の JSON>' ['<launch の追加引数>']

- ROS_DOMAIN_ID=99 で動かす（Gazebo・MATLAB とは混ざらない）
- 入力 bag は最初の動作（<bag>_phases.txt の最も早い時刻。左右 1 周なら「左回転」）の LEAD 秒前から、最後（stop）の TAIL 秒後まで再生する。/tf_static が流れないので、調整用の env にだけ
  URDF と同じ取付（base_footprint -> base_link -> velodyne_base_link -> velodyne）を static_tfs で書く
- parameters の JSON は env の parameters にそのまま入れる。キー "__env__" の中身は env の最上位を上書きする
  （例 {"__env__": {"topics": {...}, "joint_state_odometry": null}}。null の項目は消す）
- 推定器は ekf_localizer（launch の追加引数 use_ekf:=false なら NDT 単体。結果の estimator は ndt_only）
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
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'calibration')]   # plot_spin などを import する
from plot_spin import load_run  # noqa: E402
from transition_eval import evaluate as transition_metrics  # noqa: E402
from bag_paths import tune_dir  # noqa: E402

LEAD = 8.0
TAIL = 4.0
ENV_BASE = '/root/ros2_ws/install/localization_bringup/share/localization_bringup/param/env/gazebo.yaml'
URDF_TFS = [
    {'parent': 'base_footprint', 'child': 'base_link', 'xyz': [0.0, 0.0, 0.10], 'rpy': [0.0, 0.0, 0.0]},
    {'parent': 'base_link', 'child': 'velodyne_base_link', 'xyz': [-0.17, 0.0, 1.0], 'rpy': [0.0, 0.0, 0.0]},
    {'parent': 'velodyne_base_link', 'child': 'velodyne', 'xyz': [0.0, 0.0, 0.0377], 'rpy': [0.0, 0.0, 0.0]},
]
OUT_TOPICS = ['/clock', '/wheelchair/pose', '/ekf_pose', '/ndt_pose', '/localization_debug', '/ekf_odom']
IN_TOPICS = ['/clock', '/velodyne_points', '/wheelchair/odom', '/wheelchair/joint_states', '/wheelchair/pose']


def phases_of(bag):
    out = {}
    with open(bag.rstrip('/') + '_phases.txt') as f:
        for line in f:
            name, t = line.rsplit(' ', 1)
            out[name] = float(t)
    return out


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


def stop(proc):
    if proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def metrics(bag, pose_topic='/ekf_pose'):
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
        'ang_mean': float(ang[amotion].mean()),
        'ang_max': float(ang[amotion].max()),
        'xy_mean': float(e[motion, 0].mean()),
        'xy_max': float(e[motion, 0].max()),
        'spike_yaw': float(np.abs(e[switch, 2]).max()),
        'spike_ang': float(ang[aswitch].max()),
        'yaw_max': float(np.abs(e[motion, 2]).max()),
        'n_pose': int(motion.sum()), 'n_ndt': int(amotion.sum()),
    }


def replay(in_bag, name, params, extra=()):
    """入力 bag を候補のパラメータで再生して推定器を動かし、出力の bag（入力と同じ側の tune/<name>）を返す."""
    params = dict(params)
    out_bag = os.path.join(tune_dir(in_bag), name)
    os.makedirs(tune_dir(in_bag), exist_ok=True)
    subprocess.run(['rm', '-rf', out_bag])
    phases = phases_of(in_bag)
    with open(out_bag + '_phases.txt', 'w') as f:
        f.writelines(f'{k} {v:.3f}\n' for k, v in phases.items())

    with open(ENV_BASE) as f:
        env = yaml.safe_load(f)
    env['static_tfs'] = URDF_TFS
    env.setdefault('parameters', {})
    for key, value in (params.pop('__env__', None) or {}).items():
        if value is None:
            env.pop(key, None)
        else:
            env[key] = value
    for node, p in params.items():
        env['parameters'][node] = {**(env['parameters'].get(node) or {}), **p}   # env の節が空（None）でもよい
    env_path = f'/tmp/tune_env_{name}.yaml'
    with open(env_path, 'w') as f:
        yaml.safe_dump(env, f)

    penv = dict(os.environ, ROS_DOMAIN_ID='99')
    log = open(f'/tmp/tune_{name}.log', 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'localization_bringup', 'localization.launch.py', f'env:={env_path}',
         'rviz:=false', *extra],
        env=penv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    record = subprocess.Popen(['ros2', 'bag', 'record', '-o', out_bag, *OUT_TOPICS],
                              env=penv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(10.0)   # 地図の読み込み
        offset, duration = play_window(in_bag, phases)
        subprocess.run(['ros2', 'bag', 'play', in_bag, '--start-offset', f'{offset:.3f}',
                        '--topics', *IN_TOPICS], env=penv, stdout=log, stderr=subprocess.STDOUT,
                       timeout=duration + 30)
        time.sleep(2.0)
    finally:
        stop(launch)
        stop(record)
        log.close()
    # ノードが再生の途中で落ちた回は使わない（例：パラメータを読めずに起動しなかった）。止める合図（launch の
    # 'user interrupted'）より後に落ちたものは除く（止めるときの SIGINT や、終了時のまれなセグメンテーション違反）
    died = []
    with open(f'/tmp/tune_{name}.log') as f:
        for line in f:
            if 'user interrupted' in line:
                break
            m = re.search(r'\[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)', line)
            if m:
                died.append(f'{m.group(1)} (exit code {m.group(2)})')
    if died:
        raise RuntimeError(f'process died during the replay: {", ".join(died)} (see /tmp/tune_{name}.log)')
    return out_bag


def main():
    in_bag, name, params_json = sys.argv[1:4]
    extra = sys.argv[4].split() if len(sys.argv) > 4 else []
    out_bag = replay(in_bag, name, json.loads(params_json), extra)
    ndt_only = 'use_ekf:=false' in extra
    print(json.dumps({'name': name, 'estimator': 'ndt_only' if ndt_only else 'ekf_localizer', 'out_bag': out_bag,
                      **metrics(out_bag, '/ndt_pose' if ndt_only else '/ekf_pose')}))


if __name__ == '__main__':
    main()
