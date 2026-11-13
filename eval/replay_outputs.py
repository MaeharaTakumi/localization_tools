#!/usr/bin/env python3
"""録画を頭から最後まで再生して推定器を動かし、MATLAB 用の出力も録る（plot/plot_matlab_outputs.py で確かめる）.

  python3 replay_outputs.py <入力 bag> <試行名>                       # Gazebo の録画
  python3 replay_outputs.py <入力 bag> <試行名> <環境の名前> <地図>    # 実機の録画（環境は red・blue・white）

出力は入力と同じ側の tune/<試行名>（bag_paths.tune_dir）。ROS_DOMAIN_ID=99 で動かす（動いている推定・Gazebo・MATLAB とは混ざらない）。
  Gazebo：環境 param/gazebo/ の写し（/tmp/replay_env_<試行名>/。/tf_static が流れないので URDF と同じ取付を static_tfs に書く）で起動し、
          録画の /clock・点群・オドメトリ・joint_states（IN_TOPICS）を頭から最後まで流す。
  実機  ：環境（param/<env>/）の写し（use_sim_time: true）で起動し、点群とオドメトリ（whill_odometry の motor_speed）を
          --clock で流す。RATE（環境変数、既定 1.0）で再生の速さを変えられる。
  起動・録画・再生・停止の流れは launch_and_play の 1 つ。ノードが再生の途中で落ちた回はエラーにする。
PARAMS（環境変数、JSON）でノードのパラメータを上書きする（{ノード名: {パラメータ: 値}}。ノードは ekf_localizer・pcl_localization・
whill_odometry。写した環境の <ノード名>.yaml に足す。キー "__env__" の中身は env.yaml の最上位を上書きする。例 '{"ekf_localizer": {"tf_future_steps": 2}}'）。
LAUNCH_ARGS（環境変数、空白区切り）で launch の引数を足す（例 'matcher:=lidarslam' で NDT の代わりに lidarslam。'use_ekf:=false'）。
RECORD=0（環境変数）なら出力を録らない（Gazebo は /clock だけ録る。MATLAB で確かめるとき。matlab/replay_matlab.sh）。
大きい点群をコンテナの中で流すと、受け取る側で点群が落ちることがある（実機 blue の録画では NDT が解いたのは
半分ほど。ソケットの受信バッファ 208 kB に 314 kB の点群が収まらないため。ホストの net.core.rmem_max・rmem_default を
8 MB にすると 1527 個中 1515 個を解く。dev_log.md の 61・66・67）。
"""
import json
import os
import re
import signal
import subprocess
import sys
import time

import rosbag2_py

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'eval')]
from bag_io import phases_of  # noqa: E402
from bag_paths import source_of, tune_dir  # noqa: E402
from env_files import make_env, node_params  # noqa: E402
from gazebo_model import URDF_TFS  # noqa: E402

# 録る出力（推定の姿勢は /tf の map -> base。/ekf_pose は 2026-11-13 から録らない）
OUTPUTS = ['/tf', '/tf_static', '/ndt_pose', '/slam_pose', '/ekf_odom', '/localization_debug']
IN_TOPICS = ['/clock', '/velodyne_points', '/wheelchair/odom', '/wheelchair/joint_states']   # Gazebo の録画から流すもの


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


def launch_and_play(name, env_path, in_bag, out_bag, record_topics, play, launch_args=()):
    """環境のフォルダ env_path で自己位置推定を起動し、in_bag を再生して record_topics を out_bag に録る。ログのパスを返す.

    ROS_DOMAIN_ID=99 で動かす。play() は地図の読み込みを待ったあとに呼ばれ、(ros2 bag play の追加の引数, 再生にかかる時間 [s]) を返す。
    record_topics が空なら録らない。ログは /tmp/replay_<name>.log。ノードが再生の途中で落ちたらエラーにする（止める合図
    'user interrupted' より後に落ちたもの（止めるときの SIGINT や、終了時のまれなセグメンテーション違反）は除く）。
    """
    penv = dict(os.environ, ROS_DOMAIN_ID='99')
    log_path = f'/tmp/replay_{name}.log'
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
    return log_path


def prepare_out(in_bag, name):
    """出力の bag のパス（前の出力は消す）。録画の _phases.txt があれば隣に写す."""
    out_bag = os.path.join(tune_dir(in_bag), name)
    os.makedirs(tune_dir(in_bag), exist_ok=True)
    subprocess.run(['rm', '-rf', out_bag])
    if os.path.exists(in_bag.rstrip('/') + '_phases.txt') and record_outputs():
        with open(out_bag + '_phases.txt', 'w') as f:
            f.writelines(f'{k} {v:.3f}\n' for k, v in phases_of(in_bag).items())
    return out_bag


def params_from_env():
    return json.loads(os.environ.get('PARAMS') or '{}')


def launch_args():
    return os.environ.get('LAUNCH_ARGS', '').split()


def record_outputs():
    return os.environ.get('RECORD', '1') != '0'


def replay_gazebo(in_bag, name):
    out_bag = prepare_out(in_bag, name)
    params = params_from_env()
    env_values = {'static_tfs': URDF_TFS, **(params.pop('__env__', None) or {})}
    env = make_env('gazebo', f'/tmp/replay_env_{name}', env_values, params)

    def play():
        return ['--topics', *IN_TOPICS], bag_duration(in_bag)

    log_path = launch_and_play(name, env, in_bag, out_bag, ['/clock', *OUTPUTS] if record_outputs() else ['/clock'],
                               play, launch_args=launch_args())
    print(f'log: {log_path}')
    return out_bag


def replay_real(in_bag, name, env_name, map_path):
    out_bag = prepare_out(in_bag, name)
    # 録画の時刻（--clock）で動かす
    params = params_from_env()
    env_values = {'use_sim_time': True, **(params.pop('__env__', None) or {})}
    env = make_env(env_name, f'/tmp/replay_env_{name}', env_values, params)
    inputs = ['/velodyne_points', node_params(env, 'whill_odometry')['input_topic_name']]
    rate = float(os.environ.get('RATE', '1.0'))

    def play():
        return ['--clock', '200', '--rate', str(rate), '--topics', *inputs], bag_duration(in_bag) / rate

    log_path = launch_and_play(name, env, in_bag, out_bag, ['/clock', *OUTPUTS] if record_outputs() else [], play,
                               launch_args=[f'map_path:={map_path}', *launch_args()])
    print(f'log: {log_path}')
    return out_bag


def main():
    if len(sys.argv) not in (3, 5):
        print(__doc__)
        sys.exit(1)
    in_bag, name = sys.argv[1:3]
    if source_of(in_bag) == 'real':
        if len(sys.argv) != 5:
            sys.exit('real bags need <env name> <map>')
        print(replay_real(in_bag, name, *sys.argv[3:5]))
    else:
        print(replay_gazebo(in_bag, name))


if __name__ == '__main__':
    main()
