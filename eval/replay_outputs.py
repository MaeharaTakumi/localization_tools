#!/usr/bin/env python3
"""録画を頭から最後まで再生して推定器を動かし、MATLAB 用の出力も録る（plot/plot_matlab_outputs.py で確かめる）.

  python3 replay_outputs.py <入力 bag> <試行名>                       # Gazebo の録画
  python3 replay_outputs.py <入力 bag> <試行名> <環境の名前> <地図>    # 実機の録画（環境は red・blue・white）

出力は入力と同じ側の tune/<試行名>（bag_paths.tune_dir）。ROS_DOMAIN_ID=99 で動かす（動いている推定とは混ざらない）。
  Gazebo：tune_run.replay（gazebo.yaml と URDF の取付、入力は /clock を含む IN_TOPICS）で、再生する範囲だけ録画全体にする。
  実機  ：環境ファイルの写し（use_sim_time: true）で起動し、点群とオドメトリ（whill_odometry の motor_speed）を
          --clock で流す。RATE（環境変数、既定 1.0）で再生の速さを変えられる。
RECORD=0（環境変数）なら出力を録らない（Gazebo は tune_run が /clock だけ録る。MATLAB で確かめるとき。matlab/replay_matlab.sh）。
大きい点群をコンテナの中で流すと、受け取る側で点群が落ちることがある（実機 blue の録画では NDT が解いたのは
半分ほど。ソケットの受信バッファ 208 kB に 314 kB の点群が収まらないため。ホストの net.core.rmem_max・rmem_default を
8 MB にすると 1527 個中 1515 個を解く。dev_log.md の 61・66・67）。
"""
import os
import subprocess
import sys
import time

import rosbag2_py
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_paths import source_of, tune_dir  # noqa: E402
import tune_run  # noqa: E402

OUTPUTS = ['/tf_static', '/tf_ndt_update', '/ndt_pose', '/ekf_pose', '/localization/base_pose',
           '/localization/vlp16_pose', '/localization/vlp16_points']
ENV_DIR = '/root/ros2_ws/install/localization_bringup/share/localization_bringup/param/env'


def bag_duration(bag):
    return rosbag2_py.Info().read_metadata(bag, 'sqlite3').duration.nanoseconds / 1e9


def record_outputs():
    return os.environ.get('RECORD', '1') != '0'


def replay_gazebo(in_bag, name):
    tune_run.OUT_TOPICS = sorted(set(tune_run.OUT_TOPICS) | set(OUTPUTS)) if record_outputs() else ['/clock']
    tune_run.play_window = lambda bag, phases: (0.0, bag_duration(bag))
    return tune_run.replay(in_bag, name, {})


def replay_real(in_bag, name, env_name, map_path):
    out_bag = os.path.join(tune_dir(in_bag), name)
    os.makedirs(tune_dir(in_bag), exist_ok=True)
    subprocess.run(['rm', '-rf', out_bag])
    phases = in_bag.rstrip('/') + '_phases.txt'
    if os.path.exists(phases) and record_outputs():
        subprocess.run(['cp', phases, out_bag + '_phases.txt'])
    with open(os.path.join(ENV_DIR, env_name + '.yaml')) as f:
        env = yaml.safe_load(f)
    env['use_sim_time'] = True   # 録画の時刻（--clock）で動かす
    env_path = f'/tmp/replay_env_{name}.yaml'
    with open(env_path, 'w') as f:
        yaml.safe_dump(env, f)
    inputs = ['/velodyne_points', env['nodes']['whill_odometry']['topics']['motor_speed']]

    penv = dict(os.environ, ROS_DOMAIN_ID='99')
    log_path = f'/tmp/replay_{name}.log'
    log = open(log_path, 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'localization_bringup', 'localization.launch.py', f'env:={env_path}',
         f'map_path:={map_path}', 'rviz:=false'],
        env=penv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    record = None
    if record_outputs():
        record = subprocess.Popen(['ros2', 'bag', 'record', '-o', out_bag, '/clock', *OUTPUTS],
                                  env=penv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(10.0)   # 地図の読み込み
        rate = float(os.environ.get('RATE', '1.0'))
        subprocess.run(['ros2', 'bag', 'play', in_bag, '--clock', '200', '--rate', str(rate), '--topics', *inputs],
                       env=penv, stdout=log, stderr=subprocess.STDOUT, timeout=bag_duration(in_bag) / rate + 30)
        time.sleep(2.0)
    finally:
        tune_run.stop(launch)
        if record is not None:
            tune_run.stop(record)
        log.close()
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
