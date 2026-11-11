#!/usr/bin/env python3
"""録画を頭から最後まで再生して推定器を動かし、MATLAB 用の出力も録る（plot/plot_matlab_outputs.py で確かめる）.

  python3 replay_outputs.py <入力 bag> <試行名>                       # Gazebo の録画
  python3 replay_outputs.py <入力 bag> <試行名> <環境の名前> <地図>    # 実機の録画（環境は red・blue・white）

出力は入力と同じ側の tune/<試行名>（bag_paths.tune_dir）。ROS_DOMAIN_ID=99 で動かす（動いている推定とは混ざらない）。
  Gazebo：tune_run.replay（gazebo.yaml と URDF の取付、入力は /clock を含む IN_TOPICS）で、録画全体を再生する（whole_bag=True）。
  実機  ：環境（param/<env>/）の写し（/tmp/replay_env_<試行名>/、use_sim_time: true）で起動し、点群とオドメトリ（whill_odometry の motor_speed）を
          --clock で流す（起動・録画・再生・停止の流れは Gazebo と同じ tune_run.launch_and_play）。RATE（環境変数、既定 1.0）で再生の速さを変えられる。
PARAMS（環境変数、JSON）でノードのパラメータを上書きする（例 '{"ekf_localizer": {"tf_future_steps": 2}}'。
tune_run.py の parameters と同じ形で、写した環境の <ノード名>.yaml に足す）。
RECORD=0（環境変数）なら出力を録らない（Gazebo は tune_run が /clock だけ録る。MATLAB で確かめるとき。matlab/replay_matlab.sh）。
大きい点群をコンテナの中で流すと、受け取る側で点群が落ちることがある（実機 blue の録画では NDT が解いたのは
半分ほど。ソケットの受信バッファ 208 kB に 314 kB の点群が収まらないため。ホストの net.core.rmem_max・rmem_default を
8 MB にすると 1527 個中 1515 個を解く。dev_log.md の 61・66・67）。
"""
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'eval')]
from bag_paths import source_of, tune_dir  # noqa: E402
from env_files import make_env, node_params  # noqa: E402
import tune_run  # noqa: E402

OUTPUTS = ['/tf', '/tf_static', '/ndt_pose', '/ekf_pose', '/localization_debug']


def params_from_env():
    return json.loads(os.environ.get('PARAMS') or '{}')


def record_outputs():
    return os.environ.get('RECORD', '1') != '0'


def replay_gazebo(in_bag, name):
    out_topics = sorted(set(tune_run.OUT_TOPICS) | set(OUTPUTS)) if record_outputs() else ['/clock']
    return tune_run.replay(in_bag, name, params_from_env(), out_topics=out_topics, whole_bag=True)


def replay_real(in_bag, name, env_name, map_path):
    out_bag = os.path.join(tune_dir(in_bag), name)
    os.makedirs(tune_dir(in_bag), exist_ok=True)
    subprocess.run(['rm', '-rf', out_bag])
    phases = in_bag.rstrip('/') + '_phases.txt'
    if os.path.exists(phases) and record_outputs():
        subprocess.run(['cp', phases, out_bag + '_phases.txt'])
    # 録画の時刻（--clock）で動かす
    env = make_env(env_name, f'/tmp/replay_env_{name}', {'use_sim_time': True}, params_from_env())
    inputs = ['/velodyne_points', node_params(env, 'whill_odometry')['input_topic_name']]
    rate = float(os.environ.get('RATE', '1.0'))

    def play():
        return ['--clock', '200', '--rate', str(rate), '--topics', *inputs], tune_run.bag_duration(in_bag) / rate

    log_path = tune_run.launch_and_play(
        name, env, in_bag, out_bag, ['/clock', *OUTPUTS] if record_outputs() else [], play,
        launch_args=[f'map_path:={map_path}'], tag='replay')
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
