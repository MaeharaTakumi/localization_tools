#!/usr/bin/env python3
"""パラメータ推定用の録画（calibration/run_calibration.sh）から、観測ノイズ R・オドメトリの倍率・LiDAR の取付をまとめて推定する（コンテナ test 内）.

  python3 calibration/calibrate.py <gazebo|red|blue|white> <bag> [--out <出力のディレクトリ>] [--map <地図の pcd>]

推定の設定（区間の取り方・窓など）は calibration/param/calibrate.yaml（各プログラムの既定値）。
target ごとの値は設定から読む（calibrate_noise.py・calibrate_mount_scale.py の既定は Gazebo 用なので、ここで渡す）：
  - 指令のトピック：drive/config/<target>.yaml の command.topic
  - オドメトリのトピック：/wheelchair/odom（localization_bringup の launch が ekf_localizer の odom をここにつなぐ）
  - 基準と LiDAR のフレーム・今の倍率 odom_scale：localization_bringup の param/<target>/ekf_localizer.yaml
  - 車輪の間隔（R_omega の計算）：Gazebo は calibrate.yaml の noise.gazebo_tread、実機は param/<target>/whill_odometry.yaml の tread_width
  書いていない項目はコードの既定値（odom_scale [1, 1]、base_link、velodyne、tread_width 0.495）
R は R_odom・R_ndt とも走行中の残差（前後 window/2 秒の滑らかな動きからのばらつき）から求める（calibrate_noise.py）。
出力（既定 /root/ros2_ws/fig/calibration/<bag 名>/）：noise.*（calibrate_noise.py）、mount_scale.*・mount_scale_yaw.png
（calibrate_mount_scale.py）、summary.yaml（param/<target>/ekf_localizer.yaml などに入れる候補と今の値。yaml には書き込まない）。
Gazebo は、録画に真値（/wheelchair/pose）があれば --truth で真値との比較も出す（2026-11-13 から Gazebo の録画に真値を入れないので、
それより後の録画では出ない）。
"""
import argparse
import json
import os
import subprocess
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'drive', 'calibration')]
from drive_test import load_config  # noqa: E402
from env_files import env_dir, node_params  # noqa: E402
import params  # noqa: E402

ODOM_TOPIC = '/wheelchair/odom'   # localization_bringup の launch が ekf_localizer の odom をここにつなぐ


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('target', help='gazebo・red・blue・white')
    ap.add_argument('bag')
    ap.add_argument('--out', default='', help='出力のディレクトリ（既定 /root/ros2_ws/fig/calibration/<bag 名>）')
    ap.add_argument('--map', default='', help='地図の pcd（取付の z を床の高さから求める。参考）')
    args = ap.parse_args()
    bag = os.path.abspath(args.bag.rstrip('/'))
    name = os.path.basename(bag)
    out = args.out or f'/root/ros2_ws/fig/calibration/{name}'
    os.makedirs(out, exist_ok=True)

    cfg = load_config(args.target)
    settings = params.settings()
    ekf = node_params(args.target, 'ekf_localizer')
    scale = ekf.get('odom_scale', [1.0, 1.0])
    if args.target == 'gazebo':
        tread, tread_from = float(settings['noise']['gazebo_tread']), 'calibration/param/calibrate.yaml'
    else:
        tread = node_params(args.target, 'whill_odometry').get('tread_width', 0.495)
        tread_from = f'{env_dir(args.target)}/whill_odometry.yaml'
    common = ['--odom', ODOM_TOPIC, '--cmd', cfg['command']['topic'],
              '--base', ekf.get('base_frame_id', 'base_link'), '--lidar', ekf.get('lidar_frame_id', 'velodyne')]
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        recorded = {t['topic_metadata']['name'] for t in yaml.safe_load(f)['rosbag2_bagfile_information']['topics_with_message_count']}
    truth = ['--truth'] if args.target == 'gazebo' and '/wheelchair/pose' in recorded else []
    sc = f'{scale[0]},{scale[1]}'
    print(f'== {args.target}: env {env_dir(args.target)}, odom_scale {scale}, tread {tread} m ({tread_from})', flush=True)

    py = sys.executable
    print('\n== observation noise (calibration/calibrate_noise.py)', flush=True)
    subprocess.run([py, os.path.join(ROOT, 'calibration', 'calibrate_noise.py'), bag, '--out', f'{out}/noise',
                    '--scale', sc, '--tread', str(tread)] + common + truth, check=True)
    print('\n== odometry scale and LiDAR mount (calibration/calibrate_mount_scale.py)', flush=True)
    subprocess.run([py, os.path.join(ROOT, 'calibration', 'calibrate_mount_scale.py'), bag, '--out', f'{out}/mount_scale',
                    '--current-scale', sc] + common + truth + (['--map', args.map] if args.map else []), check=True)

    noise = json.load(open(f'{out}/noise.json'))
    mount_scale = json.load(open(f'{out}/mount_scale.json'))
    now = mount_scale['mount_now']

    def r3(x):
        return float(f'{x:.3g}')

    def r4(x):
        return round(float(x), 4)

    def r6(x):   # 角度 [rad]
        return round(float(x), 6)

    new_scale = [r4(mount_scale.get('v_scale', scale[0])), r4(mount_scale.get('omega_scale', scale[1]))]
    summary = {
        'bag': bag, 'target': args.target,
        'candidate': {   # param/<target>/ekf_localizer.yaml に入れる候補
            'ekf_localizer': {'odom_scale': new_scale, 'R_odom': [r3(x) for x in noise['R_odom']],
                              'R_ndt': [r3(x) for x in noise['R_ndt']]},
        },
        'current': {
            'ekf_localizer': {'odom_scale': scale, 'R_odom': ekf.get('R_odom', '(code default)'),
                              'R_ndt': ekf.get('R_ndt', '(code default)')},
        },
        'mount': {   # base -> LiDAR（実機は param/<target>/env.yaml の static_tfs、Gazebo は URDF）
            'calibrated': {'x': r4(mount_scale['mount_xy'][0]) if 'mount_xy' in mount_scale else None,
                          'y': r4(mount_scale['mount_xy'][1]) if 'mount_xy' in mount_scale else None,
                          'z': r4(mount_scale['mount_z']) if 'mount_z' in mount_scale else None,
                          'roll': r6(mount_scale['mount_roll_pitch'][0]) if 'mount_roll_pitch' in mount_scale else None,
                          'pitch': r6(mount_scale['mount_roll_pitch'][1]) if 'mount_roll_pitch' in mount_scale else None,
                          'yaw': r6(mount_scale['mount_yaw']) if 'mount_yaw' in mount_scale else None},
            'current': dict(zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), [r4(x) for x in now[:3]] + [r6(x) for x in now[3:]])),
        },

        'notes': [
            f"R_odom の v は{'直進の' if noise['R_v_from'] == 'straight' else '動いている'}区間の残差、omega は "
            f"4 R_v (s_omega / s_v)^2 / b^2（b {tread} m、今の倍率 {scale}）。直接測った omega の R は "
            f"{noise['R_odom_moving'][1]:.3g}（動いている区間）",
            'R はいまの odom_scale を掛けたオドメトリで求めた。odom_scale を変えるなら、変えてから走り直すと R も合う',
            'roll・pitch は床が水平、z は地図の床が正しいと仮定（参考）。角度は rad',
            '入れる前に録画の再生（eval/tune_run.py など）で確かめる（R を小さくすると NDT の別の解に飛びついて振動することがある）',
        ],
    }
    with open(f'{out}/summary.yaml', 'w') as f:
        f.write(f'# calibration/calibrate.py {args.target} {bag}\n# 環境の yaml には書き込んでいない。candidate を確かめてから入れる\n')
        yaml.safe_dump(summary, f, sort_keys=False, allow_unicode=True)

    print(f'\n== summary ({out}/summary.yaml)')
    c = summary['candidate']
    print(f"  R_odom      {c['ekf_localizer']['R_odom']}   now {summary['current']['ekf_localizer']['R_odom']}")
    print(f"  odom_scale  {new_scale}   now {scale}")
    print(f"  R_ndt       {c['ekf_localizer']['R_ndt']}")
    new, cur = summary['mount']['calibrated'], summary['mount']['current']
    for k in ('x', 'y', 'z'):
        if new[k] is not None:
            print(f'  mount {k:<5} {new[k]:+.4f} m   now {cur[k]:+.4f}')
    for k in ('roll', 'pitch', 'yaw'):
        if new[k] is not None:
            print(f'  mount {k:<5} {new[k]:+.6f} rad   now {cur[k]:+.6f}')   # env の static_tfs の rpy と同じ rad
    print(f'  figures: {out}/noise_hist.png, noise_segments.png, noise_summary.png, mount_scale.png, mount_scale_yaw.png')


if __name__ == '__main__':
    main()
