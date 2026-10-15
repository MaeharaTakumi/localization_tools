#!/usr/bin/env python3
"""パラメータ推定用の録画（drive の calib）から、観測ノイズ R・オドメトリの倍率・LiDAR の取付をまとめて推定する（コンテナ test 内）.

  python3 calib/estimate_all.py <gazebo|red|blue|white> <bag> [--out <出力のディレクトリ>] [--map <地図の pcd>]

target ごとの値は設定から読む（estimate_noise.py・estimate_calibration.py の既定は Gazebo 用なので、ここで渡す）：
  - 指令のトピック：drive/config/<target>.yaml の command.topic
  - オドメトリのトピック・基準と LiDAR のフレーム・今の倍率 odom_scale：localization_bringup の param/env/<target>.yaml
  - 車輪の間隔（R_omega の計算）：Gazebo は drive/config/gazebo.yaml の tread、実機は whill_odometry の param/odometry.yaml の
    tread_width（env の parameters.whill_odometry にあればそちら）
出力（既定 /root/ros2_ws/fig/calib/<bag 名>/）：noise.*（estimate_noise.py）、calib.*（estimate_calibration.py）、
summary.yaml（env に入れる候補と今の値。env の yaml には書き込まない）。Gazebo は --truth で真値との比較も出す。
"""
import argparse
import json
import math
import os
import subprocess
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path.insert(0, os.path.join(ROOT, 'drive'))
from drive_test import load_config  # noqa: E402


def share(pkg):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('target', help='gazebo・red・blue・white')
    ap.add_argument('bag')
    ap.add_argument('--out', default='', help='出力のディレクトリ（既定 /root/ros2_ws/fig/calib/<bag 名>）')
    ap.add_argument('--map', default='', help='地図の pcd（取付の z を床の高さから求める。参考）')
    args = ap.parse_args()
    bag = os.path.abspath(args.bag.rstrip('/'))
    name = os.path.basename(bag)
    out = args.out or f'/root/ros2_ws/fig/calib/{name}'
    os.makedirs(out, exist_ok=True)

    cfg = load_config(args.target)
    env_path = os.path.join(share('localization_bringup'), 'param', 'env', f'{args.target}.yaml')
    env = yaml.safe_load(open(env_path))
    params = env.get('parameters') or {}
    scale = (params.get('odom_covariance') or {}).get('odom_scale', [1.0, 1.0])
    if args.target == 'gazebo':
        tread, tread_from = float(cfg['tread']), 'drive/config/gazebo.yaml'
    else:
        odo_path = os.path.join(share('whill_odometry'), 'param', 'odometry.yaml')
        tread = yaml.safe_load(open(odo_path))['whill_odometry']['ros__parameters']['tread_width']
        tread_from = odo_path
        if 'tread_width' in (params.get('whill_odometry') or {}):
            tread, tread_from = params['whill_odometry']['tread_width'], env_path
    common = ['--odom', env['topics']['odom'], '--cmd', cfg['command']['topic'],
              '--base', env['base_frame_id'], '--lidar', env['lidar_frame_id']]
    truth = ['--truth'] if args.target == 'gazebo' else []
    sc = f'{scale[0]},{scale[1]}'
    print(f'== {args.target}: env {env_path}, odom_scale {scale}, tread {tread} m ({tread_from})', flush=True)

    py = sys.executable
    print('\n== observation noise (noise/estimate_noise.py)', flush=True)
    subprocess.run([py, os.path.join(ROOT, 'noise', 'estimate_noise.py'), bag, '--out', f'{out}/noise',
                    '--scale', sc, '--tread', str(tread)] + common + truth, check=True)
    print('\n== odometry scale and LiDAR mount (calib/estimate_calibration.py)', flush=True)
    subprocess.run([py, os.path.join(ROOT, 'calib', 'estimate_calibration.py'), bag, '--out', f'{out}/calib',
                    '--current-scale', sc] + common + truth + (['--map', args.map] if args.map else []), check=True)

    noise = json.load(open(f'{out}/noise.json'))
    cal = json.load(open(f'{out}/calib.json'))
    now = cal['mount_now']

    def r3(x):
        return float(f'{x:.3g}')

    def r4(x):
        return round(float(x), 4)

    def r6(x):   # 角度 [rad]
        return round(float(x), 6)

    new_scale = [r4(cal.get('v_scale', scale[0])), r4(cal.get('omega_scale', scale[1]))]
    summary = {
        'bag': bag, 'target': args.target,
        'candidate': {   # env（param/env/<target>.yaml）の parameters に入れる候補
            'odom_covariance': {'R_odom': [r3(x) for x in noise['R_odom']], 'odom_scale': new_scale},
            'ekf_localizer': {'R_ndt': [r3(x) for x in noise['R_ndt']]},
        },
        'current': {
            'odom_covariance': {'R_odom': params.get('odom_covariance', {}).get('R_odom'), 'odom_scale': scale},
            'ekf_localizer': {'R_ndt': params.get('ekf_localizer', {}).get('R_ndt', '(ekf.yaml)')},
        },
        'mount': {   # base -> LiDAR（実機は env の static_tfs、Gazebo は URDF）
            'estimated': {'x': r4(cal['mount_xy'][0]) if 'mount_xy' in cal else None,
                          'y': r4(cal['mount_xy'][1]) if 'mount_xy' in cal else None,
                          'z': r4(cal['mount_z']) if 'mount_z' in cal else None,
                          'roll': r6(cal['mount_roll_pitch'][0]) if 'mount_roll_pitch' in cal else None,
                          'pitch': r6(cal['mount_roll_pitch'][1]) if 'mount_roll_pitch' in cal else None,
                          'yaw': r6(cal['mount_yaw']) if 'mount_yaw' in cal else None},
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
        f.write(f'# calib/estimate_all.py {args.target} {bag}\n# env の yaml には書き込んでいない。candidate を確かめてから入れる\n')
        yaml.safe_dump(summary, f, sort_keys=False, allow_unicode=True)

    print(f'\n== summary ({out}/summary.yaml)')
    c = summary['candidate']
    print(f"  R_odom      {c['odom_covariance']['R_odom']}   now {summary['current']['odom_covariance']['R_odom']}")
    print(f"  odom_scale  {new_scale}   now {scale}")
    print(f"  R_ndt       {c['ekf_localizer']['R_ndt']}")
    est, cur = summary['mount']['estimated'], summary['mount']['current']
    for k in ('x', 'y', 'z'):
        if est[k] is not None:
            print(f'  mount {k:<5} {est[k]:+.4f} m   now {cur[k]:+.4f}')
    for k in ('roll', 'pitch', 'yaw'):
        if est[k] is not None:
            print(f'  mount {k:<5} {math.degrees(est[k]):+.3f} deg   now {math.degrees(cur[k]):+.3f}')
    print(f'  figures: {out}/noise_hist.png, noise_segments.png, noise_summary.png, calib.png')


if __name__ == '__main__':
    main()
