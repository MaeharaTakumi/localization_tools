#!/usr/bin/env python3
"""録画（bag）の情報ファイル <bag>_info.yaml を書く・一覧を出す（localizer_rl 内で実行）.

  python3 bag_info.py write <bag> --source gazebo|real [--drive ...] [--purpose ...] [--notes ...]
                                  [--estimator ...] [--odom-window N]
  python3 bag_info.py list [--source gazebo|real]

情報ファイル（bag のディレクトリの隣）：
  source       gazebo（Gazebo のシミュレーション）か real（実機）
  recorded     録画の開始時刻（bag のメタデータ、ローカル時刻）
  duration_s・size_mb・topics（トピックごとの件数）
  estimator    録画したときに動いていた推定器（出力のトピックから判定。再生では使わない）
  odom_window  Gazebo のオドメトリの移動平均の個数（diff_drive_controller の velocity_rolling_window_size）。
               /wheelchair/joint_states の車輪の角速度から計算した omega に 1 個・10 個の移動平均を掛け、
               /wheelchair/odom の omega に近いほうを選ぶ（判定できなければ unknown）
  odom_omega_ratio  オドメトリの omega / 車輪から計算した omega（回転中）。1 なら Gazebo の wheel_separation_multiplier は 1
  odom_truth_omega_ratio  オドメトリの omega / 真値の omega（回転中、Gazebo）。滑りのため普段は約 1.085
  drive・purpose・notes  走らせ方・用途・メモ（指定したもの）
既にある情報ファイルを書き直すときは、指定しなかった drive・purpose・notes は前の値を残す。
"""
import argparse
import datetime
import glob
import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]
from bag_paths import SOURCES, source_dir  # noqa: E402

WHEEL_RADIUS, WHEEL_SEPARATION = 0.1325, 0.492   # Gazebo の URDF（diff_drive_controller と同じ）
ESTIMATOR_TOPICS = [('/odometry/global', 'robot_localization'), ('/ekf_odom', 'ekf_localizer')]


def info_path(bag):
    return bag.rstrip('/') + '_info.yaml'


def metadata(bag):
    import rosbag2_py
    m = rosbag2_py.Info().read_metadata(bag, 'sqlite3')
    start = datetime.datetime.fromtimestamp(m.starting_time.nanoseconds * 1e-9)   # rclpy の Time
    topics = {t.topic_metadata.name: t.message_count for t in m.topics_with_message_count}
    size = sum(os.path.getsize(f) for f in glob.glob(os.path.join(bag, '*')) if os.path.isfile(f))
    return {'recorded': start.strftime('%Y-%m-%d %H:%M:%S'),
            'duration_s': round(m.duration.nanoseconds * 1e-9, 1),
            'size_mb': round(size / 1e6, 1),
            'topics': dict(sorted(topics.items()))}


def estimator_of(topics):
    for topic, name in ESTIMATOR_TOPICS:
        if topics.get(topic):
            return name
    return 'ndt_only' if topics.get('/ndt_pose') else 'none'


def odom_check(bag):
    """{odom_window, odom_omega_ratio, odom_truth_omega_ratio}.

    odom_window：車輪の角速度から計算した omega（直進だけなら v）に 1 個・10 個の移動平均を掛け、
    /wheelchair/odom に近いほう。odom_omega_ratio：オドメトリの omega / 車輪から計算した omega（回転中）。
    odom_truth_omega_ratio：オドメトリの omega / 真値の omega（Gazebo の /wheelchair/pose の微分、回転中。
    滑りのため普段は約 1.085、wheel_separation_multiplier 1.085 を試した録画では約 1.0）。
    """
    from plot_spin import read_bag
    from plot_velocity import truth_velocity
    d = read_bag(bag, ['/clock', '/wheelchair/odom', '/wheelchair/joint_states', '/wheelchair/pose'])
    out = {'odom_window': 'unknown', 'odom_omega_ratio': None, 'odom_truth_omega_ratio': None}
    od = d.get('/wheelchair/odom', [])
    if len(od) < 100:
        return out

    def st(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    ot = np.array([st(m) for _, m in od])
    sig_odom = {'omega': np.array([m.twist.twist.angular.z for _, m in od]),
                'v': np.array([m.twist.twist.linear.x for _, m in od])}

    def fit(pred, obs, moving):
        k = float(np.dot(pred[moving], obs[moving]) / np.dot(pred[moving], pred[moving]))
        return k, float(np.sqrt(np.mean((obs - k * pred)[moving] ** 2)))

    # 真値との omega の比（Gazebo の真値があるとき）
    if d.get('/wheelchair/pose') and d.get('/clock'):
        tt, _, tw = truth_velocity(d)
        tw_o = np.interp(ot, tt, tw)
        moving = np.abs(tw_o) > 0.2
        if moving.sum() >= 50:
            out['odom_truth_omega_ratio'] = round(fit(tw_o, sig_odom['omega'], moving)[0], 3)

    # 車輪の角速度との比較
    jt, jsig = [], {'omega': [], 'v': []}
    for _, m in d.get('/wheelchair/joint_states', []):
        if 'left_wheel_joint' not in m.name or 'right_wheel_joint' not in m.name or not m.velocity:
            continue
        wl = m.velocity[m.name.index('left_wheel_joint')]
        wr = m.velocity[m.name.index('right_wheel_joint')]
        jt.append(st(m))
        jsig['omega'].append(WHEEL_RADIUS * (wr - wl) / WHEEL_SEPARATION)
        jsig['v'].append(WHEEL_RADIUS * (wr + wl) / 2.0)
    if len(jt) < 100:
        return out
    o = np.argsort(jt)
    jt = np.array(jt)[o]
    for key in ('omega', 'v'):   # 回転があれば omega、直進だけなら v で判定する
        obs = sig_odom[key]
        moving = np.abs(obs) > 0.2
        if moving.sum() < 50:
            continue
        sig = np.array(jsig[key])[o]
        best = None
        for n in (1, 10):
            avg = np.convolve(sig, np.ones(n) / n)[:len(sig)]   # 直近 n 個の平均（diff_drive_controller と同じ向き）
            # 倍率（wheel_separation_multiplier）が違っても比べられるよう、最小二乗の倍率を掛けてから差を取る
            k, rms = fit(np.interp(ot, jt, avg), obs, moving)
            if best is None or rms < best[1]:
                best = (n, rms, k)
        out['odom_window'] = best[0]
        if key == 'omega':
            out['odom_omega_ratio'] = round(best[2], 3)
        break
    return out


def write(args):
    bag = args.bag.rstrip('/')
    old = {}
    if os.path.exists(info_path(bag)):
        with open(info_path(bag)) as f:
            old = yaml.safe_load(f) or {}
    meta = metadata(bag)
    check = odom_check(bag) if args.source == 'gazebo' else {}
    if args.odom_window:
        check['odom_window'] = args.odom_window
    if args.source == 'real':   # 実機は whill_odometry（Gazebo の移動平均・真値は無い）
        check = {'odom_window': 'n/a', 'odom_omega_ratio': None, 'odom_truth_omega_ratio': None}
    info = {
        'source': args.source,
        'recorded': meta['recorded'],
        'duration_s': meta['duration_s'],
        'size_mb': meta['size_mb'],
        'drive': args.drive or old.get('drive', 'unknown'),
        'estimator': args.estimator or estimator_of(meta['topics']),
        'odom_window': check.get('odom_window', 'unknown'),
        'odom_omega_ratio': check.get('odom_omega_ratio'),
        'odom_truth_omega_ratio': check.get('odom_truth_omega_ratio'),
        'purpose': args.purpose or old.get('purpose', 'unknown'),
        'notes': args.notes or old.get('notes', ''),
        'topics': meta['topics'],
    }
    with open(info_path(bag), 'w') as f:
        f.write(f'# 録画の情報（localization_tools/eval/bag_info.py が書いた。{datetime.datetime.now():%Y-%m-%d %H:%M}）\n')
        yaml.safe_dump(info, f, sort_keys=False, allow_unicode=True)
    print(f"saved {info_path(bag)}: {info['source']}, {info['recorded']}, odom_window {info['odom_window']}, "
          f"ratio wheels {info['odom_omega_ratio']} truth {info['odom_truth_omega_ratio']}, estimator {info['estimator']}")


def list_bags(args):
    rows = []
    for source in ([args.source] if args.source else SOURCES):
        for path in sorted(glob.glob(os.path.join(source_dir(source), '*_info.yaml'))):
            with open(path) as f:
                i = yaml.safe_load(f)
            rows.append((source, os.path.basename(path)[:-len('_info.yaml')], i))
    print(f"{'source':<7} {'name':<20} {'recorded':<17} {'dur[s]':>7} {'MB':>5} {'odom_w':>7} {'w/truth':>7} "
          f"{'estimator':<19} purpose")
    for source, name, i in rows:
        print(f"{source:<7} {name:<20} {i['recorded'][:16]:<17} {i['duration_s']:>7.1f} {i['size_mb']:>5.0f} "
              f"{str(i['odom_window']):>7} {str(i.get('odom_truth_omega_ratio')):>7} {i['estimator']:<19} {i['purpose']}")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    w = sub.add_parser('write')
    w.add_argument('bag')
    w.add_argument('--source', required=True, choices=SOURCES)
    w.add_argument('--drive', default='')
    w.add_argument('--purpose', default='')
    w.add_argument('--notes', default='')
    w.add_argument('--estimator', default='')
    w.add_argument('--odom-window', type=int, default=0, help='判定せずに指定する（0 なら録画から判定）')
    lst = sub.add_parser('list')
    lst.add_argument('--source', choices=SOURCES)
    args = ap.parse_args()
    write(args) if args.cmd == 'write' else list_bags(args)


if __name__ == '__main__':
    main()
