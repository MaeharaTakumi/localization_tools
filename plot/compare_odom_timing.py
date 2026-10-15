#!/usr/bin/env python3
"""bag のオドメトリ（diff_drive の /wheelchair/odom と、車輪の角速度からの /localization/wheel_odom）の、
真値に対する遅れ（変化の半分に達した時刻の差）と、回転中の omega の平均・ばらつきを出す（コンテナ test 内で実行）.

  python3 compare_odom_timing.py <bag>
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import read_bag  # noqa: E402
from plot_velocity import half_time, odom_series, phases_of, truth_velocity  # noqa: E402


def main():
    bag = sys.argv[1]
    d = read_bag(bag)
    ph = phases_of(bag)
    tt, tv, tw = truth_velocity(d)
    series = [('/wheelchair/odom (0.2 s average)', odom_series(d['/wheelchair/odom']), 0.0),
              ('/wheelchair/odom, stamp - 0.09 s', odom_series(d['/wheelchair/odom']), 0.09),
              ('joint_states -> /localization/wheel_odom', odom_series(d['/localization/wheel_odom']), 0.0)]
    cols = (('left turn', 'w', 'start w'), ('right turn', 'w', 'reverse w'), ('forward', 'v', 'fwd v'),
            ('forward', 'w', 'fwd w'), ('stop', 'v', 'stop v'))
    print(f"{'delay vs truth [ms]':<42}" + ''.join(f'{c[2]:>11}' for c in cols))
    for name, (t, v, w), shift in series:
        t = t - shift
        cells = []
        for pkey, key, _ in cols:
            got = half_time(t, v if key == 'v' else w, ph[pkey])
            ref = half_time(tt, tv if key == 'v' else tw, ph[pkey])
            cells.append((got - ref) * 1000.0)
        print(f'{name:<42}' + ''.join(f'{c:11.0f}' for c in cells))
    print()
    for name, t, w in (('truth', tt, tw), (series[0][0], series[0][1][0], series[0][1][2]),
                       (series[2][0], series[2][1][0], series[2][1][2])):
        m = (t > ph['left turn'] + 1.5) & (t < ph['right turn'] - 0.2)
        print(f'{name:<42} left turn omega mean {w[m].mean():.3f}, std {w[m].std():.4f} rad/s')


if __name__ == '__main__':
    main()
