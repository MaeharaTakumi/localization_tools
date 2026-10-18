#!/usr/bin/env python3
"""オドメトリと真値の速度（omega か v）の時間推移を 1 枚に描く（コンテナ test 内で実行。2026-10-19、ユーザー指示）.

  python3 plot_odom_truth.py <bag> <出力 png> <omega|v> [<題>]

drive/profiles_velocity.yaml の動き（前後に止まる区間 stop_before・stop_after）の録画を想定する。
真値は /wheelchair/pose（base_footprint）を ±0.03 s の差分で微分したもの（plot_velocity.py の truth_velocity。
v は車体の前方向成分）、オドメトリは /wheelchair/odom を stamp の時刻で描く。横軸は動き出しの指令からの時間（sim time）。
動いている区間の、動き出しの 2 s 後から止まる指令までの平均（定常値）と、その比（真値 / オドメトリ）を図に書き、表示もする。
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'calibration')]
from plot_noise import box_on  # noqa: E402
from plot_spin import capitalize_labels, GRID, INK, INK2, read_bag, SURFACE, TRUTH_COLOR, TRUTH_LW  # noqa: E402
from plot_velocity import C_ODOM, odom_series, phases_of, truth_velocity  # noqa: E402

TOPICS = ['/clock', '/wheelchair/pose', '/wheelchair/odom']
SETTLE = 2.0   # 定常値の平均を取り始める、動き出しからの時間 [s]


def main():
    if len(sys.argv) < 4 or sys.argv[3] not in ('omega', 'v'):
        raise SystemExit(__doc__)
    bag, out, key = sys.argv[1], sys.argv[2], sys.argv[3]
    title = sys.argv[4] if len(sys.argv) > 4 else os.path.basename(bag.rstrip('/'))
    d = read_bag(bag, TOPICS)
    ph = phases_of(bag)
    t0, t1 = ph['stop_before_end'], ph['stop_after_start']   # 動き出し・止まる指令の時刻
    tt, tv, tw = truth_velocity(d)
    ot, ov, ow = odom_series(d['/wheelchair/odom'])
    truth, odom = (tw, ow) if key == 'omega' else (tv, ov)

    def steady(t, y):
        m = (t >= t0 + SETTLE) & (t <= t1)
        return float(np.mean(y[m])), float(np.std(y[m]))

    (tm, ts), (om, os_) = steady(tt, truth), steady(ot, odom)
    print(f'{os.path.basename(bag.rstrip("/"))} {key}: truth {tm:.4f} (sd {ts:.4f}), odometry {om:.4f} (sd {os_:.4f}), '
          f'truth / odometry {tm / om:.4f}, moving {t1 - t0:.2f} s')

    plt.rcParams.update({
        'font.size': 10, 'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2, 'text.color': INK,
        'axes.titlecolor': INK, 'axes.titlesize': 11, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left'})
    fig, ax = plt.subplots(figsize=(10, 4.2), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    lo, hi = -2.0, t1 - t0 + 3.0
    # 後方（先に描く）ほど太く：真値（一番後ろの太い薄灰色）→ オドメトリ
    for t, y, color, lw, z in ((tt, truth, TRUTH_COLOR, TRUTH_LW, 2), (ot, odom, C_ODOM, 1.4, 3)):
        m = (t - t0 >= lo) & (t - t0 <= hi)
        ax.plot(t[m] - t0, y[m], color=color, lw=lw, zorder=z)
    unit = 'rad/s' if key == 'omega' else 'm/s'
    handles = [plt.Line2D([], [], color=TRUTH_COLOR, lw=TRUTH_LW, label='Truth (d/dt of /wheelchair/pose)'),
               plt.Line2D([], [], color=C_ODOM, lw=1.4, label='Odometry (/wheelchair/odom)')]
    ax.legend(handles=handles, loc='lower center', frameon=False, ncol=2)
    ax.text(0.5, 0.30,   # 動いている間の真ん中の高さは線が無い（立ち上がり・停止の線とも重ならない）
            f'Mean from {SETTLE:.0f} s after start to stop command:\n'
            f'truth {tm:.3f} {unit} (sd {ts:.3f}), odometry {om:.3f} {unit} (sd {os_:.3f})\n'
            f'truth / odometry = {tm / om:.3f}',
            transform=ax.transAxes, ha='center', va='bottom', fontsize=9, color=INK2)
    ax.set_xlim(lo, hi)
    ax.set_xlabel('Time from start command [s] (sim time)')
    ax.set_ylabel('Angular velocity [rad/s]' if key == 'omega' else 'Linear velocity [m/s]')
    ax.set_title(title)
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    box_on(ax)
    fig.tight_layout()
    capitalize_labels(fig)
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {out}')


if __name__ == '__main__':
    main()
