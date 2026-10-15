#!/usr/bin/env python3
"""Gazebo のオドメトリの「移動平均の変更」と「倍率の補正」の図（コンテナ test 内で実行）.

  python3 plot_odom_filter.py <移動平均 1 の bag> <出力ディレクトリ>

出力（説明の順）:
  odom_window_omega.png  omega（回転開始・反転）：真値、移動平均 10・1（倍率なし）
  odom_window_v.png      v（前進開始・停止）：真値、移動平均 10・1（補足。v の倍率は 1）
  odom_scale_omega.png   omega（回転開始・反転）：真値、移動平均 1 の倍率なし・x 0.922

bag は Gazebo の velocity_rolling_window_size: 1 で録ったもの。移動平均 1 は /wheelchair/odom そのもの、
移動平均 10 は同じ走行の /wheelchair/joint_states の車輪の回転角から diff_drive_controller と同じ計算で求める
（変更前の録画で実物と RMS 0 で一致を確認済み）。
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import (capitalize_labels, GRID, INK, INK2, read_bag, SURFACE, TRUTH_COLOR, TRUTH_LW,  # noqa: E402
                       widths_back_to_front)
from plot_velocity import odom_series, phases_of, truth_velocity  # noqa: E402

R, B, SCALE = 0.1325, 0.492, 0.922
# 色は系列ごとに固定（移動平均 1 は両方の図で同じ色）
WINDOW10, WINDOW1, WINDOW1_SCALED = '#eb6834', '#2a78d6', '#4a3aa7'
TRUTH_LABEL = 'Truth (d/dt of /wheelchair/pose)'
XLABEL = 'Time from command switch [s]'


def rolling_odometry(d, n):
    """diff_drive_controller（position_feedback: true）の v・omega を、窓 n 個の移動平均で再計算する."""
    t, pl, pr = [], [], []
    for _, m in d['/wheelchair/joint_states']:
        il, ir = m.name.index('left_wheel_joint'), m.name.index('right_wheel_joint')
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        pl.append(m.position[il])
        pr.append(m.position[ir])
    o = np.argsort(t)
    t, pl, pr = (np.array(x)[o] for x in (t, pl, pr))
    dt = np.diff(t)
    dl, dr = np.diff(pl) * R, np.diff(pr) * R
    k = np.ones(n) / n
    v = np.convolve((dl + dr) / 2 / dt, k)[:len(dt)]
    w = np.convolve((dr - dl) / B / dt, k)[:len(dt)]
    return t[1 + n - 1:], v[n - 1:], w[n - 1:]


def figure(path, title, truth, series, panels, ylabel, xlim):
    """series: [(ラベル, t, y, 色)]（後方→手前の順）。panels: [(題, 切り替え時刻)]."""
    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': INK2, 'ytick.color': INK2, 'text.color': INK, 'axes.titlecolor': INK,
                         'axes.titlesize': 10, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left'})
    fig, axes = plt.subplots(1, len(panels), figsize=(14, 4.8), facecolor=SURFACE)
    widths = widths_back_to_front(len(series))
    lines = [(TRUTH_LABEL, truth[0], truth[1], TRUTH_COLOR, TRUTH_LW)]
    lines += [(lab, t, y, c, w) for (lab, t, y, c), w in zip(series, widths)]
    for ax, (ptitle, ts) in zip(axes, panels):
        ax.set_facecolor(SURFACE)
        ax.grid(axis='y', color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for spine in ('top', 'right'):
            ax.spines[spine].set_visible(False)
        for i, (lab, t, y, c, w) in enumerate(lines):   # 後方ほど太く、点線は使わない
            m = (t >= ts + xlim[0] - 0.05) & (t <= ts + xlim[1] + 0.05)
            ax.plot(t[m] - ts, y[m], color=c, lw=w, zorder=2 + i, label=lab)
        ax.axvline(0, color=INK2, lw=0.8, zorder=1)
        ax.set_xlim(*xlim)
        ax.set_title(ptitle)
        ax.set_xlabel(XLABEL)
    axes[0].set_ylabel(ylabel)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left', bbox_to_anchor=(0.04, 0.95), ncol=3, frameon=False)
    fig.suptitle(title, x=0.04, ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(path, dpi=130, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {path}')


def main():
    bag, out = sys.argv[1:3]
    d = read_bag(bag)
    ph = phases_of(bag)
    tt, tv, tw = truth_velocity(d)
    t1, v1, w1 = odom_series(d['/wheelchair/odom'])
    t10, v10, w10 = rolling_odometry(d, 10)
    omega_panels = [('Rotation start (0 -> +1 rad/s)', ph['left turn']),
                    ('Reverse (+1 -> -1 rad/s)', ph['right turn'])]

    # 1. 移動平均の変更（倍率なし）
    figure(os.path.join(out, 'odom_window_omega.png'),
           'Odometry angular velocity: moving average 10 (0.2 s) vs 1 (0.02 s), no scale',
           (tt, tw),
           [('Moving average 10 (0.2 s)', t10, w10, WINDOW10),
            ('Moving average 1 (0.02 s)', t1, w1, WINDOW1)],
           omega_panels, 'Angular velocity [rad/s]', (-0.3, 0.6))
    figure(os.path.join(out, 'odom_window_v.png'),
           'Odometry linear velocity: moving average 10 (0.2 s) vs 1 (0.02 s)',
           (tt, tv),
           [('Moving average 10 (0.2 s)', t10, v10, WINDOW10),
            ('Moving average 1 (0.02 s)', t1, v1, WINDOW1)],
           [('Forward start (rotation -> 1.5 m/s)', ph['forward']), ('Stop (1.5 -> 0 m/s)', ph['stop'])],
           'Linear velocity [m/s]', (-0.3, 0.8))

    # 2. 倍率の補正（移動平均 1）
    figure(os.path.join(out, 'odom_scale_omega.png'),
           f'Odometry angular velocity scale: no scale vs × {SCALE} (moving average 1)',
           (tt, tw),
           [('Moving average 1, no scale', t1, w1, WINDOW1),
            (f'Moving average 1, × {SCALE}', t1, w1 * SCALE, WINDOW1_SCALED)],
           omega_panels, 'Angular velocity [rad/s]', (-0.3, 0.6))


if __name__ == '__main__':
    main()
