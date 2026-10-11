#!/usr/bin/env python3
"""Gazebo のオドメトリの取り方（diff_drive の 0.2 s 移動平均 / それを 0.09 s 戻したもの / 車輪の角速度）を比べる図（localizer_rl 内で実行）.

  python3 plot_odom_sources.py <出力の接頭辞> <入力 bag> <results の jsonl>

出力:
  <接頭辞>_zoom.png     速度が急に変わる 4 つの瞬間の omega・v（真値、/wheelchair/odom、0.09 s 戻したもの、
                        joint_states から計算した /localization/wheel_odom）
  <接頭辞>_summary.png  同じ録画の再生での指標（NDT 単体、各推定器の「平均 + 0.09 s」と「車輪の角速度」。2 回の平均と幅）
"""
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import capitalize_labels, GRID, INK, INK2, read_bag, SURFACE, TRUTH_COLOR, TRUTH_LW  # noqa: E402
from plot_velocity import odom_series, phases_of, truth_velocity  # noqa: E402

TRANSITIONS = [('start', 'left turn'), ('reverse', 'right turn'), ('to forward', 'forward'), ('stop', 'stop')]
# 描く順（後方→手前）：真値・平均・0.09 s 戻したもの・車輪の角速度。後方ほど太く、点線は使わない
SOURCES = [('/wheelchair/odom (0.2 s moving average)', '#eb6834', 2.6),
           ('/wheelchair/odom, stamp - 0.09 s', '#4a3aa7', 1.9),
           ('from wheel joint velocity (/wheelchair/joint_states)', '#2a78d6', 1.2)]
# 棒グラフの条件（名前の接頭辞、ラベル、色）。推定器ごとに、薄い色 = 平均 + 0.09 s、濃い色 = 車輪の角速度
CONDITIONS = [('hv_ndt_only', 'NDT only', '#eda100'),
              ('hv_avg_ekf_', 'ekf_localizer: average + 0.09 s', '#86b6ef'),
              ('hv_ekf_', 'ekf_localizer: wheel joint velocity', '#2a78d6'),
              ('hv_avg_rl_', 'robot_localization: average + 0.09 s', '#8fd6b9'),
              ('hv_rl_', 'robot_localization: wheel joint velocity', '#1baf7a')]
METRICS = [('start_max', 'yaw error peak: rotation start [deg]'),
           ('reverse_max', 'yaw error peak: reverse [deg]'),
           ('to_fwd_max', 'yaw error peak: rotation -> forward [deg]'),
           ('stop_max', 'yaw error peak: stop [deg]'),
           ('steady_mean', 'yaw error while turning steadily [deg]'),
           ('ang_mean', 'NDT angle_diff, mean [deg]')]


def setup():
    plt.rcParams.update({
        'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
        'ytick.color': INK2, 'text.color': INK, 'axes.titlecolor': INK, 'axes.titlesize': 10,
        'axes.titleweight': 'bold', 'axes.titlelocation': 'left'})


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for spine in ('top', 'right'):
        ax.spines[spine].set_visible(False)


def fig_zoom(prefix, bag):
    d = read_bag(bag)
    ph = phases_of(bag)
    tt, tv, tw = truth_velocity(d)
    avg = odom_series(d['/wheelchair/odom'])
    joint = odom_series(d['/localization/wheel_odom'])
    data = [(avg[0], avg[1], avg[2]), (avg[0] - 0.09, avg[1], avg[2]), joint]
    fig, axes = plt.subplots(2, 4, figsize=(16, 6.6), facecolor=SURFACE)
    for r, (key, unit) in enumerate((('omega', 'rad/s'), ('v', 'm/s'))):
        for c, (tname, pkey) in enumerate(TRANSITIONS):
            ax = axes[r][c]
            style(ax)
            ts = ph[pkey]
            lo, hi = ts - 0.4, ts + 0.8
            m = (tt >= lo) & (tt <= hi)
            ax.plot(tt[m] - ts, (tv if key == 'v' else tw)[m], color=TRUTH_COLOR, lw=TRUTH_LW, zorder=2)
            for i, ((label, color, lw), (t, v, w)) in enumerate(zip(SOURCES, data)):
                n = (t >= lo) & (t <= hi)
                ax.plot(t[n] - ts, (v if key == 'v' else w)[n], color=color, lw=lw, zorder=3 + i)
            ax.axvline(0, color=INK2, lw=0.8, zorder=1)
            ax.set_xlim(-0.4, 0.8)
            ax.set_title(f'{key} [{unit}]  ({tname})', fontsize=9.5)
            if r == 1:
                ax.set_xlabel('time from command switch [s]')
    handles = [plt.Line2D([], [], color=TRUTH_COLOR, lw=TRUTH_LW, label='truth (d/dt of /wheelchair/pose)')]
    handles += [plt.Line2D([], [], color=c, lw=w, label=lab) for lab, c, w in SOURCES]
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.03, 0.95), ncol=4, frameon=False)
    fig.suptitle('Odometry sources around each velocity change (vertical line = command switch)',
                 x=0.03, ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(f'{prefix}_zoom.png', dpi=120, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {prefix}_zoom.png')


def fig_summary(prefix, results):
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.6), facecolor=SURFACE)
    for ax, (key, title) in zip(axes.flat, METRICS):
        style(ax)
        bars = []
        for i, (pre, label, color) in enumerate(CONDITIONS):
            # 接頭辞のあとが回数（_1 など）だけの試行（hv_ekf_accv_1 などの候補は除く）
            vals = [r[key] for name, r in results.items()
                    if name.startswith(pre) and name.replace(pre, '').rstrip('0123456789_') == '']
            mean = float(np.mean(vals))
            ax.bar(i, mean, 0.8, color=color, zorder=3)
            ax.plot([i, i], [min(vals), max(vals)], color=INK, lw=1.2, zorder=4)
            bars.append((i, max(vals), mean))
        top = max(v for _, v, _ in bars)
        ax.set_ylim(0, top * 1.2)
        for i, vmax, mean in bars:   # 値を棒の上に書く（色だけに頼らない）
            ax.text(i, vmax + top * 0.02, f'{mean:.2f}', ha='center', va='bottom', fontsize=8.5, color=INK)
        ax.set_xticks([])
        ax.set_title(title, fontsize=9.5)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c, label=lab) for _, lab, c in CONDITIONS]
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.03, 0.95), ncol=3, frameon=False)
    fig.suptitle('Same recording replayed (bar = mean of 2 runs, line = range; NDT only: 1 run)', x=0.03,
                 ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(f'{prefix}_summary.png', dpi=120, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {prefix}_summary.png')


def main():
    prefix, bag, results_path = sys.argv[1:4]
    results = {}
    for line in open(results_path):
        if line.strip():
            r = json.loads(line)
            results[r['name']] = r
    setup()
    fig_zoom(prefix, bag)
    fig_summary(prefix, results)


if __name__ == '__main__':
    main()
