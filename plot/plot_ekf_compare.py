#!/usr/bin/env python3
"""Gazebo のオドメトリの移動平均を変えたときの ekf_localizer の比較の図（localizer_rl 内で実行）.

  python3 plot_ekf_compare.py <出力の接頭辞> <results の jsonl>

同じ録画（spin360_w1_ekf）を再生した結果を比べる（tune_run.py の bag は /root/bags/tune/<試行名>）。
  w1cmp_ekf_w10_*  移動平均 10（変更前。make_window_bag.py で同じ走行から作った入力）
  w1cmp_ekf_js_*   車輪の角速度（/wheelchair/joint_states、joint_state_odometry）
  w1cmp_ekf_w1_*   移動平均 1（変更後。/wheelchair/odom そのもの）
  w1cmp_ndt_only   NDT 単体（参考）
どれも odom_scale [1, 0.922]、odom_delay 0、Q・R は同じ。

出力:
  <接頭辞>_timeseries.png   変更前・変更後の誤差の時間推移（plot_spin.py と同じ形）
  <接頭辞>_transitions.png  速度が変わる 4 つの瞬間の yaw 誤差と推定値の角速度
  <接頭辞>_summary.png      指標の棒グラフ（2 回の平均と幅。NDT 単体は 1 回）
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
import plot_spin  # noqa: E402
from plot_spin import capitalize_labels, GRID, INK, INK2, load_run, read_bag, SURFACE, TRUTH_COLOR, TRUTH_LW  # noqa: E402
from plot_velocity import odom_series, truth_velocity  # noqa: E402

TUNE = '/root/bags/tune'
BEFORE = ('EKF, moving average 10 (before)', 'w1cmp_ekf_w10_1', '#eb6834')
AFTER = ('EKF, moving average 1 (after)', 'w1cmp_ekf_w1_1', '#2a78d6')
# 棒グラフの条件（試行名の接頭辞、ラベル、色）
CONDITIONS = [('w1cmp_ekf_w10_', 'EKF, moving average 10 (before)', '#eb6834'),
              ('w1cmp_ekf_js_', 'EKF, wheel joint velocity (joint_states)', '#1baf7a'),
              ('w1cmp_ekf_w1_', 'EKF, moving average 1 (after)', '#2a78d6'),
              ('w1cmp_ndt_only', 'NDT only', '#eda100')]
METRICS = [('start_max', 'Yaw error peak: rotation start [deg]'),
           ('reverse_max', 'Yaw error peak: reverse [deg]'),
           ('to_fwd_max', 'Yaw error peak: rotation -> forward [deg]'),
           ('stop_max', 'Yaw error peak: stop [deg]'),
           ('steady_mean', 'Yaw error while turning steadily [deg]'),
           ('ang_mean', 'NDT angle_diff, mean [deg]')]
TRANSITIONS = [('Rotation start (0 -> +1 rad/s)', 'left turn'), ('Reverse (+1 -> -1 rad/s)', 'right turn'),
               ('Rotation -> forward', 'forward'), ('Stop (1.5 -> 0 m/s)', 'stop')]


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


def fig_timeseries(prefix):
    plot_spin.SERIES = [BEFORE[2], AFTER[2]]
    runs = [(label, load_run(os.path.join(TUNE, name))) for label, name, _ in (BEFORE, AFTER)]
    plot_spin.plot(f'{prefix}_timeseries.png', runs)
    plt.close('all')
    print(f'saved {prefix}_timeseries.png')


def fig_transitions(prefix):
    fig, axes = plt.subplots(2, 4, figsize=(16, 7), facecolor=SURFACE)
    widths = plot_spin.widths_back_to_front(2)
    for k, (label, name, color) in enumerate((BEFORE, AFTER)):
        path = os.path.join(TUNE, name)
        r = load_run(path)   # r['t'] と r['phases'] は左回転の開始（t0）からの時刻
        d = read_bag(path)
        et, _, ew = odom_series(d['/ekf_odom'])
        with open(path + '_phases.txt') as f:
            t0 = float(dict(line.rsplit(' ', 1) for line in f)['left turn'])
        if k == 0:   # 真値は同じ録画なので 1 回だけ、一番後ろに描く
            tt, _, tw = truth_velocity(d)
            for c, (_, key) in enumerate(TRANSITIONS):
                ts = r['phases'][key]
                m = (tt - t0 >= ts - 0.4) & (tt - t0 <= ts + 1.3)
                axes[1][c].plot(tt[m] - t0 - ts, tw[m], color=TRUTH_COLOR, lw=TRUTH_LW, zorder=2,
                                label='Truth (d/dt of /wheelchair/pose)')
        for c, (_, key) in enumerate(TRANSITIONS):
            ts = r['phases'][key]
            m = (r['t'] >= ts - 0.4) & (r['t'] <= ts + 1.3)
            axes[0][c].plot(r['t'][m] - ts, r['err'][m, 2], color=color, lw=widths[k], zorder=3 + k, label=label)
            n = (et - t0 >= ts - 0.4) & (et - t0 <= ts + 1.3)
            axes[1][c].plot(et[n] - t0 - ts, ew[n], color=color, lw=widths[k], zorder=3 + k, label=label)
    for c, (title, _) in enumerate(TRANSITIONS):
        for r_ in range(2):
            ax = axes[r_][c]
            style(ax)
            ax.axvline(0, color=INK2, lw=0.8, zorder=1)
            ax.set_xlim(-0.3, 1.2)
        axes[0][c].axhline(0, color=INK2, lw=0.8, zorder=1)
        axes[0][c].set_title(title, fontsize=9.5)
        axes[1][c].set_xlabel('Time from command switch [s]')
    axes[0][0].set_ylabel('Yaw error of /ekf_pose [deg]')
    axes[1][0].set_ylabel('Angular velocity [rad/s]')
    handles, labels = axes[1][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left', bbox_to_anchor=(0.03, 0.95), ncol=3, frameon=False)
    fig.suptitle('EKF (ekf_localizer) at each velocity change: yaw error (top) and estimated angular velocity (bottom)',
                 x=0.03, ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.91))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(f'{prefix}_transitions.png', dpi=120, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {prefix}_transitions.png')


def fig_summary(prefix, results):
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.6), facecolor=SURFACE)
    for ax, (key, title) in zip(axes.flat, METRICS):
        style(ax)
        bars = []
        for i, (pre, _, color) in enumerate(CONDITIONS):
            vals = [r[key] for name, r in results.items() if name.startswith(pre)]
            mean = float(np.mean(vals))
            ax.bar(i, mean, 0.8, color=color, zorder=3)
            if len(vals) > 1:
                ax.plot([i, i], [min(vals), max(vals)], color=INK, lw=1.2, zorder=4)
            bars.append((i, max(vals), mean))
        top = max(v for _, v, _ in bars)
        ax.set_ylim(0, top * 1.2)
        for i, vmax, mean in bars:   # 値を棒の上に書く（色だけに頼らない）
            ax.text(i, vmax + top * 0.02, f'{mean:.2f}', ha='center', va='bottom', fontsize=8.5, color=INK)
        ax.set_xticks([])
        ax.set_title(title, fontsize=9.5)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c, label=lab) for _, lab, c in CONDITIONS]
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.03, 0.95), ncol=4, frameon=False)
    fig.suptitle('Same recording replayed (bar = mean of 2 runs, line = range; NDT only: 1 run)', x=0.03,
                 ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(f'{prefix}_summary.png', dpi=120, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {prefix}_summary.png')


def main():
    prefix, results_path = sys.argv[1:3]
    results = {}
    for line in open(results_path):
        if line.strip():
            r = json.loads(line)
            results[r['name']] = r
    setup()
    fig_timeseries(prefix)
    setup()
    fig_transitions(prefix)
    fig_summary(prefix, results)


if __name__ == '__main__':
    main()
