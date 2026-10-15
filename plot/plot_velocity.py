#!/usr/bin/env python3
"""オドメトリと各推定器の v・omega の推移を、真値と比べて描く（コンテナ test 内で実行）.

  python3 plot_velocity.py <出力の接頭辞> <入力 bag> <odom_delay [s]> \\
      <推定器名>:<調整前の bag>:<調整後の bag>:<トピック> ...
  例: ... ekf_localizer:/root/ros2_ws/bags/gazebo/tune/v_ekf_before_1:/root/ros2_ws/bags/gazebo/tune/final_ekf:/ekf_odom

出力: <接頭辞>_full.png（全体）、<接頭辞>_zoom.png（速度が急に変わる 4 つの瞬間の拡大）。
真値は入力 bag の /wheelchair/pose（base_footprint）を ±0.03 s の差分で微分したもの（v は車体の前方向成分）。
オドメトリは入力 bag の /wheelchair/odom。点線は odom_delay だけ stamp を戻したもの（odom_covariance と同じ補正）。
推定値は各 bag の nav_msgs/Odometry（ekf_localizer は /ekf_odom、robot_localization は /odometry/global）の
twist.linear.x・twist.angular.z を stamp の時刻で描く。
変化の半分に達した時刻の、真値からの遅れ [ms] も表にする。
環境変数：ODOM_SCALE="v,omega"（オドメトリに掛ける倍率。既定 1,1）、LABEL_BEFORE・LABEL_AFTER（推定値の凡例の名前）。
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import capitalize_labels, BAND, GRID, INK, INK2, read_bag, SURFACE, TRUTH_COLOR, TRUTH_LW  # noqa: E402
from pose_math import matrix_to_vec, pose_msg_to_matrix  # noqa: E402

# dataviz 参照パレット（調整後 slot 1・オドメトリ slot 2・調整前 slot 3・遅れを補正したオドメトリ slot 7）と真値
C_AFTER, C_ODOM, C_BEFORE, C_ODOM_COMP, C_TRUTH = '#2a78d6', '#eb6834', '#1baf7a', '#4a3aa7', TRUTH_COLOR
# 描く順（後方→手前）の線幅：真値・オドメトリ・補正したオドメトリ・調整前・調整後（後方ほど太く、点線は使わない）
LW = [TRUTH_LW, 2.8, 2.2, 1.7, 1.2]
TRANSITIONS = [('start', 'left turn', 'omega'), ('reverse', 'right turn', 'omega'),
               ('to forward', 'forward', 'v'), ('stop', 'stop', 'v')]


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def phases_of(bag):
    out = {}
    with open(bag.rstrip('/') + '_phases.txt') as f:
        for line in f:
            name, t = line.rsplit(' ', 1)
            out[name] = float(t)
    return out


def truth_velocity(d):
    clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d['/clock']])
    tt = np.interp([t for t, _ in d['/wheelchair/pose']], clk[:, 0], clk[:, 1])
    vec = np.array([matrix_to_vec(pose_msg_to_matrix(m)) for _, m in d['/wheelchair/pose']])
    o = np.argsort(tt)
    tt, vec = tt[o], vec[o]
    vec[:, 5] = np.unwrap(vec[:, 5])
    grid = np.arange(tt[0], tt[-1], 0.01)
    x, y, yaw = (np.interp(grid, tt, vec[:, i]) for i in (0, 1, 5))
    k = 3   # ±0.03 s
    t = grid[k:-k]
    dx, dy, dyaw = ((a[2 * k:] - a[:-2 * k]) / (grid[2 * k:] - grid[:-2 * k]) for a in (x, y, yaw))
    v = dx * np.cos(yaw[k:-k]) + dy * np.sin(yaw[k:-k])
    return t, v, dyaw


def odom_series(msgs):
    t = np.array([stamp(m) for _, m in msgs])
    v = np.array([m.twist.twist.linear.x for _, m in msgs])
    w = np.array([m.twist.twist.angular.z for _, m in msgs])
    o = np.argsort(t)
    return t[o], v[o], w[o]


def half_time(t, y, t_switch):
    """t_switch の前後の値の中間を、t_switch - 0.2 s 以降で最初に横切る時刻."""
    before = (t >= t_switch - 0.6) & (t <= t_switch - 0.1)
    after = (t >= t_switch + 1.0) & (t <= t_switch + 1.6)
    if not before.any() or not after.any():
        return float('nan')
    a, b = y[before].mean(), y[after].mean()
    if abs(b - a) < 1e-3:
        return float('nan')
    mid = (a + b) / 2.0
    s = np.sign(b - a)
    idx = np.where((t >= t_switch - 0.2) & (s * (y - mid) >= 0))[0]
    if not len(idx):
        return float('nan')
    i = idx[0]
    if i == 0:
        return float(t[i])
    t0, t1, y0, y1 = t[i - 1], t[i], y[i - 1], y[i]
    return float(t0 + (mid - y0) * (t1 - t0) / (y1 - y0)) if y1 != y0 else float(t1)


def style(ax, bands):
    ax.set_facecolor(SURFACE)
    for i, (_, a, b) in enumerate(bands):
        ax.axvspan(a, b, color=BAND if i % 2 == 0 else '#f6f5f2', lw=0, zorder=0)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for spine in ('top', 'right'):
        ax.spines[spine].set_visible(False)


def main():
    prefix, in_bag, delay = sys.argv[1], sys.argv[2], float(sys.argv[3])
    estimators = []
    for spec in sys.argv[4:]:
        name, before, after, topic = spec.split(':')
        estimators.append((name, odom_series(read_bag(before)[topic]),
                           odom_series(read_bag(after)[topic])))
    d = read_bag(in_bag)
    ph = phases_of(in_bag)
    t0 = ph['left turn']
    tt, tv, tw = truth_velocity(d)
    ot, ov, ow = odom_series(d['/wheelchair/odom'])
    sv, sw = (float(x) for x in os.environ.get('ODOM_SCALE', '1,1').split(','))
    ov, ow = ov * sv, ow * sw
    scale_note = '' if (sv, sw) == (1.0, 1.0) else f' x (v {sv:g}, omega {sw:g})'
    rel = {k: v - t0 for k, v in ph.items()}
    bands = [('left turn', rel['left turn'], rel['right turn']),
             ('right turn', rel['right turn'], rel['forward']), ('forward', rel['forward'], rel['stop'])]

    plt.rcParams.update({
        'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
        'ytick.color': INK2, 'text.color': INK, 'axes.titlecolor': INK, 'axes.titlesize': 10,
        'axes.titleweight': 'bold', 'axes.titlelocation': 'left'})
    handles = [
        plt.Line2D([], [], color=C_TRUTH, lw=LW[0], label='truth (d/dt of /wheelchair/pose)'),
        plt.Line2D([], [], color=C_ODOM, lw=LW[1], label=f'odometry (/wheelchair/odom){scale_note}'),
        plt.Line2D([], [], color=C_ODOM_COMP, lw=LW[2], label=f'odometry{scale_note}, stamp - {delay:.2f} s'),
        plt.Line2D([], [], color=C_BEFORE, lw=LW[3],
                   label=os.environ.get('LABEL_BEFORE', 'estimate (before tuning)')),
        plt.Line2D([], [], color=C_AFTER, lw=LW[4], label=os.environ.get('LABEL_AFTER', 'estimate (tuned)'))]

    def draw(ax, key, est, lo, hi):
        before, after = est
        idx = 1 if key == 'v' else 2
        for t, y, kw in ((tt, tv if key == 'v' else tw, dict(color=C_TRUTH, lw=LW[0], zorder=2)),
                         (ot, ov if key == 'v' else ow, dict(color=C_ODOM, lw=LW[1], zorder=3)),
                         (ot - delay, ov if key == 'v' else ow, dict(color=C_ODOM_COMP, lw=LW[2], zorder=4)),
                         (before[0], before[idx], dict(color=C_BEFORE, lw=LW[3], zorder=5)),
                         (after[0], after[idx], dict(color=C_AFTER, lw=LW[4], zorder=6))):
            m = (t - t0 >= lo) & (t - t0 <= hi)
            ax.plot(t[m] - t0, y[m], **kw)

    # ---- 全体 ----
    lo, hi = -3.0, rel['stop'] + 4.0
    fig, axes = plt.subplots(2, len(estimators), figsize=(7 * len(estimators), 8), sharex=True,
                             facecolor=SURFACE, squeeze=False)
    for col, (name, before, after) in enumerate(estimators):
        for row, (key, unit) in enumerate((('v', 'm/s'), ('omega', 'rad/s'))):
            ax = axes[row][col]
            style(ax, bands)
            draw(ax, key, (before, after), lo, hi)
            ax.set_title(f'{name}: {key} [{unit}]')
            if row == 0:
                for label, a, b in bands:
                    # v は回転中ほぼ 0 なので、区間名はグラフの中段に書く（タイトルと重ならない）
                    ax.text((a + b) / 2, 0.55, label, transform=ax.get_xaxis_transform(),
                            ha='center', va='center', fontsize=9, color=INK2)
        axes[-1][col].set_xlabel('time since start of left turn [s] (sim time)')
        axes[-1][col].set_xlim(lo, hi)
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.04, 0.975), ncol=3, frameon=False)
    fig.suptitle('Odometry and estimated v / omega vs truth', x=0.04, ha='left', fontsize=12,
                 fontweight='bold', color=INK, y=0.998)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(f'{prefix}_full.png', dpi=130, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {prefix}_full.png')

    # ---- 速度が急に変わる 4 つの瞬間の拡大（行：推定器 × v/omega）----
    rows = [(name, key) for name, _, _ in estimators for key in ('omega', 'v')]
    fig, axes = plt.subplots(len(rows), len(TRANSITIONS), figsize=(16, 3.0 * len(rows)),
                             facecolor=SURFACE, squeeze=False)
    est_map = {name: (b, a) for name, b, a in estimators}
    for r, (name, key) in enumerate(rows):
        for c, (tname, pkey, _) in enumerate(TRANSITIONS):
            ax = axes[r][c]
            style(ax, [])
            ts = rel[pkey]
            draw(ax, key, est_map[name], ts - 0.5, ts + 1.5)
            ax.axvline(ts, color=INK2, lw=0.8, ls='-', zorder=1)
            ax.set_xlim(ts - 0.5, ts + 1.5)
            ax.set_title(f'{name}: {key}  ({tname})', fontsize=9)
            if r == len(rows) - 1:
                ax.set_xlabel('time [s]')
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.03, 0.985), ncol=3, frameon=False)
    fig.suptitle('Around each velocity change (vertical line = command switch)', x=0.03, ha='left',
                 fontsize=12, fontweight='bold', color=INK, y=0.999)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(f'{prefix}_zoom.png', dpi=110, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {prefix}_zoom.png')

    # ---- 遅れ：変化の半分に達した時刻の、真値からの差 [ms] ----
    print(f"{'delay vs truth [ms]':<34}" + ''.join(f'{t:>14}' for t, _, _ in TRANSITIONS))
    series = [('odometry', ot, ov, ow), (f'odometry stamp-{delay:.2f}s', ot - delay, ov, ow)]
    for name, before, after in estimators:
        series += [(f'{name} before', before[0], before[1], before[2]),
                   (f'{name} tuned', after[0], after[1], after[2])]
    for label, t, v, w in series:
        cells = []
        for _, pkey, key in TRANSITIONS:
            ts = ph[pkey]
            ref = half_time(tt, tv if key == 'v' else tw, ts)
            got = half_time(t, v if key == 'v' else w, ts)
            cells.append((got - ref) * 1000.0)
        print(f'{label:<34}' + ''.join(f'{c:14.0f}' for c in cells))
    print('(start/reverse: omega, to forward/stop: v. positive = later than truth)')


if __name__ == '__main__':
    main()
