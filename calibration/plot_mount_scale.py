#!/usr/bin/env python3
"""calibrate_mount_scale.py の図（calibrate_mount_scale.py から呼ぶ）.

  左：その場の旋回の LiDAR の位置（回転の中心からの位置）と、推定した取付・今の取付の円
  中・右：区間ごとの omega・v の倍率（推定値と今の設定の線。--truth なら真値からの倍率の点）
plot_mount_yaw：直進の区間の各時刻の取付の yaw（RANSAC のインライア・外れ値）と、最終的な取付の yaw（<出力>_yaw.png）
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'calibration')]   # plot_spin などを import する
from plot_noise import box_on  # noqa: E402
from plot_spin import (capitalize_labels, GRID, INK, INK2, SURFACE, TRUTH_COLOR,  # noqa: E402
                       TRUTH_LW)

C_EST, C_SEG, C_TRUTH, C_OUT = '#2a78d6', '#1baf7a', '#eb6834', '#eda100'


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    box_on(ax)   # 枠は 4 辺とも黒（box on）


def setup():
    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
                         'ytick.color': INK2, 'text.color': INK, 'axes.titlecolor': INK, 'axes.titlesize': 10,
                         'axes.titleweight': 'bold', 'axes.titlelocation': 'left'})


def plot_mount_scale(path, turns, sol, out, now):
    setup()
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.6), facecolor=SURFACE, gridspec_kw={'width_ratios': [1.2, 1, 1]})
    # 左：回転の中心からの LiDAR の位置
    ax = axes[0]
    style(ax)
    for j, (t, p, _) in enumerate(turns):
        c = sol[2 + 4 * j:4 + 4 * j]
        c1 = sol[4 + 4 * j:6 + 4 * j]
        rel = p - c - np.outer(t - t.mean(), c1)
        ax.scatter(rel[:, 0] * 1e3, rel[:, 1] * 1e3, s=6, color=C_SEG, alpha=0.5, lw=0, zorder=2,
                   label='LiDAR position from the rotation center (NDT)' if j == 0 else None)
    th = np.linspace(0, 2 * math.pi, 200)
    r_now = math.hypot(now[0], now[1])
    r_est = math.hypot(*out['mount_xy'])
    ax.plot(r_now * 1e3 * np.cos(th), r_now * 1e3 * np.sin(th), color=TRUTH_COLOR, lw=TRUTH_LW, zorder=3,
            label=f'Current mount: radius {r_now * 1e3:.0f} mm')
    ax.plot(r_est * 1e3 * np.cos(th), r_est * 1e3 * np.sin(th), color=C_EST, lw=1.4, zorder=4,
            label=f'Estimated: radius {r_est * 1e3:.0f} mm')
    ax.set_aspect('equal')
    ax.set_xlabel('X [mm] (map)')
    ax.set_ylabel('Y [mm] (map)')
    ax.set_title(f"LiDAR mount: x {out['mount_xy'][0] * 1e3:+.0f} mm, y {out['mount_xy'][1] * 1e3:+.0f} mm "
                 f"(current {now[0] * 1e3:+.0f}, {now[1] * 1e3:+.0f})")
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13), frameon=False, fontsize=8.5)
    # 中・右：倍率（1 付近の比なので棒ではなく点で示す）
    for ax, key, rows, name, cur in ((axes[1], 'omega_scale', out['turning'], 'Angular velocity', out['current_scale'][1]),
                                     (axes[2], 'v_scale', out['straight'], 'Linear velocity', out['current_scale'][0])):
        style(ax)
        ax.grid(axis='x', visible=False)
        if not rows:
            ax.set_title(f'{name} scale: no segment')
            continue
        x = np.arange(1, len(rows) + 1)
        ax.axhline(cur, color=TRUTH_COLOR, lw=TRUTH_LW, zorder=2, label=f'Current setting {cur:.3f}')
        ax.axhline(out[key], color=C_EST, lw=1.4, zorder=3, label=f'Estimated {out[key]:.4f}')
        ax.scatter(x, [r['ratio'] for r in rows], s=46, color=C_SEG, zorder=4, label='Each segment (NDT / odometry)')
        if 'truth_ratio' in rows[0]:
            ax.scatter(x, [r['truth_ratio'] for r in rows], s=18, color=C_TRUTH, zorder=5,
                       label='Truth / odometry (Gazebo only)')
        vals = [r['ratio'] for r in rows] + [cur, out[key]]
        lo, hi = min(vals), max(vals)
        ax.set_ylim(lo - (hi - lo) * 0.6 - 0.005, hi + (hi - lo) * 0.9 + 0.005)
        ax.set_xticks(x)
        groups = []   # 区間の番号と録画の対応
        for i, r in enumerate(rows, 1):
            if groups and groups[-1][0] == r['bag']:
                groups[-1][2] = i
            else:
                groups.append([r['bag'], i, i])
        ax.set_xlabel('Segment  (' + ', '.join(f'{b}: {a}' + (f'-{c}' if c > a else '') for b, a, c in groups) + ')',
                      fontsize=8.5)
        ax.set_title(f'{name} scale (odom_scale)')
        ax.legend(loc='upper left', frameon=False, fontsize=8.5)
    fig.suptitle('LiDAR mount and odometry scale estimated from NDT and odometry (no truth)', x=0.02, ha='left',
                 fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(path, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {path}')


def plot_mount_yaw(path, yaw_series, out, now):
    """直進の区間の各時刻の取付の yaw（RANSAC のインライア・外れ値）と、最終的な取付の yaw.

    yaw_series：[(録画, 区間の始めからの時刻, 各時刻の yaw, インライアの印, 区間の yaw), ...]（calibrate_mount_scale.py）。区間は横に並べる。
    """
    setup()
    fig, ax = plt.subplots(figsize=(11, 5.4), facecolor=SURFACE)
    style(ax)
    final = out['mount_yaw']
    ax.axhline(math.degrees(now[5]), color=TRUTH_COLOR, lw=TRUTH_LW, zorder=2,
               label=f'Current mount yaw (TF) {math.degrees(now[5]):+.3f} deg')
    t_shift = 0.0
    for j, (bag, t, d, inl, yaw_seg) in enumerate(yaw_series):
        x = t + t_shift
        ax.scatter(x[inl], np.degrees(d[inl]), s=10, color=C_SEG, alpha=0.7, lw=0, zorder=3,
                   label='Each time, RANSAC inlier' if j == 0 else None)
        ax.scatter(x[~inl], np.degrees(d[~inl]), s=18, color=C_OUT, lw=0, zorder=4,
                   label='Each time, outlier (removed)' if j == 0 and (~inl).any() else None)
        t_shift = x.max() + 1.0
    ax.axhline(math.degrees(final), color=C_EST, lw=1.6, zorder=5,
               label=f'Final mount yaw {math.degrees(final):+.3f} deg (mean of inliers)')
    ax.set_xlabel('Time from the start of the straight segment [s]')
    ax.set_ylabel('Mount yaw (LiDAR yaw - heading of the base) [deg]')
    ax.set_title('LiDAR mount yaw on the straight segment: each time and the final value')
    ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1.0), frameon=False, fontsize=8.5)
    fig.tight_layout()
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(path, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {path}')
