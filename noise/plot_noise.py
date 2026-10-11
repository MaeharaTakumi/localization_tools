#!/usr/bin/env python3
"""estimate_noise.py の図（estimate_noise.py から呼ぶ）.

  plot_segments：使った区間（指令が一定）と、オドメトリ・NDT の残差の時間推移
  plot_summary ：標準偏差の比較（真値なしの推定：動いている区間・停止、--truth なら真値との差）
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
from plot_spin import (BAND, capitalize_labels, GRID, INK, INK2, SURFACE,  # noqa: E402
                       widths_back_to_front)

CLASS_LABEL = {'stopped': 'Stopped', 'turning': 'Turning', 'straight': 'Straight', 'curve': 'Curve'}
C_OMEGA, C_V, C_YAW, C_X, C_Y = '#2a78d6', '#eb6834', '#4a3aa7', '#1baf7a', '#eda100'
C_EST, C_STOP, C_TRUTH = '#2a78d6', '#1baf7a', '#eb6834'


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


def plot_segments(path, ot, ov, ow, nt, used, odom_rows, ndt_rows, segs, half):
    setup()
    moving = [s for s in segs if abs(s[2]) > 1e-3 or abs(s[3]) > 1e-3]
    t0 = moving[0][0] if moving else ot[0]
    fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True, facecolor=SURFACE)
    panels = [
        ('Odometry angular velocity (scaled) [rad/s]', [('Omega', ot, ow, C_OMEGA)], False),
        ('Odometry linear velocity [m/s]', [('V', ot, ov, C_V)], False),
        ('Odometry residual (after removing the local linear trend)  angular velocity [rad/s]',
         [('Omega residual', None, 'omega', C_OMEGA)], 'odom'),
        ('NDT residual  yaw [deg]', [('Yaw residual', None, 'yaw', C_YAW)], 'ndt'),
        ('NDT residual  x, y (map frame) [mm]', [('X residual', None, 'x', C_X), ('Y residual', None, 'y', C_Y)], 'ndt'),
    ]
    for ax, (title, series, kind) in zip(axes, panels):
        style(ax)
        for a, b, cls in used:   # 使った区間（指令が一定）
            ax.axvspan(a - t0, b - t0, color=BAND, lw=0, zorder=0)
        widths = widths_back_to_front(len(series))
        for i, (label, t, y, color) in enumerate(series):
            if kind is False:
                ax.plot(t - t0, y, color=color, lw=1.4, zorder=3, label=label)
                continue
            rows = odom_rows if kind == 'odom' else ndt_rows
            scale = 1.0 if kind == 'odom' else (math.degrees(1.0) if y in ('yaw',) else 1e3)
            for k, (_, r) in enumerate(rows):
                ax.plot(r['_t'] - t0, r[y] * scale, color=color, lw=widths[i], zorder=3 + i,
                        label=label if k == 0 else None)
        if kind:
            ax.axhline(0, color=INK2, lw=0.8, zorder=2)
        ax.set_title(title)
        if len(series) > 1:
            ax.legend(loc='upper right', frameon=False, ncol=len(series))
    for a, b, cls in used:   # 区間の種類を一番上に書く
        axes[0].text((a + b) / 2 - t0, 1.02, CLASS_LABEL[cls], transform=axes[0].get_xaxis_transform(),
                     ha='center', va='bottom', fontsize=8.5, color=INK2)
    axes[-1].set_xlabel('Time since the first motion command [s]')
    fig.suptitle(f'Segments with a constant command (shaded) and residuals used for R (local linear window '
                 f'{2 * half:.1f} s)', x=0.02, ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(path, dpi=110, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {path}')


def plot_summary(path, est_odom, est_ndt, tru_odom, tru_ndt):
    setup()
    groups = [('Odometry linear velocity [m/s]', est_odom, tru_odom, [('v', 'V')], 1.0),
              ('Odometry angular velocity [rad/s]', est_odom, tru_odom, [('omega', 'Omega')], 1.0),
              ('NDT position (map frame) [mm]', est_ndt, tru_ndt, [('x', 'X'), ('y', 'Y'), ('z', 'Z')], 1e3),
              ('NDT angle [deg]', est_ndt, tru_ndt, [('roll', 'Roll'), ('pitch', 'Pitch'), ('yaw', 'Yaw')],
               math.degrees(1.0))]
    bars = [('No truth: moving (used as R)', C_EST, lambda e, t, k: e['moving'][k][0]),
            ('No truth: stopped', C_STOP, lambda e, t, k: e['stopped'][k][0])]
    if tru_odom is not None:
        bars.append(('Truth check: moving (Gazebo only)', C_TRUTH, lambda e, t, k: t['moving'][k][0]))
    fig, axes = plt.subplots(1, 4, figsize=(15, 4.6), facecolor=SURFACE,
                             gridspec_kw={'width_ratios': [1, 1, 2.2, 2.2]})
    width = 0.8 / len(bars)
    for ax, (title, est, tru, keys, f) in zip(axes, groups):
        style(ax)
        top = 0.0
        for j, (label, color, get) in enumerate(bars):
            for i, (k, _) in enumerate(keys):
                var = get(est, tru, k)
                s = math.sqrt(var) * f if np.isfinite(var) else 0.0
                x = i + (j - (len(bars) - 1) / 2) * width
                ax.bar(x, s, width * 0.92, color=color, zorder=3, label=label if i == 0 else None)
                ax.text(x, s, f'{s:.3g}', ha='center', va='bottom', fontsize=7.5, color=INK)
                top = max(top, s)
        ax.set_ylim(0, top * 1.18 if top > 0 else 1)
        ax.set_xticks(range(len(keys)))
        ax.set_xticklabels([lab for _, lab in keys])
        ax.set_title(title, fontsize=9.5)
    axes[0].set_ylabel('Standard deviation')
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left', bbox_to_anchor=(0.02, 0.93), ncol=3, frameon=False)
    fig.suptitle('Observation noise (standard deviation) estimated without the truth', x=0.02, ha='left',
                 fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.84))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(path, dpi=120, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {path}')
