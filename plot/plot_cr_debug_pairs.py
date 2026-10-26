#!/usr/bin/env python3
"""MATLAB（whill_common_matlab の CR_debug）が受け取った姿勢と組を描く（matlab/check_cr_debug.m の結果）.

  python3 plot_cr_debug_pairs.py <check_cr_debug.m の結果の .mat> <出力 png>

(1) 軌跡：MATLAB の周期ごとの Plant（最新の /tf map -> base、後輪間中心）と、新しい組の LIDAR_pose
    （点群の stamp の時刻の /tf と取付、LiDAR）。(2) 新しい組の間隔（LIDAR_new の周期の間）。
    題に、点群の stamp の時刻の TF がまだ無く前の組のままにした周期（LIDAR_waiting）の数。
2026-10-26 の 68 より前の結果（10 列）は /localization/base_pose・vlp16_pose を購読していたころのもの。
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.io import loadmat  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'calibration')]   # plot_spin などを import する
from plot_noise import box_on, setup, style  # noqa: E402
from plot_spin import capitalize_labels, SERIES, SURFACE, widths_back_to_front  # noqa: E402


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    rows = loadmat(sys.argv[1])['rows']
    out = sys.argv[2]
    paired = ~np.isnan(rows[:, 4])
    new = rows[:, 6] > 0
    mismatch = int(np.sum(np.abs(rows[paired, 4] - rows[paired, 5]) > 1e-9))
    waiting = int(np.sum(rows[:, 10] > 0)) if rows.shape[1] > 10 else 0
    gaps = np.diff(rows[new, 4]) * 1e3   # 新しい組の点群の stamp の間隔 [ms]

    setup()
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), facecolor=SURFACE, gridspec_kw={'width_ratios': [1.1, 1.0]})
    ax = axes[0]
    style(ax)
    w = widths_back_to_front(2)
    ax.plot(rows[:, 1], rows[:, 2], color=SERIES[0], lw=w[0],
            label=f'Plant (latest /tf, rear axle center): {len(rows)} cycles')
    ax.plot(rows[new, 7], rows[new, 8], color=SERIES[1], lw=w[1],
            label=f'LIDAR_pose (/tf at the cloud stamp): {int(new.sum())} new pairs')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Poses received by MATLAB (CR_debug)')
    ax.legend(loc='upper left', frameon=False, fontsize=8.5)
    box_on(ax)

    ax = axes[1]
    style(ax)
    bins = np.arange(0.0, max(gaps.max(), 250.0) + 10.0, 10.0)
    ax.hist(gaps, bins=bins, weights=np.full(len(gaps), 1.0 / len(gaps)), color=SERIES[1], edgecolor='black', lw=0.5)
    ax.set_xlabel('Interval between new pairs (cloud stamps) [ms]')
    ax.set_ylabel('Relative frequency')
    ax.set_title(f'Different stamps: {mismatch} of {int(paired.sum())} cycles, waiting for the TF: {waiting}')
    box_on(ax)
    capitalize_labels(fig)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f'saved {out}: cycles {len(rows)}, paired {int(paired.sum())}, new pairs {int(new.sum())}, '
          f'mismatch {mismatch}, waiting {waiting}, interval median {np.median(gaps):.0f} ms, max {gaps.max():.0f} ms')


if __name__ == '__main__':
    main()
