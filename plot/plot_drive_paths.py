#!/usr/bin/env python3
"""テスト走行（drive/drive_test.py）の軌跡を、出発時の車体の位置・向きを基準に描く（コンテナ test 内で実行）.

  python3 plot_drive_paths.py <出力 png> <設定（gazebo・red・blue・white・yaml のパス）> <動き>=<bag> ...

車体の姿勢は録画の /tf の map -> base（自己位置推定の出力。フレームは localization_bringup の param/<target>/ekf_localizer.yaml の
map_frame_id・base_frame_id。同じ stamp が何度も届くので、stamp ごとに最後に届いた値）。時刻は stamp。出発は最初の動作の時刻の姿勢。設定に area（走ってよい範囲。2026-11-07 に drive の設定から外した）があれば
塗りつぶして重ねる。点線は使わない。
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'plot', 'eval', 'calibration', 'drive')]
from drive_test import load_config, yaw_of  # noqa: E402
from bag_io import phases_of, read_bag, stamp_sec  # noqa: E402
from env_files import node_params  # noqa: E402
from plot_style import BAND, capitalize_labels, GRID, INK, INK2, SERIES, SURFACE  # noqa: E402


def base_path(bag, frames):
    """録画の /tf の frames[0] -> frames[1]（map -> base）を stamp の順に（時刻 [s], x, y, yaw）."""
    poses = {}
    for _, m in read_bag(bag, ['/tf'])['/tf']:
        for tr in m.transforms:
            if tr.header.frame_id == frames[0] and tr.child_frame_id == frames[1]:
                t = tr.transform
                poses[stamp_sec(tr)] = (t.translation.x, t.translation.y, yaw_of(t.rotation))
    if not poses:
        raise RuntimeError(f'no /tf {frames[0]} -> {frames[1]} in {bag}')
    ts = np.array(sorted(poses))
    xs, ys, yaws = (np.array(v) for v in zip(*(poses[t] for t in ts)))
    return ts, xs, ys, yaws


def main():
    out, target = sys.argv[1:3]
    cfg = load_config(target)
    area = cfg.get('area')
    items = [a.split('=', 1) for a in sys.argv[3:]]
    ekf = node_params(cfg['target'], 'ekf_localizer')
    frames = (ekf.get('map_frame_id', 'map'), ekf.get('base_frame_id', 'base_link'))
    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
                         'ytick.color': INK2, 'axes.titlesize': 10.5})
    n = len(items)
    cols = 3
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 5.2 * rows), facecolor=SURFACE, squeeze=False)
    for ax, (name, bag) in zip(axes.flat, items):
        t, x, y, yaw = base_path(bag, frames)
        ph = phases_of(bag)
        t0, t1 = min(ph.values()), max(ph.values()) + 2.0
        k = int(np.searchsorted(t, t0))
        c, s = math.cos(yaw[k]), math.sin(yaw[k])
        m = (t >= t0) & (t <= t1)
        fx = c * (x[m] - x[k]) + s * (y[m] - y[k])
        fy = -s * (x[m] - x[k]) + c * (y[m] - y[k])
        ax.set_facecolor(SURFACE)
        # 範囲（縦軸 = 前、横軸 = 右。上から見た向きにする）
        if area:
            ax.fill_between([-area['left'], area['right']], -area['behind'], area['forward'], color=BAND, lw=0,
                            zorder=0, label='Allowed area')
        ax.plot(-fy, fx, color=SERIES[0], lw=1.8, zorder=3, label='Path (vehicle center)')
        ax.plot(0, 0, 'o', color=INK, ms=6, zorder=4)
        ax.annotate('Start', (0, 0), xytext=(6, -12), textcoords='offset points', fontsize=8, color=INK2)
        if area:
            ax.set_xlim(-area['left'] - 0.5, area['right'] + 0.5)
            ax.set_ylim(-area['behind'] - 0.5, area['forward'] + 0.5)
        ax.set_aspect('equal')
        ax.grid(color=GRID, lw=0.6)
        ax.set_title(f'{name}\nForward {fx.min():.2f} to {fx.max():.2f} m, right {-fy.max():+.2f} to {-fy.min():+.2f} m',
                     loc='left', fontsize=9)
        ax.set_xlabel('Right [m]')
        ax.set_ylabel('Forward [m]')
    for ax in list(axes.flat)[n:]:
        ax.set_visible(False)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left', ncol=2, frameon=False, bbox_to_anchor=(0.01, 0.955))
    title = f"Test drive paths in the start frame ({cfg['target']}"
    if area:
        title += (f"; area forward {area['forward']} m, behind {area['behind']} m, "
                  f"sides {area['left']} / {area['right']} m")
    fig.suptitle(title + ')',
                 x=0.01, ha='left', fontsize=12, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    capitalize_labels(fig)
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f'saved {out}')


if __name__ == '__main__':
    main()
