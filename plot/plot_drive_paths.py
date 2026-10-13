#!/usr/bin/env python3
"""テスト走行（drive/drive_test.py）の軌跡を、出発時の車体の位置・向きを基準に、走ってよい範囲と重ねて描く（localizer_rl 内で実行）.

  python3 plot_drive_paths.py <出力 png> <設定（gazebo・real・yaml のパス）> <動き>=<bag>[@<姿勢のトピック>] ...

姿勢のトピックの既定は Gazebo の真値 /wheelchair/pose（base_footprint）。/ekf_pose（LiDAR）なら設定の feedback の mount で
車体に直す。出発は最初の動作の時刻の姿勢。範囲は設定の area。点線は使わない（範囲は塗りつぶし）。
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise', 'drive')]
from drive_test import config_path, load_yaml, quat_matrix, rpy_matrix  # noqa: E402
from plot_spin import BAND, capitalize_labels, GRID, INK, INK2, read_bag, SERIES, SURFACE  # noqa: E402
from compare_run import phases_of  # noqa: E402


def base_path(bag, topic, mount):
    d = read_bag(bag, ['/clock', topic])
    clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d['/clock']]) if d.get('/clock') else None
    ts, xs, ys, yaws = [], [], [], []
    for t, m in d[topic]:
        p = m.pose.pose if hasattr(m, 'pose') and hasattr(m.pose, 'pose') else m
        tm = np.eye(4)
        tm[:3, :3] = quat_matrix(p.orientation)
        tm[:3, 3] = (p.position.x, p.position.y, p.position.z)
        if mount is not None:
            tm = tm @ mount
        ts.append(np.interp(t, clk[:, 0], clk[:, 1]) if clk is not None else t)
        xs.append(tm[0, 3])
        ys.append(tm[1, 3])
        yaws.append(math.atan2(tm[1, 0], tm[0, 0]))
    o = np.argsort(ts)
    return np.array(ts)[o], np.array(xs)[o], np.array(ys)[o], np.array(yaws)[o]


def main():
    out, target = sys.argv[1:3]
    cfg = load_yaml(config_path(target))
    area = cfg['area']
    items = [a.split('=', 1) for a in sys.argv[3:]]
    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
                         'ytick.color': INK2, 'axes.titlesize': 10.5})
    n = len(items)
    cols = 3
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 5.2 * rows), facecolor=SURFACE, squeeze=False)
    for ax, (name, spec) in zip(axes.flat, items):
        bag, topic = (spec.split('@', 1) + ['/wheelchair/pose'])[:2]
        mount = None
        if topic != '/wheelchair/pose' and cfg['feedback'].get('mount'):
            m = np.eye(4)
            m[:3, :3] = rpy_matrix(*cfg['feedback']['mount']['rpy'])
            m[:3, 3] = cfg['feedback']['mount']['xyz']
            mount = np.linalg.inv(m)
        t, x, y, yaw = base_path(bag, topic, mount)
        ph = phases_of(bag)
        t0, t1 = min(ph.values()), max(ph.values()) + 2.0
        k = int(np.searchsorted(t, t0))
        c, s = math.cos(yaw[k]), math.sin(yaw[k])
        m = (t >= t0) & (t <= t1)
        fx = c * (x[m] - x[k]) + s * (y[m] - y[k])
        fy = -s * (x[m] - x[k]) + c * (y[m] - y[k])
        ax.set_facecolor(SURFACE)
        # 範囲（縦軸 = 前、横軸 = 右。上から見た向きにする）
        ax.fill_between([-area['left'], area['right']], -area['behind'], area['forward'], color=BAND, lw=0, zorder=0,
                        label='Allowed area')
        ax.plot(-fy, fx, color=SERIES[0], lw=1.8, zorder=3, label='Path (vehicle center)')
        ax.plot(0, 0, 'o', color=INK, ms=6, zorder=4)
        ax.annotate('Start', (0, 0), xytext=(6, -12), textcoords='offset points', fontsize=8, color=INK2)
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
    fig.suptitle(f"Test drive paths in the start frame ({cfg['target']}; area forward {area['forward']} m, "
                 f"behind {area['behind']} m, sides {area['left']} / {area['right']} m)",
                 x=0.01, ha='left', fontsize=12, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    capitalize_labels(fig)
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f'saved {out}')


if __name__ == '__main__':
    main()
