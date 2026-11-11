#!/usr/bin/env python3
"""NDT と lidarslam（SLAM）の再生の誤差（Gazebo の真値との差）を並べて描く（コンテナ test 内で実行）.

  python3 plot_slam_compare.py <出力 png> <ラベル>=<出力 bag>[:<姿勢のトピック>] ...

姿勢のトピックの既定は /ekf_pose。上段が xy の誤差、下段が yaw の誤差。帯は動作の区間（_phases.txt）。
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'plot')]
import plot_style as ps  # noqa: E402
from plot_spin import load_run  # noqa: E402


def main():
    out, specs = sys.argv[1], sys.argv[2:]
    runs = []
    for spec in specs:
        label, _, rest = spec.partition('=')
        bag, _, topic = rest.partition(':')
        runs.append((label, load_run(bag, topic or '/ekf_pose')))
    ps.setup()
    fig, (ax_xy, ax_yaw) = plt.subplots(2, 1, figsize=(8, 5.2), sharex=True)
    ph = runs[0][1]['phases']
    names = sorted(ph, key=ph.get)
    ends = [ph[n] for n in names[1:]] + [ph[names[-1]] + 4.0]
    bands = [(n, ph[n], e) for n, e in zip(names, ends)]
    widths = ps.widths_back_to_front(len(runs))
    for i, (label, r) in enumerate(runs):
        kw = dict(color=ps.SERIES[i], lw=widths[i], label=label, zorder=2 + i)
        ax_xy.plot(r['t'], r['err'][:, 0] * 1e3, **kw)
        ax_yaw.plot(r['t'], r['err'][:, 2], **kw)
    for ax in (ax_xy, ax_yaw):
        ps.style(ax, bands=bands)
    for name, a, _ in bands:
        ax_xy.text(a + 0.2, 0.97, name, transform=ax_xy.get_xaxis_transform(), va='top', fontsize=8, color=ps.INK2)
    ax_xy.set_ylabel('xy error [mm]')
    ax_yaw.set_ylabel('yaw error [deg]')
    ax_yaw.set_xlabel('Time from the left turn [s]')
    ax_xy.set_title('Error from the Gazebo truth (LiDAR pose)')
    ax_xy.legend(loc='upper left', bbox_to_anchor=(0.0, 0.88), frameon=False)
    ax_yaw.axhline(0.0, color=ps.INK2, lw=0.8, zorder=1)
    fig.tight_layout()
    ps.capitalize_labels(fig)
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == '__main__':
    main()
