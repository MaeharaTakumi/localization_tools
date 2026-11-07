#!/usr/bin/env python3
"""outputs_batch.py の結果から、ekf_localizer の速度の Q（v_z と omega_roll・omega_pitch）ごとの指標を描く.

  python3 plot_q_sweep.py <出力 png> <結果の jsonl> [--metric prior_ndt|prior_truth]

候補は params.ekf_localizer.Q の 7 番（v_z）と 8・9 番（omega_roll・omega_pitch。同じ値）で分ける：
v_z だけ正なら左の列、omega だけ正なら右の列、どちらも 0 なら基準（灰色の横線。録画ごと）。両方とも正の候補は描かない。
行は指標（|z| [mm]・|roll|・|pitch|・|yaw| [deg]・水平 [mm] の平均）、点は録画ごとの 1 回ずつの値、線は録画ごとの平均。
標準出力に候補 x 録画の表も出す。
"""
import argparse
import json
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'calibration')]
from plot_noise import box_on  # noqa: E402
from plot_spin import capitalize_labels, GRID, INK, INK2, SERIES, SURFACE, TRUTH_COLOR, widths_back_to_front  # noqa: E402

AXES = [('z', 'Z [mm]'), ('roll', 'Roll [deg]'), ('pitch', 'Pitch [deg]'), ('yaw', 'Yaw [deg]'), ('xy', 'Horizontal [mm]')]
I_VZ, I_W = 7, 8


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('results')
    ap.add_argument('--metric', default='prior_ndt')
    a = ap.parse_args()
    rows = [json.loads(line) for line in open(a.results) if line.strip()]
    bags = sorted({r['input'] for r in rows})
    # data[(sweep, bag)][q] = [行, ...]、sweep は 'base'・'vz'・'w'
    data = defaultdict(lambda: defaultdict(list))
    for r in rows:
        q = r['params']['ekf_localizer']['Q']
        vz, w = q[I_VZ], q[I_W]
        sweep = 'base' if vz == 0 and w == 0 else 'vz' if w == 0 else 'w' if vz == 0 else None
        if sweep:
            data[(sweep, r['input'])][vz if sweep == 'vz' else w].append(r)

    def val(r, ax):
        return r[a.metric].get(ax, {}).get('mean', np.nan)

    # 表
    def order(r):
        q = r['params']['ekf_localizer']['Q']
        return (q[I_VZ] > 0, q[I_W] > 0, q[I_VZ], q[I_W])
    labels = []
    for r in sorted(rows, key=order):
        if r['label'] not in labels:
            labels.append(r['label'])
    print(f'{a.metric} mean（録画ごと、同じ候補の回の平均）')
    print('%-16s %-28s' % ('label', 'bag') + ''.join('%10s' % ax for ax, _ in AXES))
    for lab in labels:
        for bag in bags:
            rs = [r for r in rows if r['label'] == lab and r['input'] == bag]
            if rs:
                print('%-16s %-28s' % (lab, os.path.basename(bag)[:28]) +
                      ''.join('%10.4f' % np.mean([val(r, ax) for r in rs]) for ax, _ in AXES))

    fig, axes = plt.subplots(len(AXES), 2, figsize=(10, 2.0 * len(AXES) + 0.8), facecolor=SURFACE, sharex='col')
    lw = widths_back_to_front(len(bags))
    for col, (sweep, title) in enumerate([('vz', 'Q of v_z [m²/s³]'), ('w', 'Q of omega_roll, omega_pitch [rad²/s³]')]):
        for row, (ax_name, ylabel) in enumerate(AXES):
            ax = axes[row, col]
            ax.set_facecolor(SURFACE)
            for bi, bag in enumerate(bags):
                color = SERIES[bi % len(SERIES)]
                base = data[('base', bag)].get(0.0, [])
                if base:
                    ax.axhline(np.mean([val(r, ax_name) for r in base]), color=TRUTH_COLOR, lw=lw[bi], zorder=1)
                pts = data[(sweep, bag)]
                qs = sorted(pts)
                if not qs:
                    continue
                means = [np.mean([val(r, ax_name) for r in pts[q]]) for q in qs]
                ax.plot(qs, means, color=color, lw=lw[bi], zorder=2,
                        label=os.path.basename(bag) if row == 0 and col == 0 else None)
                for q in qs:
                    ax.scatter([q] * len(pts[q]), [val(r, ax_name) for r in pts[q]], s=22, color=color,
                               edgecolor=INK, linewidth=0.5, zorder=3)
            ax.set_xscale('log')
            ax.grid(color=GRID, lw=0.8)
            ax.tick_params(colors=INK2)
            if col == 0:
                ax.set_ylabel(ylabel, color=INK2)
            if row == len(AXES) - 1:
                ax.set_xlabel(title, color=INK2)
            box_on(ax)
    handles, labs = axes[0, 0].get_legend_handles_labels()
    handles.append(plt.Line2D([], [], color=TRUTH_COLOR, lw=lw[0]))
    labs.append('Q = 0 (before)')
    fig.legend(handles, labs, loc='upper center', ncol=min(3, len(labs)), frameon=False, bbox_to_anchor=(0.5, 0.965))
    what = 'Prior at the cloud stamp − NDT solution' if a.metric == 'prior_ndt' else 'Prior at the cloud stamp − truth'
    fig.suptitle(f'{what}, mean of absolute value', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    capitalize_labels(fig)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fig.savefig(a.out, dpi=130, facecolor=SURFACE)
    print('wrote', a.out)


if __name__ == '__main__':
    main()
