#!/usr/bin/env python3
"""再生の結果（tune_run.py の 1 行 JSON を並べた jsonl）を、候補ごとに 1 回ずつの値の点で並べて描く.

  python3 plot_runs.py <出力 png> <結果の jsonl> <候補の接頭辞>=<ラベル> ... [--keys k1,k2,...] [--title <題>]

キーは tune_run.py の指標（start_max など）か、outputs_batch.py の結果の <指標>.<軸>（例 prior_ndt.roll。平均）。

候補は試行名の接頭辞（例 state8_check_ → state8_check_1, state8_check_2, ...）。指標ごとに 1 つの図に、
候補を縦に並べ、1 回ごとの値を点、平均を縦の線で描く（再生ごとのばらつきと候補の差を見るため）。
"""
import argparse
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'calibration')]
from plot_noise import box_on  # noqa: E402
from plot_spin import capitalize_labels, GRID, INK, INK2, SERIES, SURFACE  # noqa: E402

# 指標の名前と単位（transition_eval.py・tune_run.py の出力）
LABELS = {
    'start_max': 'Yaw error at rotation start, max [deg]',
    'reverse_max': 'Yaw error at reversal, max [deg]',
    'to_fwd_max': 'Yaw error at rotation to forward, max [deg]',
    'stop_max': 'Yaw error at stop, max [deg]',
    'steady_mean': 'Yaw error while rotating steadily, mean [deg]',
    'trans_max_mean': 'Yaw error at the 4 transitions, mean of max [deg]',
    'ang_mean': 'NDT angle_diff, mean [deg]',
    'xy_mean': 'XY error, mean [m]',
}
DEFAULT_KEYS = 'start_max,reverse_max,to_fwd_max,stop_max,steady_mean,ang_mean,xy_mean'
# plot_matlab_outputs.py の指標（outputs_batch.py の結果）。キーは <指標>.<軸>（平均）か <指標>.<軸>.<p95|max>
NESTED = {'prior_ndt': 'Prior − NDT solution', 'prior_truth': 'Prior − truth', 'prior_post': 'Prior − posterior'}
AXIS_UNIT = {'xy': ('horizontal', 'mm'), 'z': ('|z|', 'mm'), 'roll': ('|roll|', 'deg'), 'pitch': ('|pitch|', 'deg'),
             'yaw': ('|yaw|', 'deg')}


def value(r, key):
    """結果の行から指標の値。'a.b' は r['a']['b']['mean']、'a.b.c' は r['a']['b']['c']"""
    parts = key.split('.')
    if len(parts) == 1:
        return r[key]
    v = r
    for p in parts:
        v = v[p]
    return v['mean'] if isinstance(v, dict) else v


def label_of(key):
    if key in LABELS:
        return LABELS[key]
    parts = key.split('.')
    if len(parts) >= 2 and parts[0] in NESTED and parts[1] in AXIS_UNIT:
        name, unit = AXIS_UNIT[parts[1]]
        stat = parts[2] if len(parts) > 2 else 'mean'
        return f'{NESTED[parts[0]]}, {name}, {stat} [{unit}]'
    return key


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('results')
    ap.add_argument('groups', nargs='+', help='<試行名の接頭辞>=<ラベル>')
    ap.add_argument('--keys', default=DEFAULT_KEYS)
    ap.add_argument('--title', default='Replay of the same recording')
    a = ap.parse_args()

    rows = [json.loads(line) for line in open(a.results) if line.strip()]
    groups = []
    for g in a.groups:
        prefix, label = g.split('=', 1)
        runs = [r for r in rows if r['name'].startswith(prefix)]
        if not runs:
            sys.exit(f'no runs for {prefix}')
        groups.append((label, runs))
    keys = a.keys.split(',')

    ncol = 2
    nrow = (len(keys) + ncol - 1) // ncol
    fig, axes = plt.subplots(nrow, ncol, figsize=(10.5, 1.0 + 1.25 * nrow * max(1, len(groups) / 2)),
                             facecolor=SURFACE, squeeze=False)
    for ax, key in zip(axes.flat, keys):
        ax.set_facecolor(SURFACE)
        for gi, (label, runs) in enumerate(groups):
            v = np.array([value(r, key) for r in runs], dtype=float)
            y = len(groups) - 1 - gi
            jitter = np.linspace(-0.12, 0.12, len(v)) if len(v) > 1 else np.zeros(1)
            color = SERIES[gi % len(SERIES)]
            ax.plot([v.mean()] * 2, [y - 0.3, y + 0.3], color=color, lw=2.4, zorder=2)
            ax.scatter(v, y + jitter, s=34, color=color, edgecolor=INK, linewidth=0.6, zorder=3)
        ax.set_yticks(range(len(groups)))
        # 候補の名前は左の列だけに書く
        ax.set_yticklabels([g[0] for g in reversed(groups)] if ax.get_subplotspec().is_first_col() else [],
                           color=INK)
        ax.set_ylim(-0.6, len(groups) - 0.4)
        ax.set_xlabel(label_of(key), color=INK2)
        ax.grid(axis='x', color=GRID, lw=0.8)
        ax.tick_params(colors=INK2)
        box_on(ax)
    for ax in list(axes.flat)[len(keys):]:
        ax.set_visible(False)
    n = ', '.join(f'{label} {len(runs)}' for label, runs in groups)
    fig.suptitle(f'{a.title}\n(dots: each run, bar: mean. Runs: {n})', color=INK)
    fig.tight_layout()
    capitalize_labels(fig)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fig.savefig(a.out, dpi=130, facecolor=SURFACE)
    print('wrote', a.out)


if __name__ == '__main__':
    main()
