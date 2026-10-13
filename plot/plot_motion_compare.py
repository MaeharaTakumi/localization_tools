#!/usr/bin/env python3
"""推定器の比較（eval/compare_run.py）の図（localizer_rl 内で実行）.

  python3 plot_motion_compare.py traj <出力 png> <動き>=<bag> ...            真値の軌跡（地図の壁の上）
  python3 plot_motion_compare.py summary <出力 png> <results.jsonl>          動きごと・推定器ごとの指標（4 回の平均と幅）
  python3 plot_motion_compare.py series <出力 png> <題> <ラベル>=<bag> ...   誤差の時間推移（stamp の時刻・実時間）
  python3 plot_motion_compare.py report <出力の接頭辞> <results.jsonl>      上の 3 つをまとめて（dev_log.md の 22. の図）

動きの名前は compare_run.py の motion（accel, curve, slalom, reverse, spin, stops）。
"""
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]
from plot_spin import (BAND, capitalize_labels, GRID, INK, INK2, SERIES, SURFACE, TRUTH_COLOR,  # noqa: E402
                       widths_back_to_front)

MAP = '/root/ros2_ws/map/SC10_4_0.05.pcd'
FLOOR_Z = 12.3
MOTION_LABELS = {
    'accel': 'Accelerate / stop', 'curve': 'Curves (figure 8)', 'slalom': 'Slalom', 'reverse': 'Reverse',
    'spin': 'Spin in place', 'stops': 'Stops (13)',
}
# 結果の label（既定は estimator）ごとの凡例と色
#   robot_localization  R_ndt は robot_localization.yaml（ekf_localizer と同じ値。launch を直す前に実際に動いていた設定）
#   rl_rndt_small       R_ndt は gazebo.yaml（止まった状態で測った小さい値。launch を直して効くようになった）
#   rl_timer            rl_rndt_small の /ekf_pose を timer（50 Hz、現在時刻）で出す（参考）
#   rl_final            2026-10-13 からの既定（R_ndt は robot_localization.yaml、timer）
EST_LABELS = {'ekf_localizer': 'EKF (ekf_localizer)',
              'robot_localization': 'RL (robot_localization), R_ndt same as ekf_localizer',
              'rl_rndt_small': 'RL (robot_localization), small R_ndt (gazebo.yaml)',
              'rl_timer': 'RL, small R_ndt, timer output (reference)',
              'rl_final': 'RL (robot_localization), new default: timer output'}
EST_COLORS = {'ekf_localizer': SERIES[0], 'robot_localization': SERIES[1], 'rl_rndt_small': SERIES[2],
              'rl_timer': SERIES[3], 'rl_final': '#e87ba4'}   # 参照パレットの slot 5（magenta）


def style():
    plt.rcParams.update({
        'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
        'ytick.color': INK2, 'text.color': INK, 'axes.titlecolor': INK, 'axes.titlesize': 10,
        'legend.frameon': False,
    })


def map_walls():
    """地図の壁（床から 0.3〜1.8 m の点を 5 cm の格子に間引いたもの）."""
    cache = '/tmp/map_walls.npy'
    if os.path.exists(cache):
        return np.load(cache)
    with open(MAP, 'rb') as f:
        while not f.readline().startswith(b'DATA'):
            pass
        p = np.frombuffer(f.read(), dtype=np.float32).reshape(-1, 3)
    p = p[(p[:, 2] > FLOOR_Z + 0.3) & (p[:, 2] < FLOOR_Z + 1.8)]
    cells = np.unique(np.round(p[:, :2] / 0.05).astype(np.int64), axis=0) * 0.05
    np.save(cache, cells)
    return cells


def traj(out, items):
    from compare_run import load, phases_of
    style()
    walls = map_walls()
    n = len(items)
    cols = 2
    rows = (n + 1) // 2
    fig, axes = plt.subplots(rows, cols, figsize=(11, 3.3 * rows), facecolor=SURFACE)
    for ax, (motion, bag) in zip(axes.flat, items):
        tt, tv, *_ = load(bag)
        ph = phases_of(bag)
        m = (tt >= min(ph.values()) - 1.0) & (tt <= max(ph.values()) + 2.0)
        x, y = tv[m, 0], tv[m, 1]
        cx, cy = (x.min() + x.max()) / 2, (y.min() + y.max()) / 2
        half_w = max(x.max() - x.min(), 2 * (y.max() - y.min()), 6.0) / 2 + 1.0
        half_h = half_w / 2
        sel = (np.abs(walls[:, 0] - cx) < half_w) & (np.abs(walls[:, 1] - cy) < half_h)
        ax.set_facecolor(SURFACE)
        ax.scatter(walls[sel, 0], walls[sel, 1], s=0.6, color=TRUTH_COLOR, lw=0, zorder=1)
        ax.plot(x, y, color=SERIES[0], lw=1.6, zorder=3)
        ax.plot(x[0], y[0], 'o', color=INK, ms=6, zorder=4)
        ax.annotate('Start', (x[0], y[0]), xytext=(6, 6), textcoords='offset points', fontsize=8, color=INK2)
        ax.set_xlim(cx - half_w, cx + half_w)
        ax.set_ylim(cy - half_h, cy + half_h)
        ax.set_aspect('equal')
        ax.grid(color=GRID, lw=0.6)
        ax.set_title(MOTION_LABELS.get(motion, motion), loc='left')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
    for ax in list(axes.flat)[n:]:
        ax.set_visible(False)
    fig.suptitle('Ground truth paths of the test motions (walls from the map)', x=0.02, ha='left',
                 fontsize=12, fontweight='bold')
    fig.tight_layout()
    capitalize_labels(fig)
    fig.savefig(out, dpi=130, facecolor=SURFACE)


SUMMARY_METRICS = [
    ('yaw_mean', 1, 'Yaw error, mean [deg]'),
    ('trans_yaw_max', 1, 'Yaw error at command changes [deg]'),
    ('z_mean', 1e3, 'Z error, mean [mm]'),
    ('tilt_mean', 1, 'Roll / pitch error, mean [deg]'),
    ('xy_raw_max', 1e3, 'XY error, max [mm]'),
    ('rt_yaw_mean', 1, 'Real-time yaw error, mean [deg]'),
    ('rt_trans_yaw_max', 1, 'Real-time yaw error at command changes [deg]'),
    ('age_mean', 1, 'Age of the latest pose, mean [ms]'),
]


def load_results(path):
    rows = [json.loads(line) for line in open(path) if line.strip()]
    return [r for r in rows if 'error' not in r]


def summary(out, results, labels=None,
            title='Comparison of ekf_localizer and robot_localization: replay of the same recordings, 4 runs each'):
    """labels を渡すとその条件だけ描く."""
    style()
    rows = load_results(results)
    if labels:
        rows = [r for r in rows if r.get('label', r['estimator']) in labels]
    motions = [m for m in MOTION_LABELS if any(r['motion'] == m for r in rows)]
    for r in rows:
        r.setdefault('label', r['estimator'])
    ests = [e for e in EST_LABELS if any(r['label'] == e for r in rows)]
    fig, axes = plt.subplots(2, 4, figsize=(15, 7.2), facecolor=SURFACE, sharey=True)
    ypos = {m: i for i, m in enumerate(reversed(motions))}
    offs = {e: 0.26 * ((len(ests) - 1) / 2 - i) for i, e in enumerate(ests)}
    for ax, (key, scale, panel_title) in zip(axes.flat, SUMMARY_METRICS):
        ax.set_facecolor(SURFACE)
        for i, m in enumerate(motions):
            if i % 2 == 0:
                ax.axhspan(ypos[m] - 0.5, ypos[m] + 0.5, color=BAND, lw=0, zorder=0)
        for e in ests:
            for m in motions:
                v = np.array([r[key] for r in rows if r['motion'] == m and r['label'] == e]) * scale
                if not len(v):
                    continue
                y = ypos[m] + offs[e]
                ax.plot([v.min(), v.max()], [y, y], color=EST_COLORS[e], lw=2.0, solid_capstyle='round', zorder=2)
                ax.plot(v.mean(), y, 'o', color=EST_COLORS[e], ms=7, mec=SURFACE, mew=1.5, zorder=3,
                        label=EST_LABELS[e] if m == motions[0] else None)
        ax.set_title(panel_title, loc='left', fontsize=9.5)
        ax.set_xlim(left=0)
        ax.grid(axis='x', color=GRID, lw=0.6)
        ax.set_yticks([ypos[m] for m in motions])
        ax.set_yticklabels([MOTION_LABELS[m] for m in motions])
        ax.set_ylim(-0.5, len(motions) - 0.5)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left', ncol=2, bbox_to_anchor=(0.01, 0.955))
    fig.suptitle(title + ' (dot = mean, line = range)', x=0.01, ha='left', fontsize=12, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.89))
    capitalize_labels(fig)
    fig.savefig(out, dpi=130, facecolor=SURFACE)


def series_plot(out, title, items):
    from compare_run import series
    style()
    runs = [(label, series(bag)) for label, bag in items]
    panels = [
        ('XY error [mm]', lambda s: s['err'][:, 1] * 1e3, 't'),
        ('Z error [mm]', lambda s: s['err'][:, 2] * 1e3, 't'),
        ('Pitch error [deg]', lambda s: s['err'][:, 4], 't'),
        ('Yaw error [deg]', lambda s: s['err'][:, 5], 't'),
        ('Real-time yaw error [deg]', lambda s: s['rt'][:, 5], 'rt_t'),
    ]
    fig, axes = plt.subplots(len(panels), 1, figsize=(11, 10.5), sharex=True, facecolor=SURFACE)
    ph = runs[0][1]['phases']
    names = sorted(ph, key=ph.get)
    edges = [ph[n] for n in names] + [ph[names[-1]] + 2.0]
    for ax, (ylabel, fn, tkey) in zip(axes, panels):
        ax.set_facecolor(SURFACE)
        for i in range(len(names)):
            ax.axvspan(edges[i], edges[i + 1], color=BAND if i % 2 == 0 else '#f6f5f2', lw=0, zorder=0)
        ax.axhline(0, color=INK2, lw=0.8, zorder=1)
        for (label, s), color, lw in zip(runs, SERIES, widths_back_to_front(len(runs))):
            ax.plot(s[tkey], fn(s), color=color, lw=lw, label=label, zorder=3)
        ax.set_ylabel(ylabel)
        ax.grid(axis='y', color=GRID, lw=0.6)
    for n, a in zip(names, edges):
        axes[0].annotate(n, (a, 1.0), xycoords=('data', 'axes fraction'), xytext=(2, 2),
                         textcoords='offset points', fontsize=7, color=INK2, rotation=30, ha='left', va='bottom')
    axes[-1].set_xlabel('Time from the first command [s]')
    axes[-1].set_xlim(-2.0, edges[-1])
    axes[0].legend(loc='upper left', bbox_to_anchor=(0.0, 1.55), ncol=len(runs))
    fig.suptitle(title, x=0.01, ha='left', fontsize=12, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    capitalize_labels(fig)
    fig.savefig(out, dpi=130, facecolor=SURFACE)


# report の入力：動きごとの録画（軌跡）と、時間推移に描く条件（結果の label）
REPORT_BAGS = {'accel': '/root/bags/drive_accel', 'curve': '/root/bags/drive_curve',
               'slalom': '/root/bags/drive_slalom', 'reverse': '/root/bags/drive_reverse',
               'spin': '/root/bags/spin360_w1_ekf2', 'stops': '/root/bags/stops_ekf'}
REPORT_SERIES = [('ekf_localizer', 'EKF (ekf_localizer)'), ('robot_localization', 'RL, R_ndt same as ekf_localizer'),
                 ('rl_rndt_small', 'RL, small R_ndt (gazebo.yaml)')]


def typical_run(rows, motion, label):
    """4 回のうち、yaw の平均と切り替え時の yaw の両方が中央値に近い回（時間推移の図に使う）."""
    rs = [r for r in rows if r['motion'] == motion and r.get('label', r['estimator']) == label]
    my = float(np.median([r['yaw_mean'] for r in rs]))
    mt = float(np.median([r['trans_yaw_max'] for r in rs]))
    return min(rs, key=lambda r: abs(r['yaw_mean'] / my - 1) + abs(r['trans_yaw_max'] / mt - 1))['name']


def report(prefix, results):
    rows = load_results(results)
    traj(f'{prefix}_paths.png', list(REPORT_BAGS.items()))
    print(f'saved {prefix}_paths.png')
    summary(f'{prefix}_summary.png', results, ['ekf_localizer', 'robot_localization', 'rl_rndt_small', 'rl_timer'])
    print(f'saved {prefix}_summary.png')
    if any(r.get('label') == 'rl_final' for r in rows):
        summary(f'{prefix}_final.png', results, ['ekf_localizer', 'rl_final'],
                'New defaults: ekf_localizer vs robot_localization with timer output, 4 runs each')
        print(f'saved {prefix}_final.png')
    for m in ('accel', 'curve', 'slalom', 'reverse', 'spin'):   # 止まる走行は区間が多いので描かない
        items = [(text, f'/root/bags/tune/{typical_run(rows, m, label)}') for label, text in REPORT_SERIES]
        series_plot(f'{prefix}_series_{m}.png',
                    f'{MOTION_LABELS[m]}: error over time (replay; one typical run each)', items)
        print(f'saved {prefix}_series_{m}.png')


def main():
    cmd, out = sys.argv[1:3]
    if cmd == 'traj':
        traj(out, [a.split('=', 1) for a in sys.argv[3:]])
    elif cmd == 'summary':
        summary(out, sys.argv[3])
    elif cmd == 'series':
        series_plot(out, sys.argv[3], [a.split('=', 1) for a in sys.argv[4:]])
    elif cmd == 'report':
        report(out, sys.argv[3])
    else:
        raise SystemExit(__doc__)


if __name__ == '__main__':
    main()
