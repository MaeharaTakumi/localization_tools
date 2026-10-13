#!/usr/bin/env python3
"""オドメトリの omega の倍率の補正（Gazebo の wheel_separation_multiplier と推定側の odom_scale）を図にする（localizer_rl 内で実行）.

  python3 plot_scale.py <出力の接頭辞> <results の jsonl> ...

出力:
  <接頭辞>_omega.png    回転中の omega：真値・オドメトリ（倍率 1.0）・0.922 倍・Gazebo で倍率 1.085 のオドメトリ
  <接頭辞>_effect.png   yaw 誤差と NDT の angle_diff の推移（補正なし・推定側の補正・Gazebo の倍率）
  <接頭辞>_summary.png  指標の棒グラフ（2 回の平均、線は 2 回の幅）
「補正なし」と「推定側の補正」は同じ録画（spin360_ekf、倍率 1.0）の再生。「Gazebo の倍率」は別の走行（spin360_mult_rl）。
"""
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import capitalize_labels, BAND, GRID, INK, INK2, load_run, read_bag, SURFACE  # noqa: E402
from plot_spin import TRUTH_COLOR, TRUTH_LW, widths_back_to_front  # noqa: E402
from plot_velocity import odom_series, phases_of, truth_velocity  # noqa: E402

B = '/root/ros2_ws/bags/gazebo'
T = B + '/tune'
SCALE = 0.922
C_NONE, C_MULT, C_SCALE, C_TRUTH = '#eb6834', '#1baf7a', '#2a78d6', TRUTH_COLOR
# 条件を描く順（後方→手前）の線幅。後方ほど太く、点線は使わない
COND_LW = widths_back_to_front(3)
CONDITIONS = [  # (ラベル, 色, 試行名の接頭辞)
    ('no correction', C_NONE, 'v_{e}_tuned_'),
    ('Gazebo wheel_separation_multiplier 1.085 (other run)', C_MULT, 'mult_{e}_'),
    (f'localization side odom_scale omega {SCALE}', C_SCALE, 'scale_{e}_'),
]
ESTIMATORS = [('EKF (ekf_localizer)', 'ekf'), ('EKF (robot_localization)', 'rl')]   # 表示名（頭文字を大文字）と試行名の略号


def setup():
    plt.rcParams.update({
        'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
        'ytick.color': INK2, 'text.color': INK, 'axes.titlecolor': INK, 'axes.titlesize': 10,
        'axes.titleweight': 'bold', 'axes.titlelocation': 'left'})


def style(ax, bands=()):
    ax.set_facecolor(SURFACE)
    for i, (_, a, b) in enumerate(bands):
        ax.axvspan(a, b, color=BAND if i % 2 == 0 else '#f6f5f2', lw=0, zorder=0)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for spine in ('top', 'right'):
        ax.spines[spine].set_visible(False)


def bands_of(ph, t0):
    return [('left turn', ph['left turn'] - t0, ph['right turn'] - t0),
            ('right turn', ph['right turn'] - t0, ph['forward'] - t0),
            ('forward', ph['forward'] - t0, ph['stop'] - t0)]


def steady(t, y, ph):
    m = ((t > ph['left turn'] + 1.5) & (t < ph['right turn'] - 0.2))
    n = ((t > ph['right turn'] + 1.5) & (t < ph['forward'] - 0.2))
    return y[m].mean(), y[n].mean()


def fig_omega(prefix):
    panels = [('multiplier 1.0 (spin360_ekf)', B + '/spin360_ekf', True),
              ('Gazebo wheel_separation_multiplier 1.085 (spin360_mult_rl)', B + '/spin360_mult_rl', False)]
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.6), sharey=True, facecolor=SURFACE)
    for ax, (title, bag, with_scale) in zip(axes, panels):
        d = read_bag(bag)
        ph = phases_of(bag)
        t0 = ph['left turn']
        tt, _, tw = truth_velocity(d)
        ot, _, ow = odom_series(d['/wheelchair/odom'])
        style(ax, bands_of(ph, t0))
        lo, hi = -1.0, ph['forward'] - t0 + 1.5
        m, n = (tt - t0 >= lo) & (tt - t0 <= hi), (ot - t0 >= lo) & (ot - t0 <= hi)
        # 後方ほど太く：真値 → オドメトリ → 0.922 倍
        ax.plot(tt[m] - t0, tw[m], color=C_TRUTH, lw=TRUTH_LW, label='truth', zorder=2)
        ax.plot(ot[n] - t0, ow[n], color=C_MULT if not with_scale else C_NONE, lw=2.2,
                label='odometry' + ('' if with_scale else ' (multiplier 1.085)'), zorder=3)
        if with_scale:
            ax.plot(ot[n] - t0, ow[n] * SCALE, color=C_SCALE, lw=1.2, label=f'odometry x {SCALE}', zorder=4)
        tl, tr = steady(tt, tw, ph)
        ol, orr = steady(ot, ow, ph)
        note = f'steady omega  truth {tl:+.3f} / {tr:+.3f}\n              odometry {ol:+.3f} / {orr:+.3f}'
        if with_scale:
            note += f'\n              x {SCALE}  {ol * SCALE:+.3f} / {orr * SCALE:+.3f}'
        ax.text(0.99, 0.97, note, transform=ax.transAxes, ha='right', va='top', fontsize=8.5,
                color=INK, family='monospace')
        for label, a, b in bands_of(ph, t0):
            ax.text((a + b) / 2, 0.03, label, transform=ax.get_xaxis_transform(), ha='center',
                    va='bottom', fontsize=9, color=INK2)
        ax.set_title(title)
        ax.set_xlabel('time since start of left turn [s] (sim time)')
        ax.set_xlim(lo, hi)
        ax.legend(loc='lower left', bbox_to_anchor=(0.0, 0.08), frameon=False, fontsize=9)
    axes[0].set_ylabel('omega [rad/s]')
    fig.suptitle('Odometry omega vs truth while turning (commanded +-1 rad/s)', x=0.05, ha='left',
                 fontsize=12, fontweight='bold', color=INK)
    fig.tight_layout()
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(f'{prefix}_omega.png', dpi=130, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {prefix}_omega.png')


def fig_effect(prefix):
    fig, axes = plt.subplots(2, 2, figsize=(15, 8.5), sharex=True, facecolor=SURFACE)
    for col, (name, e) in enumerate(ESTIMATORS):
        ref = load_run(f'{T}/v_{e}_tuned_1')
        ph = ref['phases']
        bands = [('left turn', ph['left turn'], ph['right turn']),
                 ('right turn', ph['right turn'], ph['forward']), ('forward', ph['forward'], ph['stop'])]
        lo, hi = -2.0, ph['stop'] + 3.0
        for row, (title, tk, get) in enumerate((
                ('yaw error [deg]', 't', lambda r: r['err'][:, 2]),
                ('NDT angle_diff [deg]', 'ang_t', lambda r: r['ang']))):
            ax = axes[row][col]
            style(ax, bands)
            for i, ((label, color, pat), lw) in enumerate(zip(CONDITIONS, COND_LW)):
                r = load_run(f'{T}/{pat.format(e=e)}1')
                x, y = r[tk], get(r)
                m = (x >= lo) & (x <= hi)
                ax.plot(x[m], y[m], color=color, lw=lw, zorder=3 + i)
            if row == 0:
                ax.axhline(0, color=INK2, lw=0.8, zorder=2)
                for label, a, b in bands:
                    ax.text((a + b) / 2, 1.0, label, transform=ax.get_xaxis_transform(), ha='center',
                            va='bottom', fontsize=9, color=INK2)
            ax.set_title(f'{name}: {title}', pad=16 if row == 0 else 6)
            ax.set_xlim(lo, hi)
        axes[1][col].set_xlabel('time since start of left turn [s] (sim time)')
    handles = [plt.Line2D([], [], color=c, lw=w, label=lab) for (lab, c, _), w in zip(CONDITIONS, COND_LW)]
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.04, 0.965), ncol=3, frameon=False)
    fig.suptitle('Effect of the omega scale correction (no correction and odom_scale: same recording replayed)',
                 x=0.04, ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(f'{prefix}_effect.png', dpi=130, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {prefix}_effect.png')


def fig_summary(prefix, results):
    metrics = [('steady_mean', 'yaw error while turning steadily [deg]'),
               ('ang_mean', 'NDT angle_diff, mean [deg]'),
               ('trans_max_mean', 'yaw error peak at velocity changes, mean of 4 [deg]')]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), facecolor=SURFACE)
    width = 0.26
    for ax, (key, title) in zip(axes, metrics):
        style(ax)
        labels = []
        for gi, (name, e) in enumerate(ESTIMATORS):
            for ci, (label, color, pat) in enumerate(CONDITIONS):
                vals = [results[pat.format(e=e) + str(i)][key] for i in (1, 2)]
                x = gi + (ci - 1) * (width + 0.02)
                mean = float(np.mean(vals))
                ax.bar(x, mean, width, color=color, zorder=3)
                ax.plot([x, x], [min(vals), max(vals)], color=INK, lw=1.2, zorder=4)
                labels.append((x, max(vals), mean))
        top = max(v for _, v, _ in labels)
        ax.set_ylim(0, top * 1.18)
        for x, vmax, mean in labels:   # 値は棒の上に書く（色だけに頼らない）
            ax.text(x, vmax + top * 0.02, f'{mean:.2f}', ha='center', va='bottom', fontsize=8.5,
                    color=INK, zorder=5)
        ax.set_xticks(range(len(ESTIMATORS)))
        ax.set_xticklabels([n for n, _ in ESTIMATORS])
        ax.set_title(title, fontsize=9.5)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c, label=lab) for lab, c, _ in CONDITIONS]
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.04, 0.93), ncol=3, frameon=False)
    fig.suptitle('Summary (bar = mean of 2 runs, line = range)', x=0.04, ha='left', fontsize=12,
                 fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.84))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(f'{prefix}_summary.png', dpi=130, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {prefix}_summary.png')


def main():
    prefix = sys.argv[1]
    results = {}
    for path in sys.argv[2:]:
        for line in open(path):
            if line.strip():
                r = json.loads(line)
                results[r['name']] = r
    setup()
    fig_omega(prefix)
    fig_effect(prefix)
    fig_summary(prefix, results)


if __name__ == '__main__':
    main()
