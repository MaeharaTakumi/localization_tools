#!/usr/bin/env python3
"""オドメトリの大きさ（|v|・|omega|）と R の関係（test_maehara 内で実行）.

  python3 plot_odom_r.py <出力 png> <bag> ...

- 線：odom_covariance の R（R_v = R_odom[0] + a0 v² + a1 omega²、R_omega = R_odom[1] + a2 v² + a3 omega²）を、
  今の設定（alpha 0）と dev_log.md の 24. で試した alpha で描く（加速度の項は 0）
- 点：指令が一定の区間（オドメトリが指令の変化の半分まで応答した 0.5 s 後〜次の変化の 0.1 s 前、1 s 以上）ごとに、
  （応答から数えるのは、最初の指令が届くまで遅れることがあるため。後退の録画では 0.7 s 遅れて動き出した）
  - 真値なし：オドメトリ（odom_scale 後）から前後 0.5 s の局所 1 次式を除いた残差の分散（estimate_noise.py と同じ）
  - 真値との差：オドメトリと真値（/wheelchair/pose の微分）の差の 2 乗平均（一定のずれも含む。Gazebo だけ）
  横軸は区間の指令の |v|・|omega|
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]
from estimate_noise import command_segments, odom_residuals, stamp  # noqa: E402
from plot_spin import capitalize_labels, GRID, INK2, read_bag, SERIES, SURFACE  # noqa: E402
from plot_velocity import truth_velocity  # noqa: E402

R_ODOM = (2.5e-3, 5.0e-3)
ODOM_SCALE = (1.0, 0.922)
SETTLE, MARGIN, MIN_LEN, HALF = 0.5, 0.1, 1.0, 0.5
CMD = '/wheelchair/diff_drive_controller/cmd_vel'
# (表示名, [a0, a1, a2, a3])。dev_log.md の 24.
ALPHAS = [('Current: alpha 0 (fixed R)', [0, 0, 0, 0]),
          ('Alpha 0.005 (v² to R_v, omega² to R_omega)', [0.005, 0, 0, 0.005]),
          ('Alpha 0.05 (omega² to R_omega)', [0, 0, 0, 0.05])]
C_FREE, C_TRUTH = '#e87ba4', INK2   # 点：真値なし（参照パレット slot 5）、真値との差（文字色の灰色）


def segment_stats(bag):
    """区間ごとの (|v| 指令, |omega| 指令, 真値なしの分散 [v, omega], 真値との差の 2 乗平均 [v, omega])."""
    d = read_bag(bag, ['/clock', '/wheelchair/odom', '/wheelchair/pose', CMD])
    clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d['/clock']])

    def to_sim(ts):
        return np.interp(ts, clk[:, 0], clk[:, 1])

    od = d['/wheelchair/odom']
    ot = np.array([stamp(m) for _, m in od])
    ov = np.array([m.twist.twist.linear.x for _, m in od]) * ODOM_SCALE[0]
    ow = np.array([m.twist.twist.angular.z for _, m in od]) * ODOM_SCALE[1]
    tt, tv, tw = truth_velocity(d)
    out = []
    prev = (0.0, 0.0)
    for t0, t1, v, w in command_segments(d, CMD, to_sim, ot[0], ot[-1], 0.5):
        # オドメトリが指令の変化の半分まで応答した時刻（応答しなければ指令の時刻）
        dv, dw = v - prev[0], w - prev[1]
        resp = (ot >= t0) & (ot < t1) & ((np.abs(ov / ODOM_SCALE[0] - prev[0]) >= 0.5 * abs(dv)) if abs(dv) > 1e-3 else True) \
            & ((np.abs(ow / ODOM_SCALE[1] - prev[1]) >= 0.5 * abs(dw)) if abs(dw) > 1e-3 else True)
        t_resp = ot[resp][0] if resp.any() else t0
        prev = (v, w)
        a, b = t_resp + SETTLE, t1 - MARGIN
        if b - a < MIN_LEN or (abs(v) < 1e-3 and abs(w) < 1e-3):
            continue
        m = (ot >= a) & (ot <= b)
        r = odom_residuals(ot[m], ov[m], ow[m], HALF)
        free = (np.nanvar(r['v']), np.nanvar(r['omega']))
        ev = ov[m] - np.interp(ot[m], tt, tv)
        ew = ow[m] - np.interp(ot[m], tt, tw)
        out.append((abs(v), abs(w), free, (np.mean(ev ** 2), np.mean(ew ** 2))))
    return out


def main():
    out, bags = sys.argv[1], sys.argv[2:]
    stats = [s for bag in bags for s in segment_stats(bag)]
    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
                         'ytick.color': INK2, 'axes.titlesize': 10.5, 'legend.frameon': False})
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.4), facecolor=SURFACE)
    for k, (ax, name, unit, xmax) in enumerate(zip(axes, ('V', 'Omega'), ('(m/s)²', '(rad/s)²'), (1.7, 1.15))):
        ax.set_facecolor(SURFACE)
        x = np.linspace(0, xmax, 200)
        # R の式：横軸の量（v か omega）だけを動かし、もう一方は 0
        # その量の R を変える候補だけ描く（色は候補ごとに固定）
        shown = [(label, a, color) for (label, a), color in zip(ALPHAS, SERIES)
                 if (a[0] if k == 0 else a[3]) > 0 or not any(a)]
        for (label, a, color), lw in zip(shown, (2.6, 1.9, 1.3)):
            r = R_ODOM[k] + (a[0] if k == 0 else a[3]) * x ** 2
            ax.plot(x, r, color=color, lw=lw, label=label, zorder=2)
        xs = [s[k] for s in stats]
        ax.scatter(xs, [s[3][k] for s in stats], s=34, facecolor='none', edgecolor=C_TRUTH, lw=1.2, zorder=3,
                   label='Measured: mean square of (odometry − truth) per segment (Gazebo only)')
        ax.scatter(xs, [s[2][k] for s in stats], s=30, color=C_FREE, edgecolor=SURFACE, lw=0.8, zorder=4,
                   label='Measured: residual variance without truth per segment')
        ax.set_yscale('log')
        ax.set_xlim(0, xmax)
        ax.set_xlabel(f'|{name.lower() if k == 0 else "omega"}| of the command [{"m/s" if k == 0 else "rad/s"}]')
        ax.set_ylabel(f'R_{"v" if k == 0 else "omega"} (variance) [{unit}]')
        ax.set_title(f'{name}: R used by the EKF vs measured odometry error', loc='left')
        ax.grid(color=GRID, lw=0.6, which='major')
    handles, labels = axes[1].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left', ncol=2, bbox_to_anchor=(0.01, 0.93), fontsize=9)
    fig.suptitle('Odometry magnitude and R (odom_covariance)', x=0.01, ha='left', fontsize=12, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.80))
    capitalize_labels(fig)
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f'saved {out}')
    for s in sorted(stats):
        print(f'|v| {s[0]:4.2f} |w| {s[1]:4.2f}  free var v {s[2][0]:.2e} w {s[2][1]:.2e}  '
              f'truth ms v {s[3][0]:.2e} w {s[3][1]:.2e}')


if __name__ == '__main__':
    main()
