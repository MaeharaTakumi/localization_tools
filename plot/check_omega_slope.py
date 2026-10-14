#!/usr/bin/env python3
"""omega が変わるとき（回転開始・反転）の傾きを、真値・オドメトリ・車輪の角速度で比べる（test_maehara 内で実行）.

  python3 check_omega_slope.py <出力 png> <bag>   （bag に /wheelchair/joint_states が無ければ、真値とオドメトリだけ）

- 立ち上がり時間：変化の 10 % から 90 % まで [ms]、最大の傾き |domega/dt| [rad/s^2]（0.02 s の差分）
- 車輪の角速度（joint_states）から omega = r (wR - wL) / b を計算し、diff_drive_controller と同じ 0.2 s の移動平均を
  掛けたもの（再現）がオドメトリと重なるかを見る。値はすべて odom_scale の 0.922 倍（真値と比べるため）
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import capitalize_labels, GRID, INK, INK2, read_bag, SURFACE, TRUTH_COLOR, TRUTH_LW  # noqa: E402
from plot_velocity import odom_series, phases_of, truth_velocity  # noqa: E402

R, B, SCALE, WINDOW = 0.1325, 0.492, 0.922, 0.2


def joint_omega(d):
    t, w = [], []
    for _, m in d['/wheelchair/joint_states']:
        il, ir = m.name.index('left_wheel_joint'), m.name.index('right_wheel_joint')
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        w.append(R * (m.velocity[ir] - m.velocity[il]) / B)
    o = np.argsort(t)
    return np.array(t)[o], np.array(w)[o]


def moving_average(t, y, span):
    """各時刻 t で、直近 span 秒の平均（diff_drive_controller と同じく窓の終わりの時刻の値）."""
    c = np.concatenate([[0.0], np.cumsum(y[:-1] * np.diff(t))])   # 区分定数の積分
    start = np.interp(t - span, t, c)
    ok = t - span >= t[0]
    out = np.full(len(t), np.nan)
    out[ok] = (c[ok] - start[ok]) / span
    return out


def rise(t, y, ts):
    """ts の前後の値から、10 % -> 90 % の時間 [ms] と最大の傾き [rad/s^2]."""
    a = y[(t >= ts - 0.5) & (t <= ts - 0.05)].mean()
    b = y[(t >= ts + 0.8) & (t <= ts + 1.3)].mean()
    s = np.sign(b - a)
    m = (t >= ts - 0.2) & (t <= ts + 0.8)
    tt, yy = t[m], s * (y[m] - a) / abs(b - a)
    t10 = tt[np.argmax(yy >= 0.1)]
    t90 = tt[np.argmax(yy >= 0.9)]
    grid = np.arange(ts - 0.1, ts + 0.6, 0.02)
    g = np.interp(grid, t, y)
    slope = np.abs(np.diff(g)).max() / 0.02
    return (t90 - t10) * 1000.0, slope


def main():
    out, bag = sys.argv[1:3]
    d = read_bag(bag)
    ph = phases_of(bag)
    tt, _, tw = truth_velocity(d)
    ot, _, ow = odom_series(d['/wheelchair/odom'])
    series = [('truth', tt, tw, TRUTH_COLOR, TRUTH_LW),
              (f'/wheelchair/odom x {SCALE}', ot, ow * SCALE, '#eb6834', 2.6)]
    if '/wheelchair/joint_states' in d:
        jt, jw = joint_omega(d)
        series += [(f'wheel joint velocity, 0.2 s moving average (reproduced) x {SCALE}', jt,
                    moving_average(jt, jw, WINDOW) * SCALE, '#4a3aa7', 1.8),
                   (f'wheel joint velocity (no average) x {SCALE}', jt, jw * SCALE, '#2a78d6', 1.1)]
    print(f"{'':<62}{'rotation start':>28}{'reverse':>28}")
    print(f"{'':<62}" + f"{'10-90% [ms]':>14}{'max slope':>14}" * 2)
    for name, t, y, _, _ in series:
        cells = []
        for key in ('left turn', 'right turn'):
            m = np.isfinite(y)
            r_ms, slope = rise(t[m], y[m], ph[key])
            cells += [f'{r_ms:14.0f}', f'{slope:14.1f}']
        print(f'{name:<62}' + ''.join(cells))
    if len(series) > 2:
        m = (ot > jt[0] + WINDOW) & (ot < jt[-1])
        diff = ow[m] - np.interp(ot[m], jt, moving_average(jt, jw, WINDOW))
        print(f'odom vs reproduced 0.2 s average: RMS {np.sqrt(np.nanmean(diff ** 2)):.4f} rad/s')

    plt.rcParams.update({'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': INK2, 'ytick.color': INK2, 'text.color': INK, 'axes.titlecolor': INK,
                         'axes.titlesize': 10, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left'})
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8), facecolor=SURFACE)
    for ax, (title, key) in zip(axes, (('omega at rotation start', 'left turn'),
                                       ('omega at reverse (+1 -> -1 rad/s)', 'right turn'))):
        ax.set_facecolor(SURFACE)
        ax.grid(axis='y', color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for spine in ('top', 'right'):
            ax.spines[spine].set_visible(False)
        ts = ph[key]
        for i, (name, t, y, color, lw) in enumerate(series):   # 後方ほど太く
            m = (t >= ts - 0.3) & (t <= ts + 0.6)
            ax.plot(t[m] - ts, y[m], color=color, lw=lw, zorder=2 + i, label=name)
        ax.axvline(0, color=INK2, lw=0.8, zorder=1)
        ax.axvspan(0, WINDOW, color='#f0efec', lw=0, zorder=0)
        ax.text(WINDOW / 2, 0.02, '0.2 s window', transform=ax.get_xaxis_transform(), ha='center',
                va='bottom', fontsize=8.5, color=INK2)
        ax.set_xlim(-0.3, 0.6)
        ax.set_title(title)
        ax.set_xlabel('time from command switch [s]')
    axes[0].set_ylabel('omega [rad/s]')
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left', bbox_to_anchor=(0.04, 0.95), ncol=2, frameon=False)
    fig.suptitle('Why the odometry omega changes with a gentler slope than the truth', x=0.04, ha='left',
                 fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.83))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f'saved {out}')


if __name__ == '__main__':
    main()
