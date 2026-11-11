#!/usr/bin/env python3
"""drive/run_test.sh の録画（推定器ごと）から、/ekf_pose の誤差と NDT の angle_diff の時間推移を描く.

  python3 plot_spin.py <出力 png> <ラベル>=<bag>[@<姿勢のトピック>] ...
  姿勢のトピックは map -> LiDAR の PoseWithCovarianceStamped（既定 /ekf_pose。NDT 単体なら /ndt_pose）

横軸は「左回転」の開始からの sim time [s]（<bag>_phases.txt）。
真値の LiDAR は /wheelchair/pose（base_footprint、header なし。bag の受信時刻を /clock で sim time に直す）に
URDF の取付 base_footprint -> velodyne = (-0.17, 0, 1.1377)、回転 0（common/gazebo_model.py）を掛けたもの。
bag の読み込み・図の書式などの共通部品は common/（bag_io.py・plot_style.py・pose_math.py・gazebo_model.py）。
angle_diff は /localization_debug の data[4]（header なし。同じく受信時刻から sim time に直す）。
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'plot', 'eval', 'calibration')]   # 共通部品（common/）などを import する
from bag_io import phases_of, read_bag  # noqa: E402
from gazebo_model import URDF_MOUNT  # noqa: E402
from plot_style import capitalize_labels, INK, INK2, SERIES, setup, style, SURFACE, widths_back_to_front  # noqa: E402
from plot_style import BAND, BAND_ALT  # noqa: E402
from pose_math import matrix_to_vec, pose_msg_to_matrix, vec_to_matrix, wrap  # noqa: E402


def load_run(path, pose_topic='/ekf_pose'):
    d = read_bag(path)
    clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d['/clock']])

    def to_sim(ts):
        return np.interp(ts, clk[:, 0], clk[:, 1])

    tt = to_sim([t for t, _ in d['/wheelchair/pose']])
    tv = np.array([matrix_to_vec(pose_msg_to_matrix(m)) for _, m in d['/wheelchair/pose']])
    o = np.argsort(tt)
    tt, tv = tt[o], tv[o]
    tv[:, 5] = np.unwrap(tv[:, 5])

    est_t, err = [], []
    for _, m in d[pose_topic]:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not tt[0] <= t <= tt[-1]:
            continue
        ref = vec_to_matrix([np.interp(t, tt, tv[:, i]) for i in range(6)]) @ URDF_MOUNT
        est = pose_msg_to_matrix(m.pose.pose)
        e = est[:3, 3] - ref[:3, 3]
        dyaw = wrap(matrix_to_vec(est)[5] - matrix_to_vec(ref)[5])
        est_t.append(t)
        err.append((math.hypot(e[0], e[1]), e[2], math.degrees(dyaw)))
    dbg = d.get('/localization_debug', [])
    ang_t = to_sim([t for t, _ in dbg])
    ang = np.array([m.data[4] for _, m in dbg])

    phases = phases_of(path)
    t0 = phases['left turn']
    rel = {k: v - t0 for k, v in phases.items()}
    # 真値の動き（指令どおり動いたか）
    def at(t, i):
        return np.interp(t, tt, tv[:, i])
    motion = {
        'left turn yaw [deg]': math.degrees(at(phases['right turn'], 5) - at(phases['left turn'], 5)),
        'right turn yaw [deg]': math.degrees(at(phases['forward'], 5) - at(phases['right turn'], 5)),
        'forward [m]': math.hypot(at(phases['stop'], 0) - at(phases['forward'], 0),
                                  at(phases['stop'], 1) - at(phases['forward'], 1)),
    }
    return {
        't': np.array(est_t) - t0, 'err': np.array(err),
        'ang_t': np.array(ang_t) - t0, 'ang': ang, 'phases': rel, 'motion': motion,
    }


def summarize(label, r):
    t_stop = r['phases']['stop']
    m = (r['t'] >= 0) & (r['t'] <= t_stop + 2.0)
    e = r['err'][m]
    ma = (r['ang_t'] >= 0) & (r['ang_t'] <= t_stop + 2.0)
    a = r['ang'][ma]
    pre = r['err'][(r['t'] < 0) & (r['t'] > -3.0)]
    print(f'## {label}')
    print('  truth motion: ' + ', '.join(f'{k} {v:.1f}' for k, v in r['motion'].items()))
    print(f'  before motion (-3..0 s): xy mean {pre[:, 0].mean():.3f} m, yaw mean {pre[:, 2].mean():+.2f} deg')
    print(f'  during motion (0..stop+2 s, n={len(e)}): xy mean {e[:, 0].mean():.3f} / max {e[:, 0].max():.3f} m'
          f' | z mean {e[:, 1].mean():+.3f} / max |{np.abs(e[:, 1]).max():.3f}| m'
          f' | yaw mean |{np.abs(e[:, 2]).mean():.2f}| / max |{np.abs(e[:, 2]).max():.2f}| deg')
    print(f'  NDT angle_diff (n={len(a)}): mean {a.mean():.2f} / max {a.max():.2f} deg')


def plot(out, runs, colors=None):
    """runs（[(ラベル, load_run の結果)]）の誤差と angle_diff の時間推移を描く。colors を渡すと系列の色をそれにする."""
    colors = colors or SERIES
    setup()
    fig, all_axes = plt.subplots(5, 1, figsize=(10, 11), sharex=True, facecolor=SURFACE,
                                 gridspec_kw={'height_ratios': [0.22, 1, 1, 1, 1]})
    strip, axes = all_axes[0], all_axes[1:]
    panels = [
        ('LiDAR position error (xy) [m]', lambda r: (r['t'], r['err'][:, 0])),
        ('LiDAR yaw error [deg]', lambda r: (r['t'], r['err'][:, 2])),
        ('LiDAR z error [m]', lambda r: (r['t'], r['err'][:, 1])),
        ('NDT angle_diff (/localization_debug data[4]) [deg]', lambda r: (r['ang_t'], r['ang'])),
    ]
    ph = runs[0][1]['phases']
    t_end = max(r['phases']['stop'] for _, r in runs) + 5.0
    bands = [('Left turn  +1 rad/s', ph['left turn'], ph['right turn']),
             ('Right turn  -1 rad/s', ph['right turn'], ph['forward']),
             ('Forward 1.5 m/s', ph['forward'], ph['stop'])]
    for ax, (title, get) in zip(axes, panels):
        style(ax, box=False, bands=bands)
        for i, ((label, r), color, lw) in enumerate(zip(runs, colors, widths_back_to_front(len(runs)))):
            x, y = get(r)
            m = (x >= -3.0) & (x <= t_end)
            ax.plot(x[m], y[m], color=color, lw=lw, label=label, zorder=3 + i)
        if 'yaw' in title or 'z error' in title:
            ax.axhline(0, color=INK2, lw=0.8, zorder=2)
        ax.set_title(title)
    # 走行の区間（指令）を上の帯に書く
    strip.set_facecolor(SURFACE)
    strip.set_yticks([])
    for spine in strip.spines.values():
        spine.set_visible(False)
    strip.tick_params(axis='x', length=0)
    for i, (name, a, b) in enumerate(bands):
        strip.axvspan(a, b, color=BAND if i % 2 == 0 else BAND_ALT, lw=0)
        strip.text((a + b) / 2, 0.5, name, ha='center', va='center', fontsize=9, color=INK2,
                   transform=strip.get_xaxis_transform())
    strip.text(-2.9, 0.5, 'Command', ha='left', va='center', fontsize=9, color=INK2,
               transform=strip.get_xaxis_transform())
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left', bbox_to_anchor=(0.055, 0.965), ncol=2, frameon=False)
    axes[-1].set_xlabel('Time since start of left turn [s] (sim time)')
    axes[-1].set_xlim(-3.0, t_end)
    fig.suptitle('Gazebo: left 1 turn -> right 1 turn -> forward 3 s  (LiDAR pose vs truth)',
                 x=0.06, ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(out, dpi=130, facecolor=SURFACE)


def main():
    out = sys.argv[1]
    runs = []
    for arg in sys.argv[2:]:
        label, path = arg.split('=', 1)
        path, _, topic = path.partition('@')
        runs.append((label, load_run(path, topic or '/ekf_pose')))
    for label, r in runs:
        summarize(label, r)
    plot(out, runs)
    print(f'saved {out}')


if __name__ == '__main__':
    main()
