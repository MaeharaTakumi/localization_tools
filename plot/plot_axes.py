#!/usr/bin/env python3
"""同じ録画を再生した各推定器の出力について、6 軸と処理時間の推移を描く（localizer_rl 内で実行）.

  python3 plot_axes.py <出力の接頭辞> <ラベル>=<bag>@<姿勢のトピック> ...
  出力: <接頭辞>_position.png（x, y, z。LiDAR と車体）、<接頭辞>_orientation.png（roll, pitch, yaw）、
        <接頭辞>_timing.png（NDT の位置合わせ時間、点群の時刻から /ndt_pose・出力が届くまでの遅れ）

- 姿勢のトピックは map -> LiDAR の PoseWithCovarianceStamped（/ekf_pose、NDT 単体なら /ndt_pose）。
  車体（base_footprint）は URDF の取付で直したもの。Gazebo の取付は回転 0 なので角度は LiDAR と車体で同じ
- 真値は /wheelchair/pose（base_footprint、header なし。bag の受信時刻を /clock で sim time に直す）。最初の bag のものを使う
- 遅れ = 受信時刻（sim time に直したもの）- stamp。ekf_localizer（publish_trigger: timer）の /ekf_pose は stamp が配信時刻
- 横軸は「左回転」の開始からの sim time [s]（<bag>_phases.txt）
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import capitalize_labels, BAND, GRID, INK, INK2, read_bag, SURFACE, URDF_MOUNT  # noqa: E402
from plot_spin import TRUTH_COLOR, TRUTH_LW, widths_back_to_front  # noqa: E402
from robot_localization_adapter import matrix_to_vec, pose_msg_to_matrix  # noqa: E402

COLORS = os.environ.get('PLOT_COLORS', '#2a78d6,#1baf7a,#eda100').split(',')
MOUNT_INV = np.linalg.inv(URDF_MOUNT)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def phases_of(bag):
    out = {}
    with open(bag.rstrip('/') + '_phases.txt') as f:
        for line in f:
            name, t = line.rsplit(' ', 1)
            out[name] = float(t)
    return out


def unwrap_to(yaw, ref_t, ref_yaw, t):
    """yaw を連続にし、最初の値を真値と同じ周回にそろえる."""
    y = np.unwrap(yaw)
    if len(y):
        r = np.interp(t[0], ref_t, ref_yaw)
        y += 2 * np.pi * np.round((r - y[0]) / (2 * np.pi))
    return y


def load(path, topic):
    d = read_bag(path)
    clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d['/clock']])

    def to_sim(ts):
        return np.interp(ts, clk[:, 0], clk[:, 1])

    run = {'phases': phases_of(path)}
    tt = to_sim([t for t, _ in d['/wheelchair/pose']])
    base = np.array([pose_msg_to_matrix(m) for _, m in d['/wheelchair/pose']])
    o = np.argsort(tt)
    run['truth_t'], base = tt[o], base[o]
    run['truth_base'] = np.array([matrix_to_vec(T) for T in base])
    run['truth_lidar'] = np.array([matrix_to_vec(T @ URDF_MOUNT) for T in base])
    run['truth_base'][:, 5] = np.unwrap(run['truth_base'][:, 5])
    run['truth_lidar'][:, 5] = np.unwrap(run['truth_lidar'][:, 5])

    msgs = d.get(topic, [])
    run['t'] = np.array([stamp(m) for _, m in msgs])
    lidar = [pose_msg_to_matrix(m.pose.pose) for _, m in msgs]
    run['lidar'] = np.array([matrix_to_vec(T) for T in lidar])
    run['base'] = np.array([matrix_to_vec(T @ MOUNT_INV) for T in lidar])
    for key in ('lidar', 'base'):
        run[key][:, 5] = unwrap_to(run[key][:, 5], run['truth_t'], run['truth_' + key][:, 5], run['t'])
    run['out_latency'] = (to_sim([t for t, _ in msgs]) - run['t']) * 1000.0
    ndt = d.get('/ndt_pose', [])
    run['ndt_t'] = np.array([stamp(m) for _, m in ndt])
    run['ndt_latency'] = (to_sim([t for t, _ in ndt]) - run['ndt_t']) * 1000.0
    dbg = d.get('/localization_debug', [])
    run['align_t'] = to_sim([t for t, _ in dbg])
    run['align_ms'] = np.array([m.data[1] for _, m in dbg]) * 1000.0
    return run


def style_axis(ax, bands):
    ax.set_facecolor(SURFACE)
    for i, (_, a, b) in enumerate(bands):
        ax.axvspan(a, b, color=BAND if i % 2 == 0 else '#f6f5f2', lw=0, zorder=0)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for spine in ('top', 'right'):
        ax.spines[spine].set_visible(False)


def draw_strip(strip, bands, short=False):
    strip.set_facecolor(SURFACE)
    strip.set_yticks([])
    for spine in strip.spines.values():
        spine.set_visible(False)
    strip.tick_params(axis='x', length=0)
    for i, (name, a, b) in enumerate(bands):
        strip.axvspan(a, b, color=BAND if i % 2 == 0 else '#f6f5f2', lw=0)
        label = name.split('  ')[0] if short else name   # 2 列の図では指令の値を省く
        strip.text((a + b) / 2, 0.5, label, ha='center', va='center', fontsize=9, color=INK2,
                   transform=strip.get_xaxis_transform())


def new_figure(ncols, nrows, title, size):
    plt.rcParams.update({
        'font.size': 10, 'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'xtick.color': INK2,
        'ytick.color': INK2, 'text.color': INK, 'axes.titlecolor': INK, 'axes.titlesize': 10,
        'axes.titleweight': 'bold', 'axes.titlelocation': 'left'})
    fig, axes = plt.subplots(nrows + 1, ncols, figsize=size, sharex=True, facecolor=SURFACE,
                             gridspec_kw={'height_ratios': [0.22] + [1] * nrows}, squeeze=False)
    fig.suptitle(title, x=0.06, ha='left', fontsize=12, fontweight='bold', color=INK, y=0.995)
    return fig, axes


def finish(fig, axes, out, t_range, handles):
    for ax in axes[-1]:
        ax.set_xlabel('time since start of left turn [s] (sim time)')
        ax.set_xlim(*t_range)
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.055, 0.965),
               ncol=len(handles), frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    capitalize_labels(fig)   # 頭文字を大文字に（識別子はそのまま）
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    plt.close(fig)
    print(f'saved {out}')


def main():
    prefix = sys.argv[1]
    runs = []
    for arg in sys.argv[2:]:
        label, rest = arg.split('=', 1)
        path, _, topic = rest.partition('@')
        runs.append((label, load(path, topic or '/ekf_pose')))
    ref = runs[0][1]
    t0 = ref['phases']['left turn']
    ph = {k: v - t0 for k, v in ref['phases'].items()}
    bands = [('left turn  +1 rad/s', ph['left turn'], ph['right turn']),
             ('right turn  -1 rad/s', ph['right turn'], ph['forward']),
             ('forward  1.5 m/s', ph['forward'], ph['stop'])]
    t_range = (-3.0, ph['stop'] + 5.0)

    def window(t):
        return (t - t0 >= t_range[0]) & (t - t0 <= t_range[1])

    # 後方ほど太く：真値（一番後ろ）→ 各推定器（渡した順に細く）
    widths = widths_back_to_front(len(runs), widest=2.4, thinnest=1.1)
    truth_handle = plt.Line2D([], [], color=TRUTH_COLOR, lw=TRUTH_LW, label='truth')
    handles = [truth_handle] + [plt.Line2D([], [], color=c, lw=w, label=label)
                                for (label, _), c, w in zip(runs, COLORS, widths)]

    def plot_series(ax, key, i, scale=1.0):
        m = window(ref['truth_t'])
        ax.plot(ref['truth_t'][m] - t0, ref['truth_' + key][m, i] * scale, color=TRUTH_COLOR,
                lw=TRUTH_LW, zorder=2)
        for j, ((label, r), c, w) in enumerate(zip(runs, COLORS, widths)):
            m = window(r['t'])
            ax.plot(r['t'][m] - t0, r[key][m, i] * scale, color=c, lw=w, zorder=3 + j)

    # ---- 位置（LiDAR と車体）----
    fig, axes = new_figure(2, 3, 'Position: truth vs estimates (same recording replayed)', (13, 9.5))
    for col, (key, name) in enumerate((('lidar', 'LiDAR'), ('base', 'base_footprint'))):
        draw_strip(axes[0][col], bands, short=True)
        for row, axis in enumerate('xyz'):
            ax = axes[row + 1][col]
            style_axis(ax, bands)
            plot_series(ax, key, row)
            ax.set_title(f'{axis} ({name}) [m]')
    finish(fig, axes, f'{prefix}_position.png', t_range, handles)

    # ---- 角度（LiDAR と車体で同じ）----
    fig, axes = new_figure(1, 3, 'Orientation: truth vs estimates (LiDAR = base_footprint in Gazebo)', (10, 9))
    draw_strip(axes[0][0], bands)
    for row, axis in enumerate(('roll', 'pitch', 'yaw')):
        ax = axes[row + 1][0]
        style_axis(ax, bands)
        plot_series(ax, 'lidar', row + 3, 180.0 / math.pi)
        ax.set_title(f'{axis} [deg]' + ('  (unwrapped)' if axis == 'yaw' else ''))
    finish(fig, axes, f'{prefix}_orientation.png', t_range, handles)

    # ---- 処理時間 ----
    fig, axes = new_figure(1, 3, 'Processing time and latency', (10, 9))
    draw_strip(axes[0][0], bands)
    panels = [
        ('NDT alignment time (/localization_debug data[1]) [ms]', 'align_t', 'align_ms'),
        ('latency: point cloud stamp -> /ndt_pose received [ms]', 'ndt_t', 'ndt_latency'),
        ('latency: stamp -> output received [ms]  (/ekf_pose; NDT only: /ndt_pose)', 't', 'out_latency'),
    ]
    print(f"{'':<28} {'mean':>7} {'p95':>7} {'max':>7}  [ms, during motion]")
    for row, (title, tk, vk) in enumerate(panels):
        ax = axes[row + 1][0]
        style_axis(ax, bands)
        for j, ((label, r), c, w) in enumerate(zip(runs, COLORS, widths)):
            m = window(r[tk])
            ax.plot(r[tk][m] - t0, r[vk][m], color=c, lw=w, zorder=3 + j)
            mm = (r[tk] - t0 >= 0) & (r[tk] - t0 <= ph['stop'] + 2.0)
            v = r[vk][mm]
            if len(v):
                print(f"{label[:16] + ' ' + vk:<28} {v.mean():7.1f} {np.percentile(v, 95):7.1f} {v.max():7.1f}")
        ax.set_title(title)
        ax.set_ylim(bottom=0)
    finish(fig, axes, f'{prefix}_timing.png', t_range, handles[1:])


if __name__ == '__main__':
    main()
