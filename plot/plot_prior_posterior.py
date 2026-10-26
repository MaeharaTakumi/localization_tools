#!/usr/bin/env python3
"""RViz の点群の表示に使う姿勢の違い：同じ点群の時刻の、EKF の予測値・EKF の NDT の更新値・NDT の解の差を描く.

  python3 plot_prior_posterior.py <bag> <出力 png>

どれも点群の stamp の時刻の map -> LiDAR：
  予測値      /tf（map -> base を点群の stamp で補間。NDT の初期値。2026-10-26 から 50 Hz の周期ごとの予測値、
              それまでの publish_trigger: cloud の録画は点群の stamp ちょうどの予測値）
  NDT の更新値 /tf_ndt_update（map -> base。その点群の NDT で更新した直後の EKF の値。publish_ndt_update_tf。
              RViz は /tf をこれに付け替えて点群を map に直す。録画に無ければ描かない）
  NDT の解    /ndt_pose
base の姿勢には /tf_static の取付 base -> LiDAR を掛ける。差は水平の位置 [mm]・yaw [deg] と、
LiDAR から 10 m の点が表示でずれる距離 [mm]（水平の 8 方向の最大）。
横軸は最初の動作（<bag>_phases.txt。無ければ最初の点群）からの時間 [s]。
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'calibration')]   # plot_spin などを import する
from plot_noise import box_on, setup, style  # noqa: E402
from plot_spin import capitalize_labels, INK2, read_bag, SURFACE, widths_back_to_front, wrap  # noqa: E402
from plot_velocity import phases_of  # noqa: E402
from plot_matlab_outputs import TfInterp  # noqa: E402
from pose_math import pose_msg_to_matrix  # noqa: E402

RANGE = 10.0   # 表示のずれを見る点の距離 [m]
# 比べる組（後 − 前）と色。録画にある組だけ描く（先に描くものほど太い）
PAIRS = [('ndt', 'prior', 'NDT − EKF prediction', '#2a78d6'),
         ('update', 'prior', 'EKF NDT update (RViz) − EKF prediction', '#eb6834'),
         ('ndt', 'update', 'NDT − EKF NDT update (RViz)', '#1baf7a')]
PANELS = [('xy', 1e3, 'Horizontal position difference [mm]'),
          ('yaw', 180 / np.pi, 'Yaw difference [deg]'),
          ('shift', 1e3, f'Shift of a point {RANGE:.0f} m away [mm]')]


def tf_matrix(tr):
    t, q = tr.transform.translation, tr.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def lidar_poses(bag):
    """{'prior' / 'update' / 'ndt': {点群の stamp [ns]: map -> LiDAR}} と LiDAR のフレーム名."""
    d = read_bag(bag, ['/tf', '/tf_static', '/tf_ndt_update', '/ndt_pose'])
    mount, base, lidar = None, None, None
    # 取付 base -> LiDAR（実機の環境ファイルの static_tfs。imu_link などは使わない）
    for _, m in d.get('/tf_static', []):
        for tr in m.transforms:
            if mount is None and tr.child_frame_id == 'velodyne':
                base, lidar, mount = tr.header.frame_id, tr.child_frame_id, tf_matrix(tr)
    if mount is None:
        raise RuntimeError(f'no mount (base -> velodyne) in /tf_static of {bag}')

    def from_tf(topic):
        return {ns(tr.header.stamp): tf_matrix(tr) @ mount for _, m in d.get(topic, []) for tr in m.transforms
                if tr.header.frame_id == 'map' and tr.child_frame_id == base}

    ndt = {ns(m.header.stamp): pose_msg_to_matrix(m.pose.pose) for _, m in d['/ndt_pose']}
    # /tf は点群の stamp の時刻で引く（tf2 と同じ補間。2026-10-26 までの publish_trigger: cloud の録画は stamp ちょうどの値がある）
    interp = TfInterp([(ns(tr.header.stamp), t, tf_matrix(tr)) for t, m in d['/tf'] for tr in m.transforms
                       if tr.header.frame_id == 'map' and tr.child_frame_id == base])
    prior = {k: P @ mount for k in ndt for P in [interp.at(k)] if P is not None}
    poses = {'prior': prior, 'update': from_tf('/tf_ndt_update'), 'ndt': ndt}
    return poses, lidar


def differences(poses, after, before):
    """両方ある点群の stamp ごとの [時刻 [s], 水平の位置の差 [m], yaw の差 [rad], 10 m の点のずれ [m]]（after − before）."""
    pts = np.array([[RANGE * np.cos(a), RANGE * np.sin(a), 0.0, 1.0]
                    for a in np.linspace(0, 2 * np.pi, 8, endpoint=False)]).T
    rows = []
    for k in sorted(set(poses[after]) & set(poses[before])):
        A, B = poses[after][k], poses[before][k]
        D = np.linalg.inv(B) @ A
        rows.append([k * 1e-9, np.hypot(*(A[:2, 3] - B[:2, 3])), wrap(np.arctan2(D[1, 0], D[0, 0])),
                     np.linalg.norm((A @ pts - B @ pts)[:3], axis=0).max()])
    return np.array(rows)


def plot(bag, out):
    poses, lidar = lidar_poses(bag)
    series = [(label, color, differences(poses, a, b)) for a, b, label, color in PAIRS if poses[a] and poses[b]]
    try:
        ph = phases_of(bag)
    except FileNotFoundError:
        ph = {}
    t0 = min(ph.values()) if ph else min(r[0, 0] for _, _, r in series)   # 動作の時刻が無ければ最初の点群から
    setup()
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, facecolor=SURFACE)
    widths = widths_back_to_front(len(series))
    for ax, (col, scale, ylabel) in zip(axes, PANELS):
        style(ax)
        for ts in ph.values():
            ax.axvline(ts - t0, color=INK2, lw=0.6, alpha=0.4)
        for (label, color, rows), lw in zip(series, widths):
            i = [p[0] for p in PANELS].index(col) + 1
            ax.plot(rows[:, 0] - t0, rows[:, i] * scale, color=color, lw=lw, label=label)
        ax.set_ylabel(ylabel)
        box_on(ax)
    axes[0].legend(loc='upper left', frameon=False, fontsize=9)
    fig.suptitle(f'Pose at the cloud stamp used to place the point cloud ({lidar})', x=0.01, ha='left',
                 fontsize=10, fontweight='bold')
    axes[-1].set_xlabel(f'Time from "{min(ph, key=ph.get)}" [s]' if ph else 'Time from the first cloud [s]')
    capitalize_labels(fig)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f'saved {out}')
    for label, _, rows in series:
        xy, yaw, sh = rows[:, 1] * 1e3, np.degrees(np.abs(rows[:, 2])), rows[:, 3] * 1e3
        print(f'  {label}: {len(rows)} clouds, xy mean {xy.mean():.1f} max {xy.max():.0f} mm, '
              f'yaw mean {yaw.mean():.3f} max {yaw.max():.2f} deg, {RANGE:.0f} m point mean {sh.mean():.1f} '
              f'p95 {np.percentile(sh, 95):.0f} max {sh.max():.0f} mm')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    plot(sys.argv[1], sys.argv[2])
