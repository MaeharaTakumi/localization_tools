#!/usr/bin/env python3
"""RViz の点群の表示に使う姿勢の違い：同じ点群の時刻の、EKF の予測値（TF）と NDT の解（/ndt_pose）の差を描く.

  python3 plot_prior_posterior.py <bag> <出力 png>

録画は ekf_localizer を publish_trigger: cloud で動かしたもの（TF map -> base は点群の stamp の時刻の予測値）。
RViz の /velodyne_points はこの TF で、/aligned_points（pcl_localization）は NDT の解で map に直される。
LiDAR の姿勢は TF map -> base に /tf_static の取付 base -> LiDAR を掛けたもの。差は NDT − 予測
（水平の位置 [mm]・yaw [deg]）と、LiDAR から 10 m の点が表示でずれる距離 [mm]（水平の 8 方向の最大）。
横軸は最初の動作（<bag>_phases.txt）からの時間 [s]。
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
from pose_math import pose_msg_to_matrix  # noqa: E402

C_XY, C_YAW, C_PT = '#2a78d6', '#eb6834', '#1baf7a'
RANGE = 10.0   # 表示のずれを見る点の距離 [m]


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


def differences(bag):
    """点群の stamp ごとの [時刻, 水平の位置の差 [m], yaw の差 [rad], 10 m の点のずれ [m]]."""
    d = read_bag(bag, ['/tf', '/tf_static', '/ndt_pose'])
    ndt = {ns(m.header.stamp): pose_msg_to_matrix(m.pose.pose) for _, m in d['/ndt_pose']}
    mount, base, lidar = None, None, None
    # 取付 base -> LiDAR（実機の環境ファイルの static_tfs。imu_link などは使わない）
    for _, m in d.get('/tf_static', []):
        for tr in m.transforms:
            if mount is None and tr.child_frame_id == 'velodyne':
                base, lidar, mount = tr.header.frame_id, tr.child_frame_id, tf_matrix(tr)
    if mount is None:
        raise RuntimeError(f'no mount (base -> velodyne) in /tf_static of {bag}')
    pts = np.array([[RANGE * np.cos(a), RANGE * np.sin(a), 0.0, 1.0]
                    for a in np.linspace(0, 2 * np.pi, 8, endpoint=False)]).T
    rows = []
    for _, m in d['/tf']:
        for tr in m.transforms:
            if tr.header.frame_id != 'map' or tr.child_frame_id != base:
                continue
            k = ns(tr.header.stamp)
            if k not in ndt:
                continue
            prior = tf_matrix(tr) @ mount
            post = ndt[k]
            D = np.linalg.inv(prior) @ post
            shift = np.linalg.norm((post @ pts - prior @ pts)[:3], axis=0).max()
            rows.append([k * 1e-9, np.hypot(*(post[:2, 3] - prior[:2, 3])),
                         wrap(np.arctan2(D[1, 0], D[0, 0])), shift])
    return np.array(rows), lidar


def plot(bag, out):
    rows, lidar = differences(bag)
    ph = phases_of(bag)
    t0 = min(ph.values()) if ph else rows[0, 0]   # 動作の時刻が無い録画（途中で止めたもの）は最初の点群から
    t = rows[:, 0] - t0
    setup()
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, facecolor=SURFACE)
    lw = widths_back_to_front(1)[0]
    series = [(rows[:, 1] * 1e3, C_XY, 'Horizontal position difference [mm]'),
              (np.degrees(rows[:, 2]), C_YAW, 'Yaw difference [deg]'),
              (rows[:, 3] * 1e3, C_PT, f'Shift of a point {RANGE:.0f} m away [mm]')]
    for ax, (y, c, label) in zip(axes, series):
        style(ax)
        for name, ts in ph.items():
            ax.axvline(ts - t0, color=INK2, lw=0.6, alpha=0.4)
        ax.plot(t, y, color=c, lw=lw)
        ax.set_ylabel(label)
        a = np.abs(y)
        ax.set_title(f'mean |·| {a.mean():.2f}, 95 % {np.percentile(a, 95):.2f}, max {a.max():.2f}', loc='right',
                     fontsize=9, fontweight='normal', color=INK2)
        box_on(ax)
    fig.suptitle(f'NDT pose (/aligned_points) − EKF prediction (TF, /velodyne_points) at the cloud stamp '
                      f'({lidar}, {len(rows)} clouds)', x=0.01, ha='left', fontsize=10, fontweight='bold')
    axes[-1].set_xlabel(f'Time from "{min(ph, key=ph.get)}" [s]' if ph else 'Time from the first cloud [s]')
    capitalize_labels(fig)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f'saved {out}: {len(rows)} clouds, xy mean {rows[:, 1].mean() * 1e3:.1f} mm, '
          f'yaw mean {np.degrees(np.abs(rows[:, 2])).mean():.3f} deg, max {np.degrees(np.abs(rows[:, 2])).max():.2f} deg, '
          f'{RANGE:.0f} m point mean {rows[:, 3].mean() * 1e3:.1f} mm, max {rows[:, 3].max() * 1e3:.0f} mm')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    plot(sys.argv[1], sys.argv[2])
