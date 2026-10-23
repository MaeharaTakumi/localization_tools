#!/usr/bin/env python3
"""MATLAB 用の出力（ekf_localizer の /localization/base_pose・vlp16_pose・vlp16_points）を録画で確かめ、図を描く.

  python3 plot_matlab_outputs.py <出力を録った bag> <出力 png> [<入力の bag（点群を比べる）>]

確かめること（結果は標準出力）：
  base_pose    stamp の間隔（base_pose_rate の周期か）と、stamp から受信までの時間
  vlp16_pose   /ndt_pose と同じ stamp（点群の取得時刻）か、/tf_ndt_update（map -> base）に /tf_static の取付を
               掛けた値と一致するか、stamp から受信までの時間
  vlp16_points 同じ stamp の vlp16_pose があるか、姿勢より後に届いたか、入力の bag の同じ stamp の
               /velodyne_points と中身が同じか
受信の時刻は bag の時刻（壁時計）。stamp は /clock で壁時計に直す（/clock が無ければ stamp を壁時計とみなす）。
図：(1) stamp から受信までの時間の分布、(2)(3) 同じ点群の時刻の base_pose（予測。前後の値を補間）と、
NDT で更新した直後の base（/tf_ndt_update）の差（水平の位置・yaw）。
"""
import hashlib
import os
import struct
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
import rosbag2_py  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'calibration')]   # plot_spin などを import する
from plot_noise import box_on, setup, style  # noqa: E402
from plot_spin import capitalize_labels, INK2, read_bag, SERIES, SURFACE, widths_back_to_front, wrap  # noqa: E402
from plot_velocity import phases_of  # noqa: E402
from pose_math import matrix_to_rpy, pose_msg_to_matrix, transform_msg_to_matrix  # noqa: E402

BASE, VLP16, POINTS = '/localization/base_pose', '/localization/vlp16_pose', '/localization/vlp16_points'


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def cloud_digest(data):
    """点群の中身（header・形・フィールド・data）の md5。シリアライズしたバイト列は、配信した DDS によって
    末尾の詰め物の長さが違う（実機のラズパイの点群は 3 バイト長い）ので、読み込んでから比べる."""
    m = deserialize_message(data, PointCloud2)
    h = hashlib.md5(repr((m.header, m.height, m.width, m.fields, m.is_bigendian, m.point_step, m.row_step,
                          m.is_dense)).encode())
    h.update(bytes(m.data))
    return h.hexdigest()


def raw_clouds(bag, topic, keys=None, digest=False):
    """{stamp [ns]: (受信時刻 [s], 中身の md5（digest のときだけ）)}。CDR の先頭 4 バイトの後の header.stamp だけ読む."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = {}
    while reader.has_next():
        _, data, t = reader.read_next()
        sec, nsec = struct.unpack_from('<iI', data, 4)
        k = sec * 10**9 + nsec
        if keys is None or k in keys:
            out[k] = (t * 1e-9, cloud_digest(data) if digest else None)
    return out


def mount_of(static, base, lidar):
    """/tf_static をたどった base -> lidar（Gazebo は base_footprint -> base_link -> velodyne_base_link -> velodyne）."""
    parent = {tr.child_frame_id: (tr.header.frame_id, transform_msg_to_matrix(tr.transform))
              for _, m in static for tr in m.transforms}
    T, frame = np.eye(4), lidar
    while frame != base:
        if frame not in parent:
            raise RuntimeError(f'no TF from {base} to {lidar} in /tf_static')
        frame, M = parent[frame]
        T = M @ T
    return T


def stats(name, v, unit='ms'):
    v = np.asarray(v)
    if not len(v):
        print(f'  {name}: none')
        return
    print(f'  {name}: n={len(v)} min {v.min():.1f}  mean {v.mean():.1f}  50% {np.percentile(v, 50):.1f}  '
          f'95% {np.percentile(v, 95):.1f}  max {v.max():.1f} {unit}')


def check(bag, in_bag=None):
    d = read_bag(bag, ['/clock', '/tf_static', '/tf_ndt_update', '/ndt_pose', BASE, VLP16])
    for topic in (BASE, VLP16):
        if not d.get(topic):
            raise RuntimeError(f'no {topic} in {bag}')
    clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d.get('/clock', [])])

    def to_wall(k):
        s = np.asarray(k) * 1e-9
        return np.interp(s, clk[:, 1], clk[:, 0]) if len(clk) else s

    print(f'{bag}')
    # base_pose
    base_k = np.array([ns(m.header.stamp) for _, m in d[BASE]])
    base_recv = np.array([t for t, _ in d[BASE]])
    period = np.diff(base_k) * 1e-6
    print(f'base_pose: {len(base_k)} msgs, {(len(base_k) - 1) / ((base_k[-1] - base_k[0]) * 1e-9):.1f} Hz')
    stats('stamp interval', period)
    stats('stamp -> receipt', (base_recv - to_wall(base_k)) * 1e3)

    # vlp16_pose
    vlp = {ns(m.header.stamp): (t, pose_msg_to_matrix(m.pose.pose)) for t, m in d[VLP16]}
    ndt = {ns(m.header.stamp) for _, m in d.get('/ndt_pose', [])}
    print(f'vlp16_pose: {len(vlp)} msgs, /ndt_pose {len(ndt)}, same stamp as /ndt_pose: {len(set(vlp) & ndt)}')
    stats('cloud stamp -> receipt', [(t - to_wall(k)) * 1e3 for k, (t, _) in vlp.items()])
    update = {ns(tr.header.stamp): (tr.child_frame_id, transform_msg_to_matrix(tr.transform))
              for _, m in d.get('/tf_ndt_update', []) for tr in m.transforms if tr.header.frame_id == 'map'}
    base_update = {}
    if update:
        base_frame = next(iter(update.values()))[0]
        lidar_frame = 'velodyne'   # 環境ファイルの lidar_frame_id（vlp16_pose のメッセージには書かれない）
        mount = mount_of(d['/tf_static'], base_frame, lidar_frame)
        dp, da = [], []
        for k, (_, T) in vlp.items():
            if k in update:
                D = np.linalg.inv(update[k][1] @ mount) @ T
                dp.append(np.linalg.norm(D[:3, 3]) * 1e3)
                da.append(np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1))))
        print(f'  vs /tf_ndt_update x mount ({base_frame} -> {lidar_frame}): {len(dp)} stamps, '
              f'position max {max(dp):.4f} mm, rotation max {max(da):.5f} deg')
        base_update = {k: T for k, (_, T) in update.items()}

    # vlp16_points
    pts = raw_clouds(bag, POINTS, digest=bool(in_bag))
    if pts:
        paired = [k for k in pts if k in vlp]
        after = [(pts[k][0] - vlp[k][0]) * 1e3 for k in paired]
        print(f'vlp16_points: {len(pts)} msgs, with a vlp16_pose of the same stamp: {len(paired)}, '
              f'received after the pose: {sum(a >= 0 for a in after)}, '
              f'vlp16_pose without vlp16_points: {len(set(vlp) - set(pts))}')
        stats('pose -> points receipt', after)
        if in_bag:
            src = raw_clouds(in_bag, '/velodyne_points', set(pts), digest=True)
            same = sum(src[k][1] == pts[k][1] for k in pts if k in src)
            print(f'  same content as /velodyne_points of {in_bag}: {same} / {len(pts)} (found {len(src)})')
    else:
        print('vlp16_points: not recorded')

    # 同じ点群の時刻の base_pose（予測）と NDT で更新した直後の base の差
    bx = np.array([pose_msg_to_matrix(m.pose.pose) for _, m in d[BASE]])
    byaw = np.unwrap([np.arctan2(T[1, 0], T[0, 0]) for T in bx])
    rows = []
    for k in sorted(base_update):
        if not base_k[0] <= k <= base_k[-1]:
            continue
        x = np.interp(k, base_k, bx[:, 0, 3])
        y = np.interp(k, base_k, bx[:, 1, 3])
        yaw = np.interp(k, base_k, byaw)
        U = base_update[k]
        rows.append([k * 1e-9, np.hypot(x - U[0, 3], y - U[1, 3]),
                     wrap(yaw - np.arctan2(U[1, 0], U[0, 0]))])
    rows = np.array(rows)
    if len(rows):
        stats('base_pose - NDT update (horizontal)', rows[:, 1] * 1e3, 'mm')
        stats('base_pose - NDT update (|yaw|)', np.degrees(np.abs(rows[:, 2])), 'deg')
    return {
        'base_delay': (base_recv - to_wall(base_k)) * 1e3,
        'vlp_delay': np.array([(t - to_wall(k)) * 1e3 for k, (t, _) in vlp.items()]),
        'diff': rows,
    }


def plot(bag, out, r):
    try:
        ph = phases_of(bag)
    except FileNotFoundError:
        ph = {}
    rows = r['diff']
    t0 = min(ph.values()) if ph else rows[0, 0]
    setup()
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), facecolor=SURFACE,
                             gridspec_kw={'height_ratios': [1.0, 1.0, 1.0]})
    ax = axes[0]
    style(ax)
    top = max(np.percentile(r['vlp_delay'], 99.5), 20.0)
    bins = np.arange(0.0, top + 2.0, 2.0)
    for (label, v), color in zip([('Base pose (prediction, base_pose_rate)', r['base_delay']),
                                  ('LiDAR pose (NDT update, at the cloud stamp)', r['vlp_delay'])], SERIES):
        ax.hist(np.clip(v, 0, bins[-1]), bins=bins, weights=np.full(len(v), 1.0 / len(v)), color=color,
                edgecolor='black', lw=0.5, alpha=0.85, label=f'{label}: mean {v.mean():.1f} ms')
    ax.set_xlabel('Time from the stamp to the receipt [ms]')
    ax.set_ylabel('Relative frequency')
    ax.legend(loc='upper right', frameon=False, fontsize=9)
    ax.set_title('When /localization/base_pose and /localization/vlp16_pose arrive')
    box_on(ax)
    lw = widths_back_to_front(1)[0]
    for ax, (i, scale, ylabel) in zip(axes[1:], [(1, 1e3, 'Horizontal position difference [mm]'),
                                                 (2, 180 / np.pi, 'Yaw difference [deg]')]):
        style(ax)
        for ts in ph.values():
            ax.axvline(ts - t0, color=INK2, lw=0.6, alpha=0.4)
        ax.plot(rows[:, 0] - t0, rows[:, i] * scale, color=SERIES[0], lw=lw,
                label='Base pose (prediction) − EKF NDT update (base)')
        ax.set_ylabel(ylabel)
        ax.set_xlabel(f'Time from "{min(ph, key=ph.get)}" [s]' if ph else 'Time from the first cloud [s]')
        box_on(ax)
    axes[1].legend(loc='upper left', frameon=False, fontsize=9)
    axes[1].set_title('Base pose at the cloud stamp: prediction vs NDT update')
    capitalize_labels(fig)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f'saved {out}')


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    bag, out = sys.argv[1], sys.argv[2]
    r = check(bag, sys.argv[3] if len(sys.argv) > 3 else None)
    plot(bag, out, r)


if __name__ == '__main__':
    main()
