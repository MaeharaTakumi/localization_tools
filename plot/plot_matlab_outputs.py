#!/usr/bin/env python3
"""MATLAB が /tf から引く値（後輪間中心の最新の姿勢と、点群の stamp の時刻の LiDAR の姿勢）を録画で確かめ、図を描く.

  python3 plot_matlab_outputs.py <出力を録った bag> <出力 png> [--input <入力 bag>]                     1 回分の確かめと図
  python3 plot_matlab_outputs.py compare <出力 png> <ラベル>=<bag>[,<bag>...] ... [--input <入力 bag>]  ラベルごとの平均と幅、分布の図

出力の bag は eval/replay_outputs.py で録ったもの（/tf・/tf_static・/ndt_pose・/clock。Gazebo は /wheelchair/pose も）。
確かめること（結果は標準出力。最後の行は METRICS <1 行 JSON>）：
  tf          map -> base の /tf の stamp の間隔（predict_rate の周期か）と、stamp から受信までの時間
  tf_wait     点群の stamp（/ndt_pose の stamp）より後の /tf が届くまでの時間（点群の stamp から）。NDT と MATLAB が
              点群の stamp の時刻の TF を引けるのはこれより後（点群は stamp から約 15 ms 後に届く）
  prior       点群の stamp の時刻の /tf（tf2 と同じく前後の 2 つを補間。位置は線形、回転は slerp）に取付を掛けた
              map -> LiDAR。MATLAB が点群の座標変換に使う値で、NDT の初期値と同じ。
              ekf_localizer の tf_future_steps >= 1 では同じ stamp の /tf が何度も届いて置き換わる。--input（再生した入力の
              録画）を付けると、点群が届いた時刻（入力の録画の stamp から受信までの時間で求める。TF がまだ無ければ揃った時刻）に
              購読側が持っていた値を使う。付けなければ stamp ごとに最後に届いた値
  tf_wait_receipt  --input のとき、点群が届いてから TF が揃うまでの時間（負なら届いた時点で揃っている）。waited はその割合
  prior_ndt   prior − NDT の解（/ndt_pose）。NDT が初期値からどれだけ動いたか
  prior_post  prior − NDT で更新した直後の値（/tf_ndt_update に取付）。2026-10-26 まで MATLAB が使っていた vlp16_pose との差。
              /tf_ndt_update は 2026-10-26 にやめたので、それより前の録画（dev_log.md の 68 の再生など）だけ
  Gazebo だけ prior_truth・post_truth（真値の LiDAR との差。真値は /wheelchair/pose に URDF の取付）
差は水平の位置 [mm] と |yaw| [deg]。受信の時刻は bag の時刻（壁時計）。stamp は /clock で壁時計に直す。
図（1 回分）：(1) 点群の stamp から TF が届くまでの時間の分布、(2)(3) prior − NDT の解、prior − NDT で更新した直後の値の時間推移。
"""
import json
import os
import struct
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation, Slerp  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'calibration')]   # plot_spin などを import する
from plot_noise import box_on, setup, style  # noqa: E402
from plot_spin import capitalize_labels, INK2, read_bag, SERIES, SURFACE, widths_back_to_front  # noqa: E402
from plot_velocity import phases_of  # noqa: E402
from pose_math import pose_msg_to_matrix, transform_msg_to_matrix, vec_to_matrix  # noqa: E402

LIDAR_FRAME = 'velodyne'   # 環境ファイルの lidar_frame_id


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


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


def cloud_delays(in_bag):
    """入力の録画の点群の stamp [ns] → stamp から受信までの時間 [s]（Gazebo は /clock で stamp を壁時計に直す）."""
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=in_bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/velodyne_points', '/clock']))
    clk, clouds = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        sec, nsec = struct.unpack_from('<iI', data, 4)   # CDR の頭 4 バイトの次が header.stamp（/clock は clock）
        if topic == '/clock':
            clk.append((t * 1e-9, sec + nsec * 1e-9))
        else:
            clouds.append((sec * 10**9 + nsec, t * 1e-9))
    clk = np.array(clk)

    def to_wall(s):
        return np.interp(s, clk[:, 1], clk[:, 0]) if len(clk) else s
    return {k: t - to_wall(k * 1e-9) for k, t in clouds}


def diff(A, B):
    """A − B の水平の位置 [m] と yaw [rad]（map 座標の差）."""
    yaw = np.arctan2(A[1, 0], A[0, 0]) - np.arctan2(B[1, 0], B[0, 0])
    return np.hypot(A[0, 3] - B[0, 3], A[1, 3] - B[1, 3]), np.arctan2(np.sin(yaw), np.cos(yaw))


def stats(name, v, unit='ms'):
    v = np.asarray(v)
    if not len(v):
        print(f'  {name}: none')
        return {}
    print(f'  {name}: n={len(v)} min {v.min():.2f}  mean {v.mean():.2f}  50% {np.percentile(v, 50):.2f}  '
          f'95% {np.percentile(v, 95):.2f}  max {v.max():.2f} {unit}')
    return {'mean': float(v.mean()), 'p95': float(np.percentile(v, 95)), 'max': float(v.max())}


def interp_pose(k, k0, M0, k1, M1):
    """stamp k0・k1 の姿勢の間を tf2 と同じく補間する（位置は線形、回転は slerp）."""
    if k1 == k0:
        return M0.copy()
    T = np.eye(4)
    T[:3, 3] = M0[:3, 3] + (M1[:3, 3] - M0[:3, 3]) * (k - k0) / (k1 - k0)
    slerp = Slerp([0.0, 1.0], Rotation.from_matrix([M0[:3, :3], M1[:3, :3]]))
    T[:3, :3] = slerp([(k - k0) / (k1 - k0)]).as_matrix()[0]
    return T


class TfInterp:
    """map -> base の /tf を、tf2 と同じく前後の 2 つで補間する（その時刻より後の TF が無ければ None）.

    ekf_localizer の tf_future_steps >= 1 では同じ stamp の TF が何度も届き、tf2 では後から届いた値で置き換わる。
    at(k) は stamp ごとに最後に届いた値、at(k, recv) は受信時刻 recv の時点で購読側が持っていた値で補間する.
    """

    def __init__(self, items):
        versions = {}
        for k, t, M in sorted(items, key=lambda x: x[1]):
            versions.setdefault(k, []).append((t, M))
        self.k = np.array(sorted(versions))
        self.versions = [versions[k] for k in self.k]
        # stamp ごとの最初の受信時刻（以下の互換のため recv と呼ぶ）と最後の値
        self.recv = np.array([v[0][0] for v in self.versions])
        T = np.array([v[-1][1] for v in self.versions])
        self.p = T[:, :3, 3]
        self.slerp = Slerp(self.k * 1e-9, Rotation.from_matrix(T[:, :3, :3]))
        # stamp が k 以上の TF のどれかが最初に届いた時刻（後ろからの最小）
        self.first_at_or_after = np.minimum.accumulate(self.recv[::-1])[::-1]
        self.n_sent = np.array([len(v) for v in self.versions])

    def at(self, k, recv=None):
        if not self.k[0] <= k <= self.k[-1]:
            return None
        if recv is None:
            T = np.eye(4)
            T[:3, 3] = [np.interp(k, self.k, self.p[:, i]) for i in range(3)]
            T[:3, :3] = self.slerp([k * 1e-9]).as_matrix()[0]
            return T
        # recv の時点で届いていた、k 以前の最も新しい stamp と k 以上の最も古い stamp の、その時点の値
        i = int(np.searchsorted(self.k, k, side='right')) - 1
        j = int(np.searchsorted(self.k, k, side='left'))
        while i >= 0 and self.recv[i] > recv:
            i -= 1
        while j < len(self.k) and self.recv[j] > recv:
            j += 1
        if i < 0 or j >= len(self.k):
            return None

        def value(n):
            return [M for t, M in self.versions[n] if t <= recv][-1]
        return interp_pose(k, self.k[i], value(i), self.k[j], value(j))

    def available(self, k):
        """stamp k の TF を引けるようになる受信時刻 [s]（stamp が k 以上の TF のどれかが最初に届いた時刻）."""
        i = np.searchsorted(self.k, k)
        return self.first_at_or_after[i] if i < len(self.k) else None


def check(bag, in_bag=None):
    d = read_bag(bag, ['/clock', '/tf', '/tf_static', '/tf_ndt_update', '/ndt_pose'])
    clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d.get('/clock', [])])

    def to_wall(k):
        s = np.asarray(k) * 1e-9
        return np.interp(s, clk[:, 1], clk[:, 0]) if len(clk) else s

    tf = [(ns(tr.header.stamp), t, tr.child_frame_id, transform_msg_to_matrix(tr.transform))
          for t, m in d['/tf'] for tr in m.transforms if tr.header.frame_id == 'map']
    if not tf:
        raise RuntimeError(f'no /tf map -> base in {bag}')
    base_frame = tf[0][2]
    interp = TfInterp([(k, t, M) for k, t, _, M in tf])
    mount = mount_of(d['/tf_static'], base_frame, LIDAR_FRAME)
    print(f'{bag}')
    out = {'bag': bag}

    period = np.diff(interp.k) * 1e-6
    rate = (len(interp.k) - 1) / ((interp.k[-1] - interp.k[0]) * 1e-9)
    print(f'/tf map -> {base_frame}: {len(interp.k)} stamps, {rate:.1f} Hz, '
          f'each stamp sent {interp.n_sent.mean():.2f} times on average')
    out['tf_rate'] = rate
    out['tf_sent'] = float(interp.n_sent.mean())
    out['tf_interval'] = stats('stamp interval', period)
    out['tf_latency'] = stats('stamp -> receipt', (interp.recv - to_wall(interp.k)) * 1e3)

    ndt = {ns(m.header.stamp): (t, pose_msg_to_matrix(m.pose.pose)) for t, m in d.get('/ndt_pose', [])}
    post = {ns(tr.header.stamp): transform_msg_to_matrix(tr.transform)
            for _, m in d.get('/tf_ndt_update', []) for tr in m.transforms if tr.header.frame_id == 'map'}
    print(f'/ndt_pose: {len(ndt)}, /tf_ndt_update: {len(post)}')
    out['n_ndt'] = len(ndt)
    out['ndt_latency'] = stats('/ndt_pose: cloud stamp -> receipt', [(t - to_wall(k)) * 1e3 for k, (t, _) in ndt.items()])

    truth = None
    if '/wheelchair/pose' in read_bag(bag, ['/wheelchair/pose']):
        from compare_run import load, truth_lidar  # noqa: E402  Gazebo だけ
        tt, tv, *_ = load(bag)
        truth = (tt, tv, truth_lidar)

    delays = cloud_delays(in_bag) if in_bag else {}
    waits = []   # 点群が届いてから TF が揃うまで [ms]
    rows = []   # [stamp s, tf_wait ms, prior-ndt xy, yaw, prior-post xy, yaw, prior-truth xy, yaw, post-truth xy, yaw]
    for k in sorted(ndt):
        avail = interp.available(k)
        if avail is None:
            continue
        if delays:
            if k not in delays:
                continue
            look = to_wall(k) + delays[k]   # 点群が届いた時刻
            waits.append((avail - look) * 1e3)
            P = interp.at(k, max(look, avail))
        else:
            P = interp.at(k)
        if P is None:
            continue
        prior = P @ mount
        row = [k * 1e-9, (avail - to_wall(k)) * 1e3, *diff(prior, ndt[k][1])]
        row += [*diff(prior, post[k] @ mount)] if k in post else [np.nan, np.nan]
        if truth is not None:
            tt, tv, truth_lidar = truth
            Tl = vec_to_matrix(truth_lidar(tt, tv, [k * 1e-9])[0])
            row += [*diff(prior, Tl)]
            row += [*diff(post[k] @ mount, Tl)] if k in post else [np.nan, np.nan]
        rows.append(row)
    rows = np.array(rows)
    out['tf_wait'] = stats('cloud stamp -> the first /tf after it (receipt)', rows[:, 1])
    if waits:
        out['tf_wait_receipt'] = stats('cloud receipt -> the TF at the cloud stamp available', waits)
        out['waited'] = float(np.mean(np.array(waits) > 0))
        print(f'  waited for the TF: {out["waited"] * 100:.1f} % of {len(waits)} clouds')
    names = [('prior_ndt', 2), ('prior_post', 4)] + ([('prior_truth', 6), ('post_truth', 8)] if truth else [])
    for name, i in names:
        ok = ~np.isnan(rows[:, i])
        out[name] = {'xy': stats(f'{name} (horizontal)', rows[ok, i] * 1e3, 'mm'),
                     'yaw': stats(f'{name} (|yaw|)', np.degrees(np.abs(rows[ok, i + 1])), 'deg')}
    print('METRICS ' + json.dumps(out))
    return out, rows


def plot(bag, out, rows):
    try:
        ph = phases_of(bag)
    except FileNotFoundError:
        ph = {}
    t0 = min(ph.values()) if ph else rows[0, 0]
    setup()
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), facecolor=SURFACE)
    ax = axes[0]
    style(ax)
    v = rows[:, 1]
    bins = np.arange(np.floor(v.min()) - 1.0, max(np.percentile(v, 99.5), 20.0) + 1.0, 1.0)
    ax.hist(np.clip(v, bins[0], bins[-1]), bins=bins, weights=np.full(len(v), 1.0 / len(v)), color=SERIES[0],
            edgecolor='black', lw=0.5, label=f'Mean {v.mean():.1f} ms, max {v.max():.1f} ms')
    ax.set_xlabel('Time from the cloud stamp to the first /tf after it [ms]')
    ax.set_ylabel('Relative frequency')
    ax.legend(loc='upper right', frameon=False, fontsize=9)
    ax.set_title('When the TF at the cloud stamp becomes available')
    box_on(ax)
    lws = widths_back_to_front(2)
    for ax, (k, scale, ylabel) in zip(axes[1:], [(0, 1e3, 'Horizontal position difference [mm]'),
                                                 (1, 180 / np.pi, 'Yaw difference [deg]')]):
        style(ax)
        for ts in ph.values():
            ax.axvline(ts - t0, color=INK2, lw=0.6, alpha=0.4)
        for (i, label), color, lw in zip([(2, 'Prior (/tf at the cloud stamp) − NDT solution'),
                                          (4, 'Prior − EKF NDT update (/tf_ndt_update)')], SERIES, lws):
            y = rows[:, i + k] * scale
            ax.plot(rows[:, 0] - t0, y if k == 0 else np.abs(y), color=color, lw=lw, label=label)
        ax.set_ylabel(ylabel if k == 0 else '|Yaw difference| [deg]')
        ax.set_xlabel(f'Time from "{min(ph, key=ph.get)}" [s]' if ph else 'Time from the first cloud [s]')
        box_on(ax)
    axes[1].legend(loc='upper left', frameon=False, fontsize=9)
    axes[1].set_title('LiDAR pose at the cloud stamp from /tf (what MATLAB uses)')
    capitalize_labels(fig)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f'saved {out}')


def compare(out, groups, in_bag=None):
    """ラベルごとに、1 回ごとの指標の平均と幅を表にし、tf_wait と prior − NDT の解の分布を並べて描く."""
    res = {}
    for g in groups:
        label, bags = g.split('=', 1)
        res[label] = [check(b, in_bag) for b in bags.split(',')]
    keys = [('tf_rate', None), ('tf_sent', None), ('tf_wait', 'mean'), ('tf_wait', 'max'),
            ('tf_wait_receipt', 'mean'), ('tf_wait_receipt', 'max'), ('waited', None),
            ('ndt_latency', 'mean'), ('n_ndt', None),
            ('prior_ndt', 'xy', 'mean'), ('prior_ndt', 'xy', 'max'), ('prior_ndt', 'yaw', 'mean'),
            ('prior_ndt', 'yaw', 'max'), ('prior_post', 'xy', 'mean'), ('prior_post', 'xy', 'max'),
            ('prior_post', 'yaw', 'mean'), ('prior_post', 'yaw', 'max'), ('prior_truth', 'xy', 'mean'),
            ('prior_truth', 'yaw', 'mean'), ('post_truth', 'xy', 'mean'), ('post_truth', 'yaw', 'mean')]

    def get(m, key):
        for p in key:
            if p is None:
                break
            if not isinstance(m, dict) or p not in m:
                return None
            m = m[p]
        return m
    print('\n' + 'metric'.ljust(26) + ''.join(label.rjust(26) for label in res))
    for key in keys:
        cells = []
        for runs in res.values():
            v = [get(m, key) for m, _ in runs]
            v = [x for x in v if x is not None]
            cells.append(f'{np.mean(v):.2f} ({min(v):.2f}-{max(v):.2f})' if v else '-')
        if any(c != '-' for c in cells):
            print('.'.join(p for p in key if p).ljust(26) + ''.join(c.rjust(26) for c in cells))

    setup()
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2), facecolor=SURFACE)
    lws = widths_back_to_front(len(res))
    for ax, (i, scale, xlabel) in zip(axes, [(1, 1.0, 'Cloud stamp → first /tf at or after it [ms]'),
                                             (2, 1e3, 'Prior − NDT solution, horizontal [mm]'),
                                             (3, 180 / np.pi, 'Prior − NDT solution, |yaw| [deg]')]):
        style(ax)
        for (label, runs), color, lw in zip(res.items(), SERIES, lws):
            v = np.concatenate([r[:, i] for _, r in runs]) * scale
            v = np.sort(v if i == 1 else np.abs(v))   # tf_wait は負（点群の stamp より前に届いた）もそのまま
            ax.plot(v, np.arange(1, len(v) + 1) / len(v), color=color, lw=lw, label=label)
        ax.set_xlabel(xlabel)
        ax.set_ylabel('Cumulative fraction')
        box_on(ax)
    axes[0].legend(loc='lower right', frameon=False, fontsize=9)
    capitalize_labels(fig)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f'saved {out}')


def main():
    args = sys.argv[1:]
    in_bag = None
    if '--input' in args:
        i = args.index('--input')
        in_bag = args[i + 1]
        del args[i:i + 2]
    if len(args) >= 3 and args[0] == 'compare':
        compare(args[1], args[2:], in_bag)
    elif len(args) == 2:
        _, rows = check(args[0], in_bag)
        plot(args[0], args[1], rows)
    else:
        print(__doc__)
        sys.exit(1)


if __name__ == '__main__':
    main()
