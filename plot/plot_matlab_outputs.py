#!/usr/bin/env python3
"""MATLAB が /tf から引く値（後輪間中心の最新の姿勢と、点群の stamp の時刻の LiDAR の姿勢）を録画で確かめ、図を描く.

  python3 plot_matlab_outputs.py <出力を録った bag> <出力 png>                         1 回分の確かめと図
  python3 plot_matlab_outputs.py compare <出力 png> <ラベル>=<bag>[,<bag>...] ...      ラベルごとの平均と幅、分布の図

出力の bag は eval/replay_outputs.py で録ったもの（/tf・/tf_static・/tf_ndt_update・/ndt_pose・/clock。Gazebo は /wheelchair/pose も）。
確かめること（結果は標準出力。最後の行は METRICS <1 行 JSON>）：
  tf          map -> base の /tf の stamp の間隔（predict_rate の周期か）と、stamp から受信までの時間
  tf_wait     点群の stamp（/ndt_pose の stamp）より後の /tf が届くまでの時間（点群の stamp から）。NDT と MATLAB が
              点群の stamp の時刻の TF を引けるのはこれより後（点群は stamp から約 15 ms 後に届く）
  prior       点群の stamp の時刻の /tf（tf2 と同じく前後の 2 つを補間。位置は線形、回転は slerp）に取付を掛けた
              map -> LiDAR。MATLAB が点群の座標変換に使う値で、NDT の初期値と同じ
  prior_ndt   prior − NDT の解（/ndt_pose）。NDT が初期値からどれだけ動いたか
  prior_post  prior − NDT で更新した直後の値（/tf_ndt_update に取付）。2026-10-26 まで MATLAB が使っていた vlp16_pose との差
  Gazebo だけ prior_truth・post_truth（真値の LiDAR との差。真値は /wheelchair/pose に URDF の取付）
差は水平の位置 [mm] と |yaw| [deg]。受信の時刻は bag の時刻（壁時計）。stamp は /clock で壁時計に直す。
図（1 回分）：(1) 点群の stamp から TF が届くまでの時間の分布、(2)(3) prior − NDT の解、prior − NDT で更新した直後の値の時間推移。
"""
import json
import os
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


class TfInterp:
    """map -> base の /tf を、tf2 と同じく前後の 2 つで補間する（その時刻より後の TF が無ければ None）."""

    def __init__(self, items):
        items = sorted(items, key=lambda x: x[0])
        self.k = np.array([k for k, _, _ in items])
        self.recv = np.array([t for _, t, _ in items])
        T = np.array([M for _, _, M in items])
        self.p = T[:, :3, 3]
        self.slerp = Slerp(self.k * 1e-9, Rotation.from_matrix(T[:, :3, :3]))

    def at(self, k):
        if not self.k[0] <= k <= self.k[-1]:
            return None
        T = np.eye(4)
        T[:3, 3] = [np.interp(k, self.k, self.p[:, i]) for i in range(3)]
        T[:3, :3] = self.slerp([k * 1e-9]).as_matrix()[0]
        return T

    def available(self, k):
        """stamp k の TF を引けるようになる受信時刻 [s]（stamp が k 以上の最初の TF の受信時刻）."""
        i = np.searchsorted(self.k, k)
        return self.recv[i] if i < len(self.k) else None


def check(bag):
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
    print(f'/tf map -> {base_frame}: {len(interp.k)} msgs, {rate:.1f} Hz')
    out['tf_rate'] = rate
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

    rows = []   # [stamp s, tf_wait ms, prior-ndt xy, yaw, prior-post xy, yaw, prior-truth xy, yaw, post-truth xy, yaw]
    for k in sorted(ndt):
        P = interp.at(k)
        avail = interp.available(k)
        if P is None or avail is None:
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


def compare(out, groups):
    """ラベルごとに、1 回ごとの指標の平均と幅を表にし、tf_wait と prior − NDT の解の分布を並べて描く."""
    res = {}
    for g in groups:
        label, bags = g.split('=', 1)
        res[label] = [check(b) for b in bags.split(',')]
    keys = [('tf_rate', None), ('tf_wait', 'mean'), ('tf_wait', 'max'), ('ndt_latency', 'mean'), ('n_ndt', None),
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
    for ax, (i, scale, xlabel) in zip(axes, [(1, 1.0, 'Cloud stamp → first /tf after it [ms]'),
                                             (2, 1e3, 'Prior − NDT solution, horizontal [mm]'),
                                             (3, 180 / np.pi, 'Prior − NDT solution, |yaw| [deg]')]):
        style(ax)
        for (label, runs), color, lw in zip(res.items(), SERIES, lws):
            v = np.sort(np.abs(np.concatenate([r[:, i] for _, r in runs])) * scale)
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
    if len(sys.argv) >= 4 and sys.argv[1] == 'compare':
        compare(sys.argv[2], sys.argv[3:])
    elif len(sys.argv) == 3:
        _, rows = check(sys.argv[1])
        plot(sys.argv[1], sys.argv[2], rows)
    else:
        print(__doc__)
        sys.exit(1)


if __name__ == '__main__':
    main()
