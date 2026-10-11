#!/usr/bin/env python3
"""観測ノイズの共分散 R（オドメトリ・NDT）を、真値を使わずに録画から求める（localizer_rl 内で実行）.

  python3 estimate_noise.py <bag> [--out <出力の接頭辞>] [--truth] [オプション]

考え方（実機と同じく真値なしで求める）
- 指令（cmd_vel）が一定の区間（変化の --settle 秒後から次の変化の --margin 秒前まで）では、車体の真の速度・姿勢は
  滑らかに変わる。各観測から滑らかな成分（前後 --window/2 秒の局所 1 次式）を取り除き、残り（残差）の分散を
  観測ノイズとみなす。残差は当てはめの自由度の分だけ小さく出るので、てこ比 h で r / sqrt(1 - h) に直してから分散を取る。
- オドメトリ：v・omega（--scale を掛けた後。推定器に入るのと同じ値）の残差 → R_odom [v, omega]
- NDT：LiDAR の姿勢を取付（/tf_static か --mount）で車体の姿勢に直して局所 1 次式を当て、それを LiDAR に戻した姿勢との差
  （map 座標の x, y, z と roll, pitch, yaw）→ R_ndt [x, y, z, roll, pitch, yaw]（ekf_localizer・robot_localization_adapter の並び）
- 区間は 停止 / 旋回 / 直進 に分ける。R の推奨値は「動いている区間（旋回 + 直進）」をまとめた分散（停止は参考）
- --truth（Gazebo だけ）：同じ区間で真値（/wheelchair/pose）との差の分散も出して比べる。R の決定には使わない

出力：表（標準出力）、<接頭辞>.yaml（env の parameters に貼れる形）、<接頭辞>.json、
      <接頭辞>_segments.png（区間と残差）、<接頭辞>_summary.png（標準偏差の比較）
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import read_bag  # noqa: E402
from robot_localization_adapter import (matrix_to_vec, pose_msg_to_matrix,  # noqa: E402
                                        transform_msg_to_matrix, vec_to_matrix)

CLASSES = ('stopped', 'turning', 'straight', 'curve')
ODOM_KEYS = ('v', 'omega')
NDT_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def mount_from_tf_static(d, base, lidar):
    """/tf_static から base -> lidar の変換（4x4）. 見つからなければ None."""
    edges = {}
    for _, m in d.get('/tf_static', []):
        for tr in m.transforms:
            edges[tr.child_frame_id] = (tr.header.frame_id, transform_msg_to_matrix(tr.transform))
    t = np.eye(4)
    frame = lidar
    while frame != base:
        if frame not in edges:
            return None
        parent, tf = edges[frame]
        t = tf @ t
        frame = parent
    return t


def command_segments(d, cmd_topic, to_sim, t_begin, t_end, timeout):
    """指令が一定の区間 [(開始, 終了, v, omega)]. 最初の指令までは停止とみなす."""
    cmds = []
    for t_recv, m in d.get(cmd_topic, []):
        tw = m.twist if hasattr(m, 'twist') else m
        t = stamp(m) if hasattr(m, 'header') and stamp(m) > 0 else float(to_sim(t_recv))
        cmds.append((t, tw.linear.x, tw.angular.z))
    cmds.sort()
    changes = [(t_begin, 0.0, 0.0)]
    for t, v, w in cmds:
        if abs(v - changes[-1][1]) > 1e-3 or abs(w - changes[-1][2]) > 1e-3:
            changes.append((t, v, w))
    if cmds and (abs(changes[-1][1]) > 1e-3 or abs(changes[-1][2]) > 1e-3):
        changes.append((cmds[-1][0] + timeout, 0.0, 0.0))   # 指令が途切れると controller が止める
    segs = []
    for i, (t0, v, w) in enumerate(changes):
        t1 = changes[i + 1][0] if i + 1 < len(changes) else t_end
        segs.append((t0, t1, v, w))
    return segs


def classify(v, w):
    if abs(v) < 1e-3 and abs(w) < 1e-3:
        return 'stopped'
    if abs(v) < 1e-3:
        return 'turning'
    if abs(w) < 1e-3:
        return 'straight'
    return 'curve'


def local_linear(t, y, half):
    """各点で前後 half 秒の局所 1 次式の値と、てこ比 h を返す（点が 4 未満なら nan）."""
    fit = np.full(y.shape, np.nan)
    lev = np.full(len(t), np.nan)
    for i, ti in enumerate(t):
        m = np.abs(t - ti) <= half
        n = int(m.sum())
        if n < 4:
            continue
        dt = t[m] - ti
        a = np.vstack([np.ones(n), dt]).T
        coef, *_ = np.linalg.lstsq(a, y[m], rcond=None)
        fit[i] = coef[0]
        tbar = dt.mean()
        sxx = ((dt - tbar) ** 2).sum()
        lev[i] = 1.0 / n + (0.0 - tbar) ** 2 / sxx if sxx > 0 else np.nan
    return fit, lev


def odom_residuals(t, v, w, half):
    out = {}
    for key, y in (('v', v), ('omega', w)):
        fit, lev = local_linear(t, y, half)
        out[key] = (y - fit) / np.sqrt(1.0 - lev)
    return out


def ndt_residuals(t, mats, mount, half):
    inv_mount = np.linalg.inv(mount)
    base = np.array([matrix_to_vec(m @ inv_mount) for m in mats])
    for i in (3, 4, 5):
        base[:, i] = np.unwrap(base[:, i])
    fits = []
    lev = None
    for i in range(6):
        f, lev = local_linear(t, base[:, i], half)
        fits.append(f)
    fits = np.array(fits).T
    res = np.full((len(t), 6), np.nan)
    for k, (m, f) in enumerate(zip(mats, fits)):
        if np.isnan(f).any():
            continue
        pred = vec_to_matrix(f) @ mount
        z, p = matrix_to_vec(m), matrix_to_vec(pred)
        res[k, :3] = z[:3] - p[:3]
        res[k, 3:] = wrap(z[3:] - p[3:])
    res /= np.sqrt(1.0 - lev)[:, None]
    return {key: res[:, i] for i, key in enumerate(NDT_KEYS)}


def truth_errors(d, to_sim, ot, ov, ow, nt, mats, mount):
    """（--truth）オドメトリと NDT の、真値との差（区間ごとの平均は後で引く）."""
    from plot_velocity import truth_velocity
    tt, tv, tw = truth_velocity(d)
    odom = {'v': ov - np.interp(ot, tt, tv), 'omega': ow - np.interp(ot, tt, tw)}
    pt = to_sim([t for t, _ in d['/wheelchair/pose']])
    pv = np.array([matrix_to_vec(pose_msg_to_matrix(m)) for _, m in d['/wheelchair/pose']])
    o = np.argsort(pt)
    pt, pv = pt[o], pv[o]
    pv[:, 5] = np.unwrap(pv[:, 5])
    ndt = np.full((len(nt), 6), np.nan)
    for k, (t, m) in enumerate(zip(nt, mats)):
        if not pt[0] <= t <= pt[-1]:
            continue
        ref = vec_to_matrix([np.interp(t, pt, pv[:, i]) for i in range(6)]) @ mount
        z, r = matrix_to_vec(m), matrix_to_vec(ref)
        ndt[k, :3] = z[:3] - r[:3]
        ndt[k, 3:] = wrap(z[3:] - r[3:])
    return odom, {key: ndt[:, i] for i, key in enumerate(NDT_KEYS)}


def pooled(rows, keys, classes):
    """区間ごとの残差（平均は引かない）を class ごとにまとめた分散と標本数."""
    out = {}
    for c in classes:
        res = {k: np.concatenate([r[k] for cls, r in rows if cls == c] or [np.array([])]) for k in keys}
        out[c] = {k: (float(np.nanmean(res[k] ** 2)) if np.isfinite(res[k]).any() else float('nan'),
                      int(np.isfinite(res[k]).sum())) for k in keys}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--out', default='/root/fig/noise')
    ap.add_argument('--odom', default='/wheelchair/odom')
    ap.add_argument('--ndt', default='/ndt_pose')
    ap.add_argument('--cmd', default='/wheelchair/diff_drive_controller/cmd_vel')
    ap.add_argument('--scale', default='1.0,0.922', help='odom_scale [v, omega]（推定器に入るのと同じ値にする）')
    ap.add_argument('--base', default='base_footprint')
    ap.add_argument('--lidar', default='velodyne')
    ap.add_argument('--mount', default='', help='x,y,z,roll,pitch,yaw（/tf_static に無いとき）')
    ap.add_argument('--settle', type=float, default=0.5, help='指令が変わってから使わない時間 [s]')
    ap.add_argument('--margin', type=float, default=0.1, help='次の変化の前に使わない時間 [s]')
    ap.add_argument('--min-len', type=float, default=1.0, help='使う区間の最短 [s]')
    ap.add_argument('--window', type=float, default=1.0, help='局所 1 次式の窓 [s]')
    ap.add_argument('--cmd-timeout', type=float, default=0.5)
    ap.add_argument('--truth', action='store_true', help='Gazebo の真値との差も出す（比較だけ）')
    args = ap.parse_args()
    sv, sw = (float(x) for x in args.scale.split(','))
    half = args.window / 2.0

    d = read_bag(args.bag)
    if '/clock' in d:
        clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d['/clock']])

        def to_sim(ts):
            return np.interp(ts, clk[:, 0], clk[:, 1])
    else:
        def to_sim(ts):
            return np.asarray(ts, float)
    mount = mount_from_tf_static(d, args.base, args.lidar)
    if mount is None:
        if not args.mount:
            sys.exit(f'{args.base} -> {args.lidar} が /tf_static に無いので --mount を指定する')
        mount = vec_to_matrix([float(x) for x in args.mount.split(',')])
    print('mount', args.base, '->', args.lidar, np.round(matrix_to_vec(mount), 4).tolist())

    od = sorted((stamp(m), m.twist.twist.linear.x * sv, m.twist.twist.angular.z * sw) for _, m in d[args.odom])
    ot, ov, ow = (np.array(x) for x in zip(*od))
    nd = sorted(((stamp(m), pose_msg_to_matrix(m.pose.pose)) for _, m in d[args.ndt]), key=lambda x: x[0])
    nt = np.array([t for t, _ in nd])
    mats = [m for _, m in nd]
    t_begin, t_end = max(ot[0], nt[0]), min(ot[-1], nt[-1])
    segs = command_segments(d, args.cmd, to_sim, t_begin, t_end, args.cmd_timeout)
    if args.truth:
        terr_odom, terr_ndt = truth_errors(d, to_sim, ot, ov, ow, nt, mats, mount)

    odom_rows, ndt_rows, truth_odom_rows, truth_ndt_rows, used = [], [], [], [], []
    for t0, t1, v, w in segs:
        a, b = t0 + args.settle, t1 - args.margin
        if b - a < args.min_len:
            continue
        cls = classify(v, w)
        used.append((a, b, cls))
        mo = (ot >= a) & (ot <= b)
        mn = (nt >= a) & (nt <= b)
        odom_rows.append((cls, {**odom_residuals(ot[mo], ov[mo], ow[mo], half), '_t': ot[mo]}))
        ndt_rows.append((cls, {**ndt_residuals(nt[mn], [mats[i] for i in np.where(mn)[0]], mount, half), '_t': nt[mn]}))
        if args.truth:   # 区間ごとの平均（一定のずれ）を引いた真値との差
            truth_odom_rows.append((cls, {k: terr_odom[k][mo] - np.nanmean(terr_odom[k][mo]) for k in ODOM_KEYS}))
            truth_ndt_rows.append((cls, {k: terr_ndt[k][mn] - np.nanmean(terr_ndt[k][mn]) for k in NDT_KEYS}))

    # 動いている区間（旋回 + 直進）をまとめたものを推奨値にする
    def with_moving(rows):
        return rows + [('moving', r) for cls, r in rows if cls in ('turning', 'straight', 'curve')]
    classes = CLASSES + ('moving',)
    est_odom = pooled(with_moving(odom_rows), ODOM_KEYS, classes)
    est_ndt = pooled(with_moving(ndt_rows), NDT_KEYS, classes)
    tru_odom = pooled(with_moving(truth_odom_rows), ODOM_KEYS, classes) if args.truth else None
    tru_ndt = pooled(with_moving(truth_ndt_rows), NDT_KEYS, classes) if args.truth else None

    print(f'segments used (settle {args.settle} s, margin {args.margin} s, window {args.window} s):')
    for a, b, cls in used:
        print(f'  {a:8.2f} - {b:8.2f} s  {cls:<9} ({b - a:.1f} s)')
    unit = {'v': ('m/s', 1.0), 'omega': ('rad/s', 1.0), 'x': ('mm', 1e3), 'y': ('mm', 1e3), 'z': ('mm', 1e3),
            'roll': ('deg', math.degrees(1.0)), 'pitch': ('deg', math.degrees(1.0)), 'yaw': ('deg', math.degrees(1.0))}
    print(f"\n{'std (sigma)':<14}" + ''.join(f'{c:>13}' for c in classes) + ('   truth(moving)' if args.truth else ''))
    for keys, est, tru in ((ODOM_KEYS, est_odom, tru_odom), (NDT_KEYS, est_ndt, tru_ndt)):
        for k in keys:
            u, f = unit[k]
            cells = ''.join(f'{math.sqrt(est[c][k][0]) * f:13.4f}' if est[c][k][1] else f'{"-":>13}' for c in classes)
            extra = f'{math.sqrt(tru["moving"][k][0]) * f:16.4f}' if tru else ''
            print(f'{k + " [" + u + "]":<14}' + cells + extra)
    print('  (n = ' + ', '.join(f'{c} {est_odom[c]["v"][1]}/{est_ndt[c]["x"][1]}' for c in classes)
          + '  [odom/ndt samples])')

    r_odom = [est_odom['moving'][k][0] for k in ODOM_KEYS]
    r_ndt = [est_ndt['moving'][k][0] for k in NDT_KEYS]
    snippet = {
        'odom_covariance': {'R_odom': [float(f'{x:.3g}') for x in r_odom]},
        'ekf_localizer': {'R_ndt': [float(f'{x:.3g}') for x in r_ndt]},
        'robot_localization_adapter': {'R_ndt': [float(f'{x:.3g}') for x in r_ndt]},
    }
    print('\n# R（動いている区間の残差の分散）。env の parameters に入れる\n' + yaml.safe_dump(snippet, sort_keys=False))
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out + '.yaml', 'w') as f:
        f.write(f'# estimate_noise.py {args.bag}（真値なし。動いている区間の残差の分散）\n')
        yaml.safe_dump(snippet, f, sort_keys=False)
    with open(args.out + '.json', 'w') as f:
        json.dump({'bag': args.bag, 'segments': used, 'odom': est_odom, 'ndt': est_ndt,
                   'truth_odom': tru_odom, 'truth_ndt': tru_ndt, 'R_odom': r_odom, 'R_ndt': r_ndt}, f, indent=1)
    print(f'saved {args.out}.yaml, {args.out}.json')

    from plot_noise import plot_segments, plot_summary
    plot_segments(args.out + '_segments.png', ot, ov, ow, nt, used, odom_rows, ndt_rows, segs, half)
    plot_summary(args.out + '_summary.png', est_odom, est_ndt, tru_odom, tru_ndt)


if __name__ == '__main__':
    main()
