#!/usr/bin/env python3
"""LiDAR の取付位置とオドメトリの倍率を、NDT の解とオドメトリから推定する（真値なし。localizer_rl 内で実行）.

  python3 estimate_calibration.py <bag> [<bag> ...] [--out /root/fig/calib] [--truth] [--map <地図の pcd>]

使う走行：その場の旋回（取付位置・omega の倍率）と直進（v の倍率・取付の yaw）。区間は指令（cmd_vel）が一定のところ
（変化の --settle 秒後から次の変化の --margin 秒前まで）。gazebo/drive_spin.py・drive_stops.py の走行で求まる。

- omega の倍率：旋回の区間ごとに、NDT の yaw の変化 / オドメトリの omega の積分（全区間の和の比）
- v の倍率：直進の区間ごとに、NDT の LiDAR の移動距離 / オドメトリの v の積分（向きが一定なので LiDAR と車体の移動は同じ）
- 取付の水平位置 (x, y)：その場の旋回では車体（回転の中心）が動かないので、LiDAR の位置 p = c + R(yaw_L) m'
  （c は区間ごとの回転の中心、ゆっくりした移動も 1 次で許す）を最小二乗で解く。m' = R(-yaw_m) m
- 取付の yaw：直進の区間で、LiDAR の yaw - 進む向き atan2(dy, dx)（横滑りしないと仮定）
- 取付の z・roll・pitch（参考）：止まっている区間の NDT の roll・pitch（床が水平と仮定）と、--map の地図から求めた
  床の高さとの差（地図の床が正しいと仮定）
- --truth（Gazebo だけ）：/tf_static の取付と、真値の yaw の変化・移動距離から求めた倍率を並べて出す
入力の odom は倍率を掛ける前の値（/wheelchair/odom など）。今の設定（--current-scale、/tf_static の取付）と比べる。
"""
import argparse
import json
import math
import os
import struct
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise', 'calib')]   # plot_spin などを import する
from estimate_noise import classify, command_segments, mount_from_tf_static, stamp  # noqa: E402
from plot_spin import read_bag  # noqa: E402
from robot_localization_adapter import matrix_to_vec, pose_msg_to_matrix  # noqa: E402


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def load(bag, args):
    d = read_bag(bag, ['/clock', '/tf_static', args.odom, args.ndt, args.cmd]
                 + (['/wheelchair/pose'] if args.truth else []))
    if '/clock' in d:
        clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d['/clock']])

        def to_sim(ts):
            return np.interp(ts, clk[:, 0], clk[:, 1])
    else:
        def to_sim(ts):
            return np.asarray(ts, float)
    od = sorted((stamp(m), m.twist.twist.linear.x, m.twist.twist.angular.z) for _, m in d[args.odom])
    ot, ov, ow = (np.array(x) for x in zip(*od))
    nd = sorted((stamp(m), matrix_to_vec(pose_msg_to_matrix(m.pose.pose))) for _, m in d[args.ndt])
    nt = np.array([t for t, _ in nd])
    nv = np.array([v for _, v in nd])
    nv[:, 5] = np.unwrap(nv[:, 5])
    segs = command_segments(d, args.cmd, to_sim, max(ot[0], nt[0]), min(ot[-1], nt[-1]), args.cmd_timeout)
    truth = None
    if args.truth and '/wheelchair/pose' in d:
        pt = to_sim([t for t, _ in d['/wheelchair/pose']])
        pv = np.array([matrix_to_vec(pose_msg_to_matrix(m)) for _, m in d['/wheelchair/pose']])
        o = np.argsort(pt)
        pt, pv = pt[o], pv[o]
        pv[:, 5] = np.unwrap(pv[:, 5])
        truth = (pt, pv)
    return {'name': os.path.basename(bag.rstrip('/')), 'd': d, 'ot': ot, 'ov': ov, 'ow': ow, 'nt': nt, 'nv': nv,
            'segs': segs, 'truth': truth}


def integrate(t, y, a, b):
    """y(t) の a..b の積分（台形、端は補間）."""
    m = (t > a) & (t < b)
    tt = np.concatenate([[a], t[m], [b]])
    yy = np.concatenate([[np.interp(a, t, y)], y[m], [np.interp(b, t, y)]])
    return float(np.trapz(yy, tt))


def fit_offset(turns):
    """その場の旋回の LiDAR の位置から、取付の水平位置 m'（LiDAR の yaw の座標）と各区間の回転の中心を最小二乗で求める.

    p_k = c_j + c1_j (t_k - tm_j) + R(yaw_k) m'   （未知数：m' 2 個 + 区間ごとに c 2 個と c1 2 個）
    """
    n_seg = len(turns)
    rows, rhs = [], []
    for j, (t, p, yaw) in enumerate(turns):
        tm = t.mean()
        for tk, pk, yk in zip(t, p, yaw):
            r = rot(yk)
            for axis in range(2):
                row = np.zeros(2 + 4 * n_seg)
                row[:2] = r[axis]
                row[2 + 4 * j + axis] = 1.0
                row[2 + 4 * j + 2 + axis] = tk - tm
                rows.append(row)
                rhs.append(pk[axis])
    a, b = np.array(rows), np.array(rhs)
    x, *_ = np.linalg.lstsq(a, b, rcond=None)
    res = b - a @ x
    dof = max(1, len(b) - len(x))
    cov = np.linalg.inv(a.T @ a) * (res @ res / dof)
    return x[:2], np.sqrt(np.diag(cov)[:2]), float(np.sqrt(res @ res / dof)), x


def read_pcd_xyz(path):
    """PCD（ascii / binary、x y z を含む）を読む."""
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii', 'replace').strip()
            if not line or line.startswith('#'):
                continue
            key, *vals = line.split()
            header[key.upper()] = vals
            if key.upper() == 'DATA':
                break
        fields = header['FIELDS']
        sizes = [int(s) for s in header['SIZE']]
        types = header['TYPE']
        counts = [int(c) for c in header.get('COUNT', ['1'] * len(fields))]
        n = int(header['POINTS'][0])
        if header['DATA'][0] == 'ascii':
            data = np.loadtxt(f, dtype=np.float64)
            idx = [sum(counts[:fields.index(k)]) for k in 'xyz']
            return data[:, idx]
        if header['DATA'][0] != 'binary':
            raise ValueError('binary_compressed の PCD は読めない')
        fmt = ''.join({('F', 4): 'f', ('F', 8): 'd', ('U', 1): 'B', ('U', 2): 'H', ('U', 4): 'I',
                       ('I', 1): 'b', ('I', 2): 'h', ('I', 4): 'i'}[(t, s)] * c for t, s, c in zip(types, sizes, counts))
        step = struct.calcsize('<' + fmt)
        raw = np.frombuffer(f.read(n * step), dtype=np.uint8).reshape(n, step)
        offs = [sum(s * c for s, c in zip(sizes[:fields.index(k)], counts[:fields.index(k)])) for k in 'xyz']
        dt = '<f4' if sizes[fields.index('x')] == 4 else '<f8'
        return np.stack([raw[:, o:o + sizes[fields.index('x')]].copy().view(dt)[:, 0] for o in offs], axis=1)


def floor_height(points, xy, z_lidar, radius=1.0):
    """xy の周り radius m の地図の点のうち、LiDAR より 0.5 m 以上低い所で最も点の多い 1 cm の高さ（床）."""
    m = (np.hypot(points[:, 0] - xy[0], points[:, 1] - xy[1]) < radius) & (points[:, 2] < z_lidar - 0.5)
    z = points[m, 2]
    if len(z) < 20:
        return float('nan'), len(z)
    hist, edges = np.histogram(z, bins=np.arange(z.min(), z.max() + 0.02, 0.01))
    i = int(np.argmax(hist))
    sel = z[(z >= edges[i] - 0.01) & (z <= edges[i + 1] + 0.01)]
    return float(np.median(sel)), len(z)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', default='/root/fig/calib')
    ap.add_argument('--odom', default='/wheelchair/odom', help='倍率を掛ける前のオドメトリ')
    ap.add_argument('--ndt', default='/ndt_pose')
    ap.add_argument('--cmd', default='/wheelchair/diff_drive_controller/cmd_vel')
    ap.add_argument('--base', default='base_footprint')
    ap.add_argument('--lidar', default='velodyne')
    ap.add_argument('--settle', type=float, default=0.5)
    ap.add_argument('--margin', type=float, default=0.1)
    ap.add_argument('--min-len', type=float, default=1.0)
    ap.add_argument('--cmd-timeout', type=float, default=0.5)
    ap.add_argument('--current-scale', default='1.0,0.922', help='今の odom_scale [v, omega]（比べるだけ）')
    ap.add_argument('--map', default='', help='地図の pcd（取付の z を床の高さから求めるとき）')
    ap.add_argument('--truth', action='store_true', help='Gazebo の真値でも同じ量を求めて並べる（比較だけ）')
    args = ap.parse_args()

    runs = [load(b, args) for b in args.bags]
    mount_now = mount_from_tf_static(runs[0]['d'], args.base, args.lidar)
    cur_v, cur_w = (float(x) for x in args.current_scale.split(','))

    turns, w_rows, v_rows, yaw_rows, stops = [], [], [], [], []
    for r in runs:
        for t0, t1, v, w in r['segs']:
            a, b = t0 + args.settle, t1 - args.margin
            cls = classify(v, w)
            if b - a < args.min_len:
                continue
            mn = (r['nt'] >= a) & (r['nt'] <= b)
            if mn.sum() < 5:
                continue
            nt, nv = r['nt'][mn], r['nv'][mn]
            ta, tb = nt[0], nt[-1]
            row = {'bag': r['name'], 'cls': cls, 't0': float(ta), 't1': float(tb)}
            if r['truth'] is not None:
                pt, pv = r['truth']
                tru_a = np.array([np.interp(ta, pt, pv[:, i]) for i in range(6)])
                tru_b = np.array([np.interp(tb, pt, pv[:, i]) for i in range(6)])
            if cls == 'turning':
                dyaw = float(nv[-1, 5] - nv[0, 5])
                odom = integrate(r['ot'], r['ow'], ta, tb)
                row.update(dyaw=dyaw, odom=odom, ratio=dyaw / odom)
                if r['truth'] is not None:
                    row['truth_ratio'] = float((tru_b[5] - tru_a[5]) / odom)
                w_rows.append(row)
                turns.append((nt, nv[:, :2], nv[:, 5]))
            elif cls == 'straight':
                dist = float(np.hypot(*(nv[-1, :2] - nv[0, :2])))
                odom = integrate(r['ot'], r['ov'], ta, tb)
                heading = math.atan2(nv[-1, 1] - nv[0, 1], nv[-1, 0] - nv[0, 0])
                if odom < 0:   # 後退
                    heading += math.pi
                yaw_m = math.atan2(math.sin(nv[:, 5].mean() - heading), math.cos(nv[:, 5].mean() - heading))
                row.update(dist=dist, odom=odom, ratio=dist / abs(odom), yaw_m=yaw_m)
                if r['truth'] is not None:
                    row['truth_ratio'] = float(np.hypot(*(tru_b[:2] - tru_a[:2])) / abs(odom))
                v_rows.append(row)
            elif cls == 'stopped':
                stops.append({'bag': r['name'], 'vec': nv.mean(axis=0)})

    out = {}
    print(f'segments: {len(w_rows)} turning, {len(v_rows)} straight, {len(stops)} stopped')
    if w_rows:
        sw = sum(x['dyaw'] for x in w_rows) / sum(x['odom'] for x in w_rows)
        out['omega_scale'] = sw
        print(f'\nomega scale (NDT yaw change / odometry): {sw:.4f}   now {cur_w}')
        for x in w_rows:
            extra = f"  truth {x['truth_ratio']:.4f}" if 'truth_ratio' in x else ''
            print(f"  {x['bag']:<18} {x['t0']:8.2f}-{x['t1']:8.2f}  {math.degrees(x['dyaw']):8.1f} deg / "
                  f"{math.degrees(x['odom']):8.1f} deg = {x['ratio']:.4f}{extra}")
    if v_rows:
        sv = sum(x['dist'] for x in v_rows) / sum(abs(x['odom']) for x in v_rows)
        out['v_scale'] = sv
        print(f'\nv scale (NDT distance / odometry): {sv:.4f}   now {cur_v}')
        for x in v_rows:
            extra = f"  truth {x['truth_ratio']:.4f}" if 'truth_ratio' in x else ''
            print(f"  {x['bag']:<18} {x['t0']:8.2f}-{x['t1']:8.2f}  {x['dist']:7.3f} m / {abs(x['odom']):7.3f} m = "
                  f"{x['ratio']:.4f}  mount yaw {math.degrees(x['yaw_m']):+.2f} deg{extra}")
    yaw_m = float(np.mean([x['yaw_m'] for x in v_rows])) if v_rows else 0.0
    now = matrix_to_vec(mount_now) if mount_now is not None else np.full(6, np.nan)
    if turns:
        m_rot, m_sd, rms, sol = fit_offset(turns)
        m = rot(yaw_m) @ m_rot
        out.update(mount_xy=m.tolist(), mount_xy_sd=m_sd.tolist(), mount_yaw=yaw_m, fit_rms=rms)
        print(f'\nLiDAR mount (base -> LiDAR), from {len(turns)} turning segments (fit rms {rms * 1000:.1f} mm):')
        print(f'  x {m[0]:+.4f} m (+-{m_sd[0]:.4f})   now {now[0]:+.4f}')
        print(f'  y {m[1]:+.4f} m (+-{m_sd[1]:.4f})   now {now[1]:+.4f}')
        print(f'  yaw {math.degrees(yaw_m):+.3f} deg (from straight segments)   now {math.degrees(now[5]):+.3f}')
    if stops:
        rp = np.array([s['vec'][3:5] for s in stops])
        out['mount_roll_pitch'] = rp.mean(axis=0).tolist()
        print(f'  roll {math.degrees(rp[:, 0].mean()):+.3f} deg, pitch {math.degrees(rp[:, 1].mean()):+.3f} deg '
              f'(stopped NDT, level floor assumed; spread {math.degrees(rp[:, 0].std()):.3f}/'
              f'{math.degrees(rp[:, 1].std()):.3f})   now {math.degrees(now[3]):+.3f}, {math.degrees(now[4]):+.3f}')
        if args.map:
            pts = read_pcd_xyz(args.map)
            zs = []
            for s in stops:
                v = s['vec']
                base_xy = v[:2] - rot(v[5] - yaw_m) @ np.array(out.get('mount_xy', now[:2]))
                fz, n = floor_height(pts, base_xy, v[2])
                zs.append(v[2] - fz)
            zs = np.array(zs)
            out['mount_z'] = float(np.nanmean(zs))
            print(f'  z {np.nanmean(zs):+.4f} m (LiDAR z - map floor, {np.isfinite(zs).sum()} stops, '
                  f'spread {np.nanstd(zs):.4f})   now {now[2]:+.4f}')
    out['mount_now'] = now.tolist()
    out['current_scale'] = [cur_v, cur_w]
    out['turning'] = w_rows
    out['straight'] = v_rows
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out + '.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(f'saved {args.out}.json')
    if turns:
        from plot_calibration import plot_calibration
        plot_calibration(args.out + '.png', turns, sol, out, now)


if __name__ == '__main__':
    main()
