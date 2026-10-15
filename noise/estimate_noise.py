#!/usr/bin/env python3
"""観測ノイズの共分散 R（オドメトリ・NDT）を、真値を使わずに録画から求める（コンテナ test 内で実行）.

  python3 estimate_noise.py <bag> [--out <出力の接頭辞>] [--truth] [オプション]

考え方（実機と同じく真値なしで求める）
- 指令（cmd_vel）が一定の区間（変化の --settle 秒後から次の変化の --margin 秒前まで）では、車体の真の速度・姿勢は
  滑らかに変わる。各観測から滑らかな成分（前後 --window/2 秒の局所 1 次式）を取り除き、残り（残差）の分散を
  観測ノイズとみなす。残差は当てはめの自由度の分だけ小さく出るので、てこ比 h で r / sqrt(1 - h) に直してから分散を取る。
- オドメトリ：v・omega（--scale を掛けた後。推定器に入るのと同じ値）の残差 → R_odom [v, omega]
  - R_v は定速で直進する区間の v の残差から求める（直進の区間が無ければ、動いている区間全体）
  - R_omega は R_v と車輪の間隔（--tread）から計算する：左右の車輪の速度のノイズが独立で同じ大きさ sigma_w なら
    v = (vR + vL) / 2、omega = (vR - vL) / b から Var(v) = sigma_w^2 / 2、Var(omega) = 2 sigma_w^2 / b^2。
    倍率 s_v・s_omega を掛けた後の値なので R_omega = 4 R_v (s_omega / s_v)^2 / b^2（b / s_omega が実効的な車輪の間隔）。
    直接測った omega の残差（直進・旋回・動いている区間）も比べるために出す（旋回中は滑りなど車輪の速度以外のばらつきも入る）
- NDT：LiDAR の姿勢を取付（/tf_static か --mount）で車体の姿勢に直して局所 1 次式を当て、それを LiDAR に戻した姿勢との差
  （map 座標の x, y, z と roll, pitch, yaw）→ R_ndt [x, y, z, roll, pitch, yaw]（ekf_localizer の並び）
- 区間は 停止 / 旋回 / 直進 に分ける。R の推奨値は「動いている区間（旋回 + 直進）」をまとめた分散（停止は参考）
- 時間的なつながりの補正（既定で有効、--no-correlation で無効）：EKF は観測ノイズを毎回独立とみなすが、実際の誤差は
  つながっている（同じ情報が何度も来るのと同じ）。長めの窓（--corr-window）の残差の自己相関 rho_k から
  積分相関時間 tau_int = 1 + 2 sum rho_k（rho_k が正の間、--max-lag 秒まで）を求め、R_eff = 分散 x tau_int を推奨値にする
- --truth（Gazebo だけ）：同じ区間で真値（/wheelchair/pose）との差の分散も出して比べる。R の決定には使わない

出力：表（標準出力）、<接頭辞>.yaml（env の parameters に貼れる形）、<接頭辞>.json、
      <接頭辞>_segments.png（区間と残差）、<接頭辞>_summary.png（標準偏差の比較）、
      <接頭辞>_hist.png（残差のヒストグラムと正規分布）
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
from pose_math import (matrix_to_vec, pose_msg_to_matrix,  # noqa: E402
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


def correlation_time(rows, keys, rate, max_lag):
    """残差の自己相関から積分相関時間 tau_int（サンプル数）と rho_1..（各成分）を求める.

    rows は区間ごとの残差（dict）。区間をまたがない組だけで自己共分散を取り、全体の分散で割る。
    rho_k が 0 以下になったところで打ち切って足す（initial positive sequence）.
    """
    out = {}
    nlag = max(1, int(round(max_lag * rate)))
    for k in keys:
        rs = [r[k][np.isfinite(r[k])] for r in rows]
        rs = [r for r in rs if len(r) > 2]
        if not rs:
            out[k] = (float('nan'), [])
            continue
        var = np.mean(np.concatenate(rs) ** 2)
        rho = []
        for lag in range(1, nlag + 1):
            pairs = [(r[:-lag] * r[lag:]).sum() for r in rs if len(r) > lag]
            count = sum(len(r) - lag for r in rs if len(r) > lag)
            if count < 10:
                break
            rho.append(sum(pairs) / count / var)
        total = 0.0
        for x in rho:
            if x <= 0:
                break
            total += x
        out[k] = (1.0 + 2.0 * total, rho)
    return out


def pooled(rows, keys, classes):
    """区間ごとの残差（平均は引かない）を class ごとにまとめた分散と標本数."""
    out = {}
    for c in classes:
        res = {k: np.concatenate([r[k] for cls, r in rows if cls == c] or [np.array([])]) for k in keys}
        out[c] = {k: (float(np.nanmean(res[k] ** 2)) if np.isfinite(res[k]).any() else float('nan'),
                      int(np.isfinite(res[k]).sum())) for k in keys}
    return out


def plot_residual_hist(path, odom_rows, ndt_rows, sigma_w_tread, args):
    """残差のヒストグラムと正規分布：v（直進）、omega（直進・旋回）、NDT の 6 成分（動いている区間）."""
    from plot_noise import C_EFF, C_TRUTH, plot_hist

    def pick(rows, key, classes):
        return np.concatenate([r[key] for cls, r in rows if cls in classes] or [np.array([])])

    def rms(x):
        x = x[np.isfinite(x)]
        return float(np.sqrt(np.mean(x ** 2))) if len(x) else float('nan')

    moving = ('turning', 'straight', 'curve')
    panels = []
    for title, key, classes, f, unit in (('Odometry v, straight', 'v', ('straight',), 1e3, 'mm/s'),
                                         ('Odometry omega, straight', 'omega', ('straight',), 1e3, 'mrad/s'),
                                         ('Odometry omega, turning', 'omega', ('turning',), 1e3, 'mrad/s')):
        x = pick(odom_rows, key, classes) * f
        curves = [('Normal, residual sigma', rms(x), C_EFF)]
        if key == 'omega':
            curves.append((f'Normal from v and tread {args.tread} m', sigma_w_tread * f, C_TRUTH))
        panels.append({'title': title, 'unit': unit, 'data': x, 'curves': curves})
    for key in NDT_KEYS:
        f, unit = (1e3, 'mm') if key in ('x', 'y', 'z') else (math.degrees(1.0), 'deg')
        x = pick(ndt_rows, key, moving) * f
        panels.append({'title': f'NDT {key}, moving', 'unit': unit, 'data': x,
                       'curves': [('Normal, residual sigma', rms(x), C_EFF)]})
    plot_hist(path, panels, f'Residual distribution and the normal distribution (no truth; local linear window '
                            f'{args.window:.1f} s, no correlation correction)')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--out', default='/root/ros2_ws/fig/noise')
    ap.add_argument('--odom', default='/wheelchair/odom')
    ap.add_argument('--ndt', default='/ndt_pose')
    ap.add_argument('--cmd', default='/wheelchair/diff_drive_controller/cmd_vel')
    ap.add_argument('--scale', default='1.0,0.922', help='odom_scale [v, omega]（推定器に入るのと同じ値にする）')
    ap.add_argument('--tread', type=float, default=0.492,
                    help='オドメトリが omega を計算するときの車輪の間隔 [m]（R_omega の計算に使う。Gazebo 0.492（diff_drive_controller の '
                         'wheel_separation）、実機は whill_odometry の tread_width）')
    ap.add_argument('--base', default='base_footprint')
    ap.add_argument('--lidar', default='velodyne')
    ap.add_argument('--mount', default='', help='x,y,z,roll,pitch,yaw（/tf_static に無いとき）')
    ap.add_argument('--settle', type=float, default=0.5, help='指令が変わってから使わない時間 [s]')
    ap.add_argument('--margin', type=float, default=0.1, help='次の変化の前に使わない時間 [s]')
    ap.add_argument('--min-len', type=float, default=1.0, help='使う区間の最短 [s]')
    ap.add_argument('--window', type=float, default=1.0, help='局所 1 次式の窓 [s]')
    ap.add_argument('--cmd-timeout', type=float, default=0.5)
    ap.add_argument('--corr-window', type=float, default=3.0, help='時間的なつながりを測るときの局所 1 次式の窓 [s]')
    ap.add_argument('--max-lag', type=float, default=2.0, help='自己相関を足す最大の遅れ [s]')
    ap.add_argument('--no-correlation', action='store_true', help='時間的なつながりの補正をしない（R = 分散）')
    ap.add_argument('--truth', action='store_true', help='Gazebo の真値との差も出す（比較だけ）')
    args = ap.parse_args()
    sv, sw = (float(x) for x in args.scale.split(','))
    half = args.window / 2.0

    d = read_bag(args.bag, ['/clock', '/tf_static', args.odom, args.ndt, args.cmd]
                 + (['/wheelchair/pose'] if args.truth else []))
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
    corr_odom, corr_ndt = [], []   # 時間的なつながりを測る残差（動いている区間、長めの窓）
    corr_straight = []             # 同じく直進の区間だけ（R_v 用）
    half_c = args.corr_window / 2.0
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
        if cls != 'stopped':
            corr_odom.append(odom_residuals(ot[mo], ov[mo], ow[mo], half_c))
            corr_ndt.append(ndt_residuals(nt[mn], [mats[i] for i in np.where(mn)[0]], mount, half_c))
            if cls == 'straight':
                corr_straight.append(corr_odom[-1])
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

    r_odom_white = [est_odom['moving'][k][0] for k in ODOM_KEYS]
    r_ndt_white = [est_ndt['moving'][k][0] for k in NDT_KEYS]
    tau_odom = correlation_time(corr_odom, ODOM_KEYS, 1.0 / np.median(np.diff(ot)), args.max_lag)
    tau_ndt = correlation_time(corr_ndt, NDT_KEYS, 1.0 / np.median(np.diff(nt)), args.max_lag)
    print(f'\ncorrelation (moving, window {args.corr_window} s): tau_int and rho_1..3')
    for k, (tau, rho) in list(tau_odom.items()) + list(tau_ndt.items()):
        print(f'  {k:<6} tau_int {tau:6.2f}   rho ' + ' '.join(f'{x:+.2f}' for x in rho[:3]))
    if args.no_correlation:
        r_odom_moving, r_ndt = r_odom_white, r_ndt_white
    else:
        r_odom_moving = [r * tau_odom[k][0] for r, k in zip(r_odom_white, ODOM_KEYS)]
        r_ndt = [r * tau_ndt[k][0] for r, k in zip(r_ndt_white, NDT_KEYS)]

    # R_v は直進の区間から、R_omega は R_v と車輪の間隔から（左右の車輪の速度のノイズが独立で同じ大きさと仮定）
    var_v_straight = est_odom['straight']['v'][0] if est_odom['straight']['v'][1] else float('nan')
    tau_v_straight = correlation_time(corr_straight, ('v',), 1.0 / np.median(np.diff(ot)), args.max_lag)['v'][0]
    tread_factor = 4.0 * (sw / sv) ** 2 / args.tread ** 2
    if np.isfinite(var_v_straight):
        v_from = 'straight'
        tau_v = 1.0 if args.no_correlation or not np.isfinite(tau_v_straight) else tau_v_straight
        r_v_white, r_v = var_v_straight, var_v_straight * tau_v
    else:
        print('\n直進の区間が無いので、R_v は動いている区間全体から求める')
        v_from, tau_v = 'moving', (1.0 if args.no_correlation else tau_odom['v'][0])
        r_v_white, r_v = r_odom_white[0], r_odom_moving[0]
    r_w_tread, r_w_tread_white = r_v * tread_factor, r_v_white * tread_factor
    r_odom = [r_v, r_w_tread]
    measured_w = {c: est_odom[c]['omega'][0] for c in ('straight', 'turning', 'moving')}
    print(f'\nR_odom: v from {v_from} segments (tau_int {tau_v:.2f}), omega = 4 R_v (s_omega / s_v)^2 / b^2 '
          f'(tread b {args.tread} m, scale {sv}, {sw}; effective tread {args.tread / sw * sv:.4f} m)')
    print(f'  sigma_v      {math.sqrt(r_v_white):.4g} m/s (white)   R_v {r_v:.3g}')
    print(f'  sigma_omega  {math.sqrt(r_w_tread_white):.4g} rad/s (from v and tread, white)   R_omega {r_w_tread:.3g}')
    print('  measured omega residual sigma (white): ' + ', '.join(
        f'{c} {math.sqrt(x):.4g}' if np.isfinite(x) else f'{c} -' for c, x in measured_w.items())
        + ' rad/s' + (f'   moving x tau_int: R_omega {r_odom_moving[1]:.3g}' if not args.no_correlation else ''))

    snippet = {
        'odom_covariance': {'R_odom': [float(f'{x:.3g}') for x in r_odom]},
        'ekf_localizer': {'R_ndt': [float(f'{x:.3g}') for x in r_ndt]},
    }
    how = '分散' if args.no_correlation else '分散 x 積分相関時間'
    print(f'\n# R（R_v は{"直進の" if v_from == "straight" else "動いている"}区間、R_omega は R_v と車輪の間隔 {args.tread} m から、'
          f'R_ndt は動いている区間の残差の{how}）。env の parameters に入れる\n' + yaml.safe_dump(snippet, sort_keys=False))
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out + '.yaml', 'w') as f:
        f.write(f'# estimate_noise.py {args.bag}（真値なし。残差の{how}）\n'
                f'# R_odom：v は{"直進の" if v_from == "straight" else "動いている"}区間、omega は 4 R_v (s_omega / s_v)^2 / b^2'
                f'（b {args.tread} m、倍率 {sv}, {sw}）\n'
                f'# 直接測った omega の R（動いている区間{"" if args.no_correlation else " x 積分相関時間"}）は '
                f'{r_odom_moving[1]:.3g}（比べるだけ）\n')
        yaml.safe_dump(snippet, f, sort_keys=False)
    with open(args.out + '.json', 'w') as f:
        json.dump({'bag': args.bag, 'segments': used, 'odom': est_odom, 'ndt': est_ndt,
                   'truth_odom': tru_odom, 'truth_ndt': tru_ndt, 'R_odom': r_odom, 'R_ndt': r_ndt,
                   'R_odom_white': [r_v_white, r_w_tread_white], 'R_ndt_white': r_ndt_white,
                   'R_odom_moving': r_odom_moving, 'R_odom_moving_white': r_odom_white,
                   'R_v_from': v_from, 'tau_v': tau_v, 'tread': args.tread, 'scale': [sv, sw],
                   'tau_odom': {k: v[0] for k, v in tau_odom.items()},
                   'tau_ndt': {k: v[0] for k, v in tau_ndt.items()}}, f, indent=1)
    print(f'saved {args.out}.yaml, {args.out}.json')

    from plot_noise import plot_segments, plot_summary
    plot_segments(args.out + '_segments.png', ot, ov, ow, nt, used, odom_rows, ndt_rows, segs, half)
    tau = None if args.no_correlation else {k: v[0] for k, v in list(tau_odom.items()) + list(tau_ndt.items())}
    plot_summary(args.out + '_summary.png', est_odom, est_ndt, tru_odom, tru_ndt, tau)
    plot_residual_hist(args.out + '_hist.png', odom_rows, ndt_rows, math.sqrt(r_w_tread_white), args)


if __name__ == '__main__':
    main()
