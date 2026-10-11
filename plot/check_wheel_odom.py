#!/usr/bin/env python3
"""Gazebo の左右車輪の回転から v・omega を計算し、diff_drive_controller のオドメトリと真値に照らし合わせる（localizer_rl 内で実行）.

  python3 check_wheel_odom.py <bag>（/tf に robot_state_publisher の車輪リンク、/wheelchair/odom、/wheelchair/pose、/clock）

- 車輪の回転角は /tf の base_link -> {left,right}_wheel_link（URDF の origin rpy (-pi/2, 0, 0) の後に z 軸回り）から取る
- 変換は diff_drive_controller と同じ：v = r (wR + wL) / 2、omega = r (wR - wL) / b（r = 0.1325、b = 0.492）
- 1. diff_drive_controller と同じ計算（位置の差分 / 周期 の 10 サンプル平均 = 直近 0.2 s の回転角の変化 / 0.2 s）と
     /wheelchair/odom を比べる（変換の検証）。録画の車輪の /tf は約 60 ms ごとなので、回転角を補間して求める
- 2. 移動平均の前の値と真値を比べる（倍率、変化の半分に達する時刻の差）
- 3. 回転中の実効的な車輪間隔 b_eff = r (wR - wL) / omega_真値、左右の車輪の滑り
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import read_bag  # noqa: E402
from plot_velocity import half_time, odom_series, phases_of, truth_velocity  # noqa: E402

# B は diff_drive_controller が使う車輪間隔（wheel_separation × wheel_separation_multiplier）。環境変数 WHEEL_SEP で変える
R, B, WINDOW = 0.1325, float(os.environ.get('WHEEL_SEP', '0.492')), 10


def wheel_angles(d, child):
    """base_link -> child の回転から車輪の回転角（連続）を取り出す."""
    c, s = math.cos(-math.pi / 2), math.sin(-math.pi / 2)
    r_origin = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    out = []
    for _, m in d['/tf']:
        for tr in m.transforms:
            if tr.child_frame_id.endswith(child):
                q = tr.transform.rotation
                x, y, z, w = q.x, q.y, q.z, q.w
                rot = np.array([
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
                m_joint = r_origin.T @ rot
                t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                out.append((t, math.atan2(m_joint[1, 0], m_joint[0, 0])))
    out = sorted(dict(out).items())   # 同じ stamp の重複を除く
    t = np.array([a for a, _ in out])
    return t, np.unwrap(np.array([b for _, b in out]))


def main():
    path = sys.argv[1]
    d = read_bag(path)
    ph = phases_of(path)
    tl, al = wheel_angles(d, 'left_wheel_link')
    tr_, ar = wheel_angles(d, 'right_wheel_link')
    t = np.intersect1d(np.round(tl, 6), np.round(tr_, 6))
    al = np.interp(t, tl, al)
    ar = np.interp(t, tr_, ar)
    dt = np.diff(t)
    # 差分の速度は区間の中点の時刻の値
    wl, wr, tm = np.diff(al) / dt, np.diff(ar) / dt, (t[1:] + t[:-1]) / 2
    v_raw = R * (wr + wl) / 2
    w_raw = R * (wr - wl) / B
    # diff_drive_controller：50 Hz の位置差分の 10 サンプル平均 = 直近 0.2 s の回転角の変化 / 0.2 s（時刻は窓の終わり）
    span = WINDOW * 0.02
    tw_end = t[t >= t[0] + span]
    dl = np.interp(tw_end, t, al) - np.interp(tw_end - span, t, al)
    dr = np.interp(tw_end, t, ar) - np.interp(tw_end - span, t, ar)
    v_ma = R * (dr + dl) / 2 / span
    w_ma = R * (dr - dl) / B / span
    ot, ov, ow = odom_series(d['/wheelchair/odom'])
    tt, tv, tw = truth_velocity(d)
    print(f'wheel samples: {len(tm)}, mean interval {np.mean(dt) * 1000:.1f} ms;'
          f' odom samples: {len(ot)}')

    # ---- 1. 変換の検証：移動平均を掛けた値と /wheelchair/odom ----
    m = (ot > tw_end[0]) & (ot < tw_end[-1])
    dv = ov[m] - np.interp(ot[m], tw_end, v_ma)
    dw = ow[m] - np.interp(ot[m], tw_end, w_ma)
    print('\n[1] diff_drive odom vs my conversion (angle change over the last 0.2 s):')
    print(f'    v: RMS {np.sqrt(np.mean(dv ** 2)):.4f} m/s, max {np.abs(dv).max():.4f} | '
          f'omega: RMS {np.sqrt(np.mean(dw ** 2)):.4f} rad/s, max {np.abs(dw).max():.4f}')

    # ---- 2. 移動平均の前と真値 ----
    print('\n[2] steady values (mean) and delay of half change vs truth [ms]:')
    for name, key, a, b in (('left turn', 'omega', ph['left turn'] + 1.5, ph['right turn'] - 0.2),
                            ('right turn', 'omega', ph['right turn'] + 1.5, ph['forward'] - 0.2),
                            ('forward', 'v', ph['forward'] + 1.0, ph['stop'] - 0.2)):
        mt = (tt > a) & (tt < b)
        mw = (tm > a) & (tm < b)
        mo = (ot > a) & (ot < b)
        truth = (tv if key == 'v' else tw)[mt].mean()
        raw = (v_raw if key == 'v' else w_raw)[mw].mean()
        odo = (ov if key == 'v' else ow)[mo].mean()
        print(f'    {name:<10} {key:<5}: truth {truth:+.3f}, wheels {raw:+.3f} (x{raw / truth:.3f}), '
              f'odom {odo:+.3f} (x{odo / truth:.3f})')
    print(f"    {'':<17}" + ''.join(f'{n:>12}' for n in ('start', 'reverse', 'to fwd(v)', 'to fwd(w)', 'stop')))
    for label, ts, vs, ws in (('wheels (no average)', tm, v_raw, w_raw), ('wheels, 0.2 s avg', tw_end, v_ma, w_ma),
                              ('/wheelchair/odom', ot, ov, ow)):
        cells = []
        for pkey, key in (('left turn', 'omega'), ('right turn', 'omega'), ('forward', 'v'),
                          ('forward', 'omega'), ('stop', 'v')):
            ref = half_time(tt, tv if key == 'v' else tw, ph[pkey])
            got = half_time(ts, vs if key == 'v' else ws, ph[pkey])
            cells.append((got - ref) * 1000)
        print(f'    {label:<17}' + ''.join(f'{c:12.0f}' for c in cells))

    # ---- 3. 実効的な車輪間隔と滑り ----
    print('\n[3] effective wheel separation and wheel slip while turning (wheel speed vs truth at the wheel):')
    for name, a, b in (('left turn', ph['left turn'] + 1.5, ph['right turn'] - 0.2),
                       ('right turn', ph['right turn'] + 1.5, ph['forward'] - 0.2)):
        mw = (tm > a) & (tm < b)
        mt = (tt > a) & (tt < b)
        w_truth = tw[mt].mean()
        diff = (R * (wr - wl))[mw].mean()
        vl, vr = (R * wl)[mw].mean(), (R * wr)[mw].mean()
        print(f'    {name:<10}: b_eff = {diff / w_truth:.3f} m (URDF/controller {B}); '
              f'wheel surface speed L {vl:+.3f} R {vr:+.3f} m/s, '
              f'body speed at wheel (truth) L {-w_truth * B / 2:+.3f} R {w_truth * B / 2:+.3f} m/s')


if __name__ == '__main__':
    main()
