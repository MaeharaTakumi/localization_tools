#!/usr/bin/env python3
"""同じ再生で、MATLAB の CR_debug（data.LIDAR_pose）と rviz2 が点群の stamp で引いた TF を点群ごとに比べる（dev_log.md の 107）.

  python3 rviz/compare_rviz_cr_debug.py <フォルダ> [<点群のフレーム（既定 velodyne）>]

フォルダは matlab/replay_cr_debug_rviz.sh の出力：rviz_tf.log（rviz/tf_hook.cpp の記録）と cr_debug_poses.txt（matlab/check_cr_debug.m の
TEST_POSES）。図は rviz_cr_debug.png。
CR_debug は 20 Hz の周期ごとに最新の点群の stamp で引き直す（引けなければ最新の TF、TF が無ければ姿勢なし）。rviz2 は点群 1 つにつき 2 回引き、
2 回目の値で描く（dev_log.md の 106）。CR_debug の各周期の値を、次の 3 つと比べる：
  - rviz2 が描いた値（同じ点群の 2 回目）
  - 再生の後の決まりの値（前の刻みの最後・後の刻みの最初に届いた値の補間。rviz2 の tf2 に入った TF から計算）
  - 再生の後の最後に届いた値どうしの補間
合わない周期は、rviz2 の tf2 に TF が入っていく途中のどの時点の決まりの値と合うかを探し、CR_debug が引いたときにまだ無かった TF を調べる。
時刻は CLOCK_MONOTONIC（rviz2 のフックの steady_clock と MATLAB の java.lang.System.nanoTime）。
"""
import bisect
import math
import os
import sys
from collections import Counter, defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(os.path.dirname(HERE), 'common')]
from check_rviz_tf_interpolation import (INF, TOL_P, TOL_R, Grid, compose, diff, inverse, parse,  # noqa: E402
                                         static_chain, stats, yaw_deg)
from plot_style import capitalize_labels, SERIES, setup, style, SURFACE, TRUTH_COLOR, TRUTH_LW  # noqa: E402


def euler_zyx_q(roll, pitch, yaw):
    """MATLAB の eul2quat（ZYX）と同じ。四元数は (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy])


def load_cr_debug(path):
    """cr_debug_poses.txt：<前・後の CLOCK_MONOTONIC> <点群の stamp の sec・nanosec> <X Y Z Roll Pitch Yaw Stamp>（姿勢が無ければ nan）."""
    rows = []
    seen = set()
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) != 11:
                continue
            sec, nsec = int(float(p[2])), int(float(p[3]))
            row = dict(m0=int(float(p[0])), m1=int(float(p[1])), t=sec * 10**9 + nsec, has=p[4].lower() != 'nan')
            row['new'] = row['t'] not in seen
            seen.add(row['t'])
            if row['has']:
                x, y, z, roll, pitch, yaw, stamp = (float(v) for v in p[4:11])
                row['v'] = np.concatenate([[x, y, z], euler_zyx_q(roll, pitch, yaw)])
                row['at_cloud'] = abs(stamp - (float(sec) + float(nsec) * 1e-9)) < 1e-6
                row['stamp'] = stamp
            rows.append(row)
    return rows


def close(d):
    return d[0] < TOL_P and d[1] < TOL_R


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__)
        sys.exit(1)
    folder = sys.argv[1]
    lidar = sys.argv[2] if len(sys.argv) == 3 else 'velodyne'
    sets, looks = parse(os.path.join(folder, 'rviz_tf.log'))
    buf = Counter(s['buf'] for s in sets).most_common(1)[0][0]
    sets = [s for s in sets if s['buf'] == buf]
    dyn_all = [s for s in sets if not s['static']]
    (map_frame, base), _ = Counter((s['parent'], s['child']) for s in dyn_all).most_common(1)[0]
    dyn = sorted((s for s in dyn_all if (s['parent'], s['child']) == (map_frame, base) and s['inserted']),
                 key=lambda s: s['seq'])
    statics = {s['child']: (s['parent'], s['v']) for s in sets if s['static']}
    chain = static_chain(statics, base, lidar)
    if chain is None:
        sys.exit(f'no static TF from {base} to {lidar}')
    grid = Grid(dyn)
    walls = [s['wall'] for s in dyn]
    seqs = [s['seq'] for s in dyn]
    seq_index = {s: i for i, s in enumerate(seqs)}

    def seq_at(mono):
        """rviz2 の tf2 にこの時刻までに入った map → base の TF の最後の seq（無ければ 0）."""
        i = bisect.bisect_right(walls, mono) - 1
        return seqs[i] if i >= 0 else 0

    def value(t, n=INF):
        r = grid.rule(t, n)
        return None if r is None else compose(r[1], chain)

    # rviz2 が点群の stamp で引いた値（同じ stamp の 2 回目で描く）
    by_t = defaultdict(list)
    for lk in looks:
        if lk['kind'] == 'L' and lk['buf'] == buf and lk['target'] == map_frame and lk['source'] == lidar and lk['t']:
            by_t[lk['t']].append(lk)
    draw = {}
    for t, ls in by_t.items():
        ls.sort(key=lambda lk: lk['wall'])
        if len(ls) >= 2:
            draw[t] = ls[1]

    rows = load_cr_debug(os.path.join(folder, 'cr_debug_poses.txt'))
    at_cloud = [r for r in rows if r['has'] and r['at_cloud']]
    fallback = [r for r in rows if r['has'] and not r['at_cloud']]
    nopose = [r for r in rows if not r['has']]
    clouds_m = {r['t'] for r in rows}
    clouds_m_pose = {r['t'] for r in at_cloud}
    print(f'rviz2：点群の stamp で引いた点群 {len(by_t)} 個（2 回目を引いて描いた {len(draw)} 個）、'
          f'tf2 に入った {map_frame} → {base} の刻み {len(grid.stamps)} 個')
    print(f'CR_debug：点群のある周期 {len(rows)} 回、点群 {len(clouds_m)} 個'
          f'（点群の stamp で引いた周期 {len(at_cloud)} 回・点群 {len(clouds_m_pose)} 個、最新の TF を使った周期 {len(fallback)} 回、'
          f'姿勢の無い周期 {len(nopose)} 回）')
    both = clouds_m_pose & set(draw)
    print(f'両方が点群の stamp で引いた点群 {len(both)} 個（rviz2 だけ {len(set(draw) - clouds_m)} 個、'
          f'CR_debug だけ {len(clouds_m_pose - set(by_t))} 個）')

    for r in at_cloud:
        t = r['t']
        post = value(t)
        r['post'] = diff(r['v'], post) if post is not None else (np.nan, np.nan)
        ll = grid.last_last(t)
        r['ll'] = diff(r['v'], compose(ll, chain)) if ll is not None else (np.nan, np.nan)
        d = draw.get(t)
        r['rviz'] = diff(r['v'], d['v']) if d is not None else None
        r['vs_draw_ms'] = (r['m1'] - d['wall']) * 1e-6 if d is not None else None
        # CR_debug が引いた間（getSensorData の前から後）に rviz2 の tf2 に入っていた TF での決まりの値と合うか
        n0, n1 = seq_at(r['m0']), seq_at(r['m1'])
        cand = [n0] + [s for s in seqs[seq_index.get(n0, -1) + 1:seq_index.get(n1, -1) + 1]] if n0 else []
        r['live'] = any(value(t, n) is not None and close(diff(r['v'], value(t, n))) for n in cand)
        if not close(r['post']):
            # 合う時点を、点群の stamp の前後 0.2 s の刻みの TF の入った順に探す
            lo = bisect.bisect_left(seqs, grid.arr[grid.stamps[max(0, bisect.bisect_left(grid.stamps, t - 200_000_000))]][0][0])
            hi = bisect.bisect_right(seqs, seq_at(r['m1'] + 1_000_000_000))
            match = [seqs[i] for i in range(lo, hi) if value(t, seqs[i]) is not None
                     and close(diff(r['v'], value(t, seqs[i])))]
            r['match'] = (match[0], match[-1]) if match else None

    def report(sub, label):
        print(f'  {label}：{len(sub)} 回')
        if not sub:
            return
        for key, name in (('rviz', 'rviz2 が描いた値（同じ点群の 2 回目）'), ('post', '再生の後の決まりの値'),
                          ('ll', '再生の後の最後に届いた値どうしの補間')):
            s = [r for r in sub if r[key] is not None]
            p = np.array([r[key][0] for r in s])
            a = np.array([r[key][1] for r in s])
            if not s:
                print(f'    {name}：なし')
                continue
            print(f'    {name}との差（{len(s)} 回）：位置 {stats(p)} m、回転 {stats(a)} deg'
                  f'（ずれる {int(np.sum((p >= TOL_P) | (a >= TOL_R)))} 回、1 mm 以上 {np.mean(p >= 1e-3) * 100:.1f} %・'
                  f'0.1° 以上 {np.mean(a >= 0.1) * 100:.1f} %）')
        dt = np.array([r['vs_draw_ms'] for r in sub if r['vs_draw_ms'] is not None])
        if dt.size:
            print(f'    CR_debug が引き終えた時刻 − rviz2 が描くために引いた時刻：中央値 {np.median(dt):.1f}・最小 {dt.min():.1f}・'
                  f'最大 {dt.max():.1f} ms')

    print()
    print('CR_debug が点群の stamp で引いた値：')
    report([r for r in at_cloud if r['new']], 'その点群で最初の周期')
    report([r for r in at_cloud if not r['new']], '同じ点群を引き直した周期')
    off = [r for r in at_cloud if not close(r['post'])]
    live_ok = sum(1 for r in at_cloud if r['live'])
    print(f'  引いた間（getSensorData の前から後）に rviz2 の tf2 に入っていた TF での決まりの値と合う：{live_ok} / {len(at_cloud)} 回')
    if off:
        print(f'  再生の後の決まりの値と合わない {len(off)} 回：')
        for r in off[:20]:
            b = grid.bracket(r['t'], INF)
            m = r.get('match')
            if m is None or b is None or b[1] is None:
                print(f'    点群 {r["t"] * 1e-9:.3f} s：差 {r["post"][0]:.2e} m・{r["post"][1]:.2e}°、合う時点なし')
                continue
            had = (grid.count(b[0], m[1]), grid.count(b[1], m[1]))
            nxt = seq_index[m[1]] + 1
            lack = (walls[nxt] - r['m1']) * 1e-6 if nxt < len(walls) else float('nan')
            print(f'    点群 {r["t"] * 1e-9:.3f} s（{"最初" if r["new"] else "引き直し"}）：差 {r["post"][0]:.2e} m・{r["post"][1]:.2e}°。'
                  f'前の刻みが {had[0]} 回・後の刻みが {had[1]} 回届いた時点の値と合う'
                  f'（次の TF が rviz2 に入ったのは CR_debug が引き終えた {lack:+.1f} ms 後）')
    if fallback:
        print(f'最新の TF を使った周期 {len(fallback)} 回：')
        for r in fallback:
            # 引いた間（getSensorData の前から後）のどこかで、rviz2 の tf2 の一番新しい刻みの最後に届いた値（tf2 の最新）と合うか
            n0, n1 = seq_at(r['m0']), seq_at(r['m1'])
            cand = [n0] + seqs[seq_index.get(n0, -1) + 1:seq_index.get(n1, -1) + 1]
            best = None
            for n in cand:
                k = grid.newest(n)
                if k is None:
                    continue
                d = diff(r['v'], compose(grid.last(k, n), chain))
                if best is None or d < best[0]:
                    best = (d, k, grid.count(k, n))
            first_t = grid.stamps[0]
            where = '最初の TF の stamp より前' if r['t'] < first_t else '点群の stamp の TF が引けなかった'
            msg = (f'    点群 {r["t"] * 1e-9:.3f} s（{where}）：使った TF の stamp − 点群の stamp {(r["stamp"] - r["t"] * 1e-9) * 1e3:+.1f} ms、'
                   f'引いた {(r["m1"] - r["m0"]) * 1e-6:.0f} ms の間の tf2 の最新（一番新しい刻みの最後に届いた値）との差 ')
            if best is None:
                msg += 'なし（rviz2 にまだ TF が無い）'
            else:
                msg += f'{best[0][0]:.2e} m・{best[0][1]:.2e}°（{(best[1] - r["t"]) * 1e-6:+.0f} ms の刻み、そのとき {best[2]} 回届いていた）'
            post = value(r['t'])
            r['post'] = diff(r['v'], post) if post is not None else None
            if r['post'] is not None:
                msg += f'。点群の stamp の値（再生の後の決まりの値）との差 {r["post"][0]:.3f} m・{r["post"][1]:.2f}°'
            print(msg)
        err = [r['post'] for r in fallback if r['post'] is not None]
        if err:
            print(f'  点群の stamp の値との差（{len(err)} 回）：位置 最大 {max(e[0] for e in err):.3f} m、回転 最大 {max(e[1] for e in err):.2f}°')
    ok = at_cloud and all(r['rviz'] is None or close(r['rviz']) for r in at_cloud) and not off
    print()
    print('→ CR_debug の値は、点群の stamp で引いたすべての周期で rviz2 が描いた値（＝再生の後の決まりの値）と一致' if ok
          else '→ rviz2 が描いた値・再生の後の決まりの値と合わない周期がある')
    out = os.path.join(folder, 'rviz_cr_debug.png')
    plot(grid, chain, at_cloud, fallback, draw, out)
    print(f'図：{out}')


def plot(grid, chain, at_cloud, fallback, draw, fig_path):
    setup()
    u = grid.stamps
    t0 = u[0]
    inv = inverse(chain)
    # 上：再生の後の決まりの値と合わない周期があればその点群、無ければ最後に届いた値どうしの補間との差（回転）が一番大きい点群を挟む 120 ms
    off = [r for r in at_cloud if not close(r['post'])]
    pick = max(off or at_cloud, key=lambda r: np.nan_to_num(r['post'][1] if off else r['ll'][1]))
    kc = bisect.bisect_right(u, pick['t']) - 1
    kc = min(max(kc, 3), len(u) - 4)
    win = u[kc - 3:kc + 4]
    yaw0 = yaw_deg(grid.last(win[0]))

    def rel(v):
        return (yaw_deg(v) - yaw0 + 180) % 360 - 180

    def ms(t):
        return (np.asarray(t, dtype=float) - win[0]) * 1e-6

    fig, axes = plt.subplots(3, 1, figsize=(8.2, 9.6), gridspec_kw=dict(height_ratios=[1.15, 1, 1]))
    run = os.path.basename(os.path.normpath(os.path.dirname(os.path.abspath(fig_path))))
    fig.suptitle(f'TF at point cloud stamp: MATLAB CR_debug vs. RViz2 in one replay ({run})', x=0.01, ha='left',
                 fontweight='bold', fontsize=11)
    ax = axes[0]
    y_last = [rel(grid.last(k)) for k in win]
    y_first = [rel(grid.first(k)) for k in win]
    ax.plot(ms(win), y_last, '-o', color=TRUTH_COLOR, lw=TRUTH_LW, ms=7, zorder=1,
            label='EKF TF stamps: last sent value (linear between)')
    for i in range(len(win) - 1):
        ax.plot(ms(win[i:i + 2]), [y_last[i], y_first[i + 1]], '-', color=SERIES[0], lw=1.2, zorder=2,
                label='Older stamp last value to newer stamp first value' if i == 0 else None)
    ax.plot(ms(win), y_first, 's', color=SERIES[1], ms=6, zorder=3, label='EKF TF stamps: first sent value')
    sel = [r for r in at_cloud if win[0] < r['t'] < win[-1]]
    if sel:
        ax.plot(ms([r['t'] for r in sel]), [rel(compose(r['v'], inv)) for r in sel], 'o', color=SERIES[2], mfc='none',
                mew=1.6, ms=12, zorder=4, label='MATLAB CR_debug LIDAR_pose (every cycle)')
    dsel = [d for t, d in draw.items() if win[0] < t < win[-1]]
    if dsel:
        ax.plot(ms([d['t'] for d in dsel]), [rel(compose(d['v'], inv)) for d in dsel], 'o', color=SERIES[0], ms=5,
                zorder=5, label='RViz2 lookup at point cloud stamp (used to draw)')
    ax.set_xlabel(f'Time from {(win[0] - t0) * 1e-9:.2f} s [ms]')
    ax.set_ylabel('Yaw change [deg]')
    yr = [min(y_last + y_first), max(y_last + y_first)]
    pad = 0.9 * (yr[1] - yr[0])
    if y_last[-1] >= y_last[0]:
        ax.set_ylim(yr[0] - pad, yr[1] + 0.05 * (yr[1] - yr[0]))
        loc = 'lower right'
    else:
        ax.set_ylim(yr[0] - 0.05 * (yr[1] - yr[0]), yr[1] + pad)
        loc = 'upper right'
    ax.legend(loc=loc, frameon=False, fontsize=8)
    style(ax, grid_axis='both')

    # 中・下：CR_debug の値と 3 つの値の差（点群の stamp で引いたすべての周期）。最新の TF を使った周期は点群の stamp の値との差
    floor = 1e-12
    series = [('ll', 'CR_debug vs. last values of both stamps (after replay)', SERIES[2], 40),
              ('post', 'CR_debug vs. older last / newer first (after replay)', SERIES[1], 18),
              ('rviz', 'CR_debug vs. RViz2 value used to draw the same point cloud', SERIES[0], 5)]
    tt = (np.array([r['t'] for r in at_cloud], dtype=float) - t0) * 1e-9
    for p, (ax, name) in enumerate(zip(axes[1:], ['Rotation difference [deg]', 'Position difference [m]'])):
        idx = 1 if p == 0 else 0
        for key, label, color, size in series:
            e = np.array([r[key][idx] if r[key] is not None else np.nan for r in at_cloud])
            ax.scatter(tt, np.maximum(e, floor), s=size, color=color, lw=0, label=label)
        fb = [r for r in fallback if r.get('post') is not None]
        if fb:
            ax.scatter((np.array([r['t'] for r in fb], dtype=float) - t0) * 1e-9, [max(r['post'][idx], floor) for r in fb],
                       s=18, color=SERIES[3], lw=0, label='CR_debug with the latest TF (none at cloud stamp yet) vs. older last / newer first')
        ax.set_yscale('log')
        ax.set_ylim(floor / 3, 300)
        ax.set_ylabel(name)
        ax.text(0.01, 0.12, 'Differences below 1e-12 are drawn at 1e-12', transform=ax.transAxes, fontsize=8, color='#52514e')
        style(ax, grid_axis='both')
    axes[2].set_xlabel('Time from first EKF TF [s]')
    handles, labels = axes[2].get_legend_handles_labels()
    for h in handles:
        h.set_sizes([30])
    fig.legend(handles, labels, loc='lower center', ncol=1, frameon=False, fontsize=8)
    fig.tight_layout(rect=(0, 0.07, 1, 0.97))
    capitalize_labels(fig)
    fig.savefig(fig_path, dpi=130, facecolor=SURFACE)


if __name__ == '__main__':
    main()
