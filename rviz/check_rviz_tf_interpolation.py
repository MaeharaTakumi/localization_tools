#!/usr/bin/env python3
"""rviz/tf_hook.cpp の記録（rviz2 の tf2 に入った TF と、rviz2 が引いた TF）を、tf2 の補間の決まりと比べる（dev_log.md の 106）.

  python3 rviz/check_rviz_tf_interpolation.py <rviz_tf.log> <図の .png> [<点群のフレーム（既定 velodyne）>]

tf2 0.25（Humble）の TimeCache は同じ stamp を置き換えずに、先に入った値の手前に重ねる。そのため時刻 t で引くと：
  - t が刻みの間：前の刻みの最後に届いた値と、後の刻みの最初に届いた値を lerp + slerp（以下「決まり」）
  - t が刻みちょうど：最後に届いた値（一番古い刻みだけ最初の値）
  - t = 0（最新）：一番新しい刻みの最後に届いた値
rviz2 が引いた値を、引いたときに入っていた TF（引く前の seq まで。合わなければ引いた後の seq まで）から計算した決まりの値と比べる。
点群の stamp で引いたもの（map → 点群のフレーム）は、再生の後の値（後から届いた分も入れた決まりの値と、最後に届いた値どうしの補間）とも比べる。
rviz2 は点群 1 つにつき同じ stamp で 2 回引く：1 回目は tf2_ros::MessageFilter の waitForTransform（TF が揃ったときに tf2_ros::Buffer が引いて
future に入れる。値は使わない）、2 回目はメインのスレッドで点群を処理するとき（FrameManager::getTransform。この値で描く）。図は 2 回目。
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

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'common')]
from plot_style import capitalize_labels, SERIES, setup, style, SURFACE, TRUTH_COLOR, TRUTH_LW  # noqa: E402

TOL_P, TOL_R = 1e-6, 1e-4    # 一致とみなす差 [m]・[deg]（105 と同じ）
LATE = 3                     # 引いた後の seq より先に入っていたとみてよい TF の数（check_live）
INF = float('inf')


# ---- 四元数（x, y, z, w）と姿勢 [x, y, z, qx, qy, qz, qw] ----
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def qrot(q, v):
    return qmul(qmul(q, np.array([v[0], v[1], v[2], 0.0])), qconj(q))[:3]


def compose(a, b):
    """a（親 → 中）と b（中 → 子）から親 → 子."""
    return np.concatenate([a[:3] + qrot(a[3:], b[:3]), qmul(a[3:], b[3:])])


def inverse(a):
    qi = qconj(a[3:]) / np.dot(a[3:], a[3:])
    return np.concatenate([-qrot(qi, a[:3]), qi])


def slerp(q1, q2, t):
    """tf2::Quaternion::slerp と同じ式（内積が負なら q2 の符号を反転）."""
    d = float(np.dot(q1, q2))
    s = math.sqrt(np.dot(q1, q1) * np.dot(q2, q2))
    theta = math.acos(min(1.0, max(-1.0, abs(d) / s)))
    if theta == 0.0:
        return q1
    k = 1.0 / math.sin(theta)
    s0, s1 = math.sin((1.0 - t) * theta), math.sin(t * theta)
    return (q1 * s0 + (-q2 if d < 0 else q2) * s1) * k


def interpolate(one, two, s1, s2, t):
    """tf2 の TimeCache::interpolate（比は整数の ns の差の比、位置は (1 − r)·one + r·two）."""
    if s1 == s2:
        return two
    r = (t - s1) / (s2 - s1)
    return np.concatenate([(1.0 - r) * one[:3] + r * two[:3], slerp(one[3:], two[3:], r)])


def angle_deg(q1, q2):
    q = qmul(qconj(q1), q2)
    return math.degrees(2.0 * math.atan2(np.linalg.norm(q[:3]), abs(q[3])))


def yaw_deg(v):
    x, y, z, w = v[3:]
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def diff(a, b):
    return float(np.linalg.norm(a[:3] - b[:3])), angle_deg(a[3:], b[3:])


# ---- 記録 ----
def parse(path):
    sets, looks = [], []
    with open(path) as f:
        for line in f:
            p = line.split()
            if not p:
                continue
            if p[0] == 'S' and len(p) == 16:
                sets.append(dict(seq=int(p[1]), wall=int(p[2]), buf=p[3], stamp=int(p[4]), parent=p[5], child=p[6],
                                 static=p[7] == '1', inserted=p[8] == '1', v=np.array([float(x) for x in p[9:16]])))
            elif p[0] == 'L' and len(p) == 16:
                looks.append(dict(kind='L', before=int(p[1]), after=int(p[2]), wall=int(p[3]), buf=p[4], target=p[5], source=p[6],
                                  t=int(p[7]), stamp=int(p[8]), v=np.array([float(x) for x in p[9:16]])))
            elif p[0] == 'E' and len(p) == 8:
                looks.append(dict(kind='E', before=int(p[1]), after=int(p[2]), buf=p[4], target=p[5], source=p[6],
                                  t=int(p[7])))
    return sets, looks


class Grid:
    """map → 基準のフレームの TF を刻み（stamp）ごとに、届いた順に持つ."""

    def __init__(self, dyn):
        self.arr = defaultdict(list)          # stamp → [(seq, 値)]（届いた順）
        for s in dyn:
            self.arr[s['stamp']].append((s['seq'], s['v']))
        self.stamps = sorted(self.arr)
        self.first_seq = [self.arr[k][0][0] for k in self.stamps]

    def present(self, i, n):
        return self.first_seq[i] <= n

    def last(self, k, n=INF):
        v = None
        for seq, val in self.arr[k]:
            if seq > n:
                break
            v = val
        return v

    def first(self, k):
        return self.arr[k][0][1]

    def count(self, k, n=INF):
        return sum(1 for seq, _ in self.arr[k] if seq <= n)

    def bracket(self, t, n):
        """(前の刻み, 後の刻み)。t ちょうどなら (t, None)。範囲の外は None."""
        j = bisect.bisect_right(self.stamps, t) - 1
        while j >= 0 and not self.present(j, n):
            j -= 1
        if j < 0:
            return None
        if self.stamps[j] == t:
            return t, None
        i = j + 1
        while i < len(self.stamps) and not self.present(i, n):
            i += 1
        if i == len(self.stamps):
            return None
        return self.stamps[j], self.stamps[i]

    def newest(self, n):
        i = len(self.stamps) - 1
        while i >= 0 and not self.present(i, n):
            i -= 1
        return self.stamps[i] if i >= 0 else None

    def rule(self, t, n=INF):
        """時刻 t（0 は最新）で引いたときの値（n までの seq の TF が入っているとき）。(stamp, 値) か None."""
        if t == 0:
            k = self.newest(n)
            return None if k is None else (k, self.last(k, n))
        b = self.bracket(t, n)
        if b is None:
            return None
        if b[1] is None:
            return t, self.last(t, n)
        return t, interpolate(self.last(b[0], n), self.first(b[1]), b[0], b[1], t)

    def last_last(self, t):
        b = self.bracket(t, INF)
        if b is None:
            return None
        if b[1] is None:
            return self.last(t)
        return interpolate(self.last(b[0]), self.last(b[1]), b[0], b[1], t)


def static_chain(statics, base, frame):
    """基準のフレーム → frame の静的な TF（たどれなければ None）."""
    chain = []
    f = frame
    while f != base:
        if f not in statics:
            return None
        parent, v = statics[f]
        chain.append(v)
        f = parent
    out = np.array([0, 0, 0, 0, 0, 0, 1.0])
    for v in reversed(chain):
        out = compose(out, v)
    return out


def late_note(rows):
    """引く前の seq では合わず、引いた後の seq やその先まで入っていたとみると合った回数."""
    after = sum(1 for r in rows if r['live'] and r['live'][3] == r['after'] != r['before'])
    late = sum(1 for r in rows if r['live'] and r['live'][3] > r['after'])
    note = ''
    if after:
        note += f'、引いた後の seq まで入っていたとみて合う {after} 回'
    if late:
        note += f'、引いた後の seq より先の TF まで入っていたとみて合う {late} 回'
    return note


def stats(x):
    x = np.asarray(x)
    if x.size == 0:
        return 'なし'
    return f'中央値 {np.median(x):.2e}・95 % {np.percentile(x, 95):.2e}・最大 {x.max():.2e}'


def main():
    if len(sys.argv) not in (3, 4):
        print(__doc__)
        sys.exit(1)
    log_path, fig_path = sys.argv[1:3]
    lidar = sys.argv[3] if len(sys.argv) == 4 else 'velodyne'
    sets, looks = parse(log_path)
    # TF を一番多く受け取ったバッファ（rviz2 の FrameManager の tf2_ros::Buffer）
    buf = Counter(s['buf'] for s in sets).most_common(1)[0][0]
    sets = [s for s in sets if s['buf'] == buf]
    looks = [lk for lk in looks if lk['buf'] == buf]
    dyn_all = [s for s in sets if not s['static']]
    (map_frame, base), _ = Counter((s['parent'], s['child']) for s in dyn_all).most_common(1)[0]
    dyn = [s for s in dyn_all if (s['parent'], s['child']) == (map_frame, base) and s['inserted']]
    statics = {s['child']: (s['parent'], s['v']) for s in sets if s['static']}
    grid = Grid(dyn)
    T = int(np.median(np.diff(grid.stamps)))
    sends = Counter(len(grid.arr[k]) for k in grid.stamps)
    print(f'rviz2 の tf2 に入った TF：{map_frame} → {base}、刻み {len(grid.stamps)} 個（{T * 1e-6:.1f} ms 間隔、'
          f'{(grid.stamps[-1] - grid.stamps[0]) * 1e-9:.1f} s）、受け取った TF {len(dyn)} 個'
          f'（入らなかった {sum(1 for s in dyn_all if not s["inserted"])} 個）')
    print('  1 刻みが届いた回数：' + '、'.join(f'{c} 回 {n} 個' for c, n in sorted(sends.items())))
    print(f'  静的な TF：{", ".join(f"{p} → {c}" for c, (p, _) in statics.items())}')

    chains = {}
    at_stamp, latest, other = [], [], Counter()
    for lk in looks:
        key = (lk['target'], lk['source'], 'stamp' if lk['t'] else 'latest')
        if lk['kind'] == 'E':
            other[key + ('例外',)] += 1
            continue
        if lk['target'] != map_frame or lk['source'] == map_frame:
            other[key] += 1
            continue
        if lk['source'] not in chains:
            chains[lk['source']] = static_chain(statics, base, lk['source'])
        if chains[lk['source']] is None:
            other[key] += 1
            continue
        (at_stamp if lk['t'] else latest).append(lk)
    print(f'rviz2 が引いた TF：{len(looks)} 回')
    for (target, source, kind), n in sorted(Counter((lk['target'], lk['source'], 'stamp' if lk['t'] else 'latest')
                                                    for lk in at_stamp + latest).items()):
        print(f'  {target} → {source}（{"時刻を指定" if kind == "stamp" else "最新（時刻 0）"}）：{n} 回')
    for key, n in sorted(other.items()):
        print(f'  比べない：{key[0]} → {key[1]}（{"時刻を指定" if key[2] == "stamp" else "最新"}{"、" + key[3] if len(key) > 3 else ""}）：{n} 回')

    def expected(lk, n):
        r = grid.rule(lk['t'], n)
        if r is None:
            return None
        return r[0], compose(r[1], chains[lk['source']])

    def check_live(lk):
        """引いたときに入っていた TF での決まりの値との差。引く前の seq で合わなければ引いた後の seq（とその先の LATE 個）で.

        フックは tf2 に入れ終わってから seq を増やすので、その間に引くと、引いた後の seq より先の TF が既に入っている。
        """
        best = None
        for n in (lk['before'], *range(lk['after'], lk['after'] + LATE + 1)):
            e = expected(lk, n)
            if e is None:
                continue
            dp, dr = diff(lk['v'], e[1])
            res = (dp, dr, e[0] == lk['stamp'], n)
            if best is None or (dp, dr) < best[:2]:
                best = res
            if dp < TOL_P and dr < TOL_R and e[0] == lk['stamp']:
                return res
        return best

    # 時刻を指定して引いたもの（点群の stamp）。同じ stamp の 1 回目は tf2_ros::MessageFilter の waitForTransform（TF が揃ったときに
    # tf2_ros::Buffer が引いて future に入れる。値は使わない）、2 回目は rviz2 がメインのスレッドで点群を処理するときに引いて描画に使う値
    rows = []
    for lk in at_stamp:
        live = check_live(lk)
        post = expected(lk, INF)
        ll = grid.last_last(lk['t'])
        b = grid.bracket(lk['t'], lk['before'])
        row = dict(t=lk['t'], source=lk['source'], v=lk['v'], live=live, before=lk['before'], after=lk['after'],
                   wall=lk['wall'], post=diff(lk['v'], post[1]) if post else (np.nan, np.nan),
                   ll=diff(lk['v'], compose(ll, chains[lk['source']])) if ll is not None else (np.nan, np.nan),
                   lead=(grid.newest(lk['before']) - lk['t']) * 1e-6)
        if b is not None and b[1] is not None:
            row['counts'] = (grid.count(b[0], lk['before']), grid.count(b[1], lk['before']),
                             len(grid.arr[b[0]]), len(grid.arr[b[1]]))
        rows.append(row)
    by_t = defaultdict(list)
    for r in rows:
        by_t[(r['source'], r['t'])].append(r)
    for rs in by_t.values():     # 同じ stamp の点群が 2 つあれば 1・2 回目、3・4 回目の組
        rs.sort(key=lambda r: r['wall'])
        for i, r in enumerate(rs):
            r['role'] = 'draw' if i % 2 == 1 else 'filter'
            if i % 2 == 1:
                r['gap'] = (r['wall'] - rs[i - 1]['wall']) * 1e-6
                r['changed'] = r['before'] != rs[i - 1]['before']
                r['vs_filter'] = diff(r['v'], rs[i - 1]['v'])
    print()
    print(f'時刻を指定して引いた TF（{len(rows)} 回。点群の stamp {len(by_t)} 個）：')
    if rows:
        live_p = [r['live'][0] for r in rows if r['live']]
        live_r = [r['live'][1] for r in rows if r['live']]
        bad = sum(1 for r in rows if not r['live'] or r['live'][0] >= TOL_P or r['live'][1] >= TOL_R or not r['live'][2])
        print(f'  引いたときに入っていた TF での決まりの値との差：位置 最大 {max(live_p):.2e} m、回転 最大 {max(live_r):.2e} deg'
              f'（ずれる {bad} 回{late_note(rows)}）')
        for role, label in (('draw', '2 回目（rviz2 が点群を処理するとき。描画に使う値）'),
                            ('filter', '1 回目（MessageFilter の waitForTransform。値は使わない）')):
            sub = [r for r in rows if r['role'] == role]
            print(f'  {label}：{len(sub)} 回')
            if not sub:
                continue
            for key, name in (('post', '再生の後の決まりの値（後から届いた分も入れる）'), ('ll', '再生の後の最後に届いた値どうしの補間')):
                p = np.array([r[key][0] for r in sub])
                a = np.array([r[key][1] for r in sub])
                print(f'    {name}との差：')
                print(f'      位置 {stats(p)} m（1 mm 以上 {np.mean(p >= 1e-3) * 100:.1f} %）')
                print(f'      回転 {stats(a)} deg（0.1° 以上 {np.mean(a >= 0.1) * 100:.1f} %、ずれる {int(np.sum((p >= TOL_P) | (a >= TOL_R)))} 回）')
            cnt = Counter(r['counts'][:2] for r in sub if 'counts' in r)
            print('    引いたときに届いていた回数（前の刻み・後の刻み）：' + '、'.join(f'{a}・{b} 回が {n} 個' for (a, b), n in cnt.most_common()))
            lead = np.array([r['lead'] for r in sub])
            print(f'    引いたときの一番新しい刻み − 点群の stamp：中央値 {np.median(lead):.1f}・最小 {lead.min():.1f}・最大 {lead.max():.1f} ms')
        pairs = [r for r in rows if r['role'] == 'draw']
        if pairs:
            gap = np.array([r['gap'] for r in pairs])
            changed = [r for r in pairs if r['changed']]
            print(f'  1 回目から 2 回目まで：中央値 {np.median(gap):.1f}・最大 {gap.max():.1f} ms。その間に TF が入った組 {len(changed)} 個'
                  + (f'（2 回の値の差 最大 位置 {max(r["vs_filter"][0] for r in changed):.2e} m、'
                     f'回転 {max(r["vs_filter"][1] for r in changed):.2e} deg）' if changed else ''))

    # 最新（時刻 0）で引いたもの（視点の Target Frame・TF の表示）
    lrows = []
    for lk in latest:
        live = check_live(lk)
        k = lk['stamp']
        final = grid.last(k) if k in grid.arr else None
        lrows.append(dict(stamp=k, source=lk['source'], v=lk['v'], live=live, before=lk['before'], after=lk['after'],
                          count=grid.count(k, lk['before']) if k in grid.arr else 0,
                          final=diff(lk['v'], compose(final, chains[lk['source']])) if final is not None else (np.nan, np.nan)))
    print()
    print(f'最新（時刻 0）で引いた TF（{len(lrows)} 回。視点の Target Frame と TF の表示）：')
    if lrows:
        bad = sum(1 for r in lrows if not r['live'] or r['live'][0] >= TOL_P or r['live'][1] >= TOL_R or not r['live'][2])
        print(f'  引いたときの一番新しい刻みの最後に届いた値との差：位置 最大 {max(r["live"][0] for r in lrows if r["live"]):.2e} m、'
              f'回転 最大 {max(r["live"][1] for r in lrows if r["live"]):.2e} deg（ずれる {bad} 回{late_note(lrows)}）')
        cnt = Counter(r['count'] for r in lrows)
        print('  その刻みが引いたときに届いていた回数：' + '、'.join(f'{c} 回 {n} 個' for c, n in sorted(cnt.items())))
        p = np.array([r['final'][0] for r in lrows])
        a = np.array([r['final'][1] for r in lrows])
        print(f'  その刻みの最後に届いた値（再生の後）との差：位置 {stats(p)} m、回転 {stats(a)} deg')

    ok = rows and all(r['live'] and r['live'][0] < TOL_P and r['live'][1] < TOL_R and r['live'][2] for r in rows) \
        and all(r['live'] and r['live'][0] < TOL_P and r['live'][1] < TOL_R and r['live'][2] for r in lrows)
    print()
    print('→ rviz2 の引いた値はすべて tf2 の決まり（前の刻みは最後・後の刻みは最初の値の補間、最新は一番新しい刻みの最後の値）と一致'
          if ok else '→ 決まりと合わない引きがある')
    plot(grid, rows, lrows, chains, lidar, fig_path)
    print(f'図：{fig_path}')


def plot(grid, rows, lrows, chains, lidar, fig_path):
    setup()
    u = grid.stamps
    t0 = u[0]
    clouds = [r for r in rows if r['source'] == lidar and r['role'] == 'draw']   # 描画に使う値（2 回目）
    cloud_t = np.array([r['t'] for r in clouds])
    # 上：最後に届いた値どうしの補間との差（回転）が一番大きい点群の stamp を挟む 120 ms
    if clouds:
        tc = clouds[int(np.nanargmax([r['ll'][1] for r in clouds]))]['t']
        kc = bisect.bisect_right(u, tc) - 1
    else:
        kc = int(np.argmax([angle_deg(grid.first(k)[3:], grid.last(k)[3:]) for k in u]))
    kc = min(max(kc, 3), len(u) - 4)
    win = u[kc - 3:kc + 4]
    yaw0 = yaw_deg(grid.last(win[0]))

    def rel(v):
        return (yaw_deg(v) - yaw0 + 180) % 360 - 180

    def ms(t):
        return (np.asarray(t, dtype=float) - win[0]) * 1e-6

    fig, axes = plt.subplots(3, 1, figsize=(8.2, 9.6), gridspec_kw=dict(height_ratios=[1.15, 1, 1]))
    run = os.path.basename(os.path.dirname(os.path.abspath(fig_path)))
    fig.suptitle(f'TF from ekf_localizer looked up inside RViz2 (tf2 hooked with LD_PRELOAD, {run})', x=0.01, ha='left',
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
    inv = {s: inverse(c) for s, c in chains.items() if c is not None}
    sel = [r for r in clouds if win[0] < r['t'] < win[-1]]
    if sel:
        ax.plot(ms([r['t'] for r in sel]), [rel(compose(r['v'], inv[r['source']])) for r in sel], 'o', color=SERIES[0],
                ms=6, zorder=4, label='RViz2 lookup at point cloud stamp (used to draw)')
    seen = {}
    for r in lrows:
        if win[0] <= r['stamp'] <= win[-1]:
            seen.setdefault((r['stamp'], round(rel(compose(r['v'], inv[r['source']])), 9)), None)
    if seen:
        ax.plot(ms([k for k, _ in seen]), [y for _, y in seen], 'D', color=SERIES[2], mfc='none', mew=1.4, ms=9, zorder=5,
                label='RViz2 lookup of latest (time 0)')
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

    # 中・下：点群の stamp で rviz2 が引いた値と 3 つの値の差
    floor = 1e-12
    series = [('ll', 'Vs. last values of both stamps (after replay)', SERIES[2], 40),
              ('post', 'Vs. older last / newer first (after replay)', SERIES[1], 18),
              ('live', 'Vs. older last / newer first (TF in RViz2 at the lookup)', SERIES[0], 5)]
    tt = (cloud_t - t0) * 1e-9
    for p, (ax, name) in enumerate(zip(axes[1:], ['Rotation difference [deg]', 'Position difference [m]'])):
        for key, label, color, size in series:
            idx = 1 if p == 0 else 0
            e = np.array([r[key][idx] if r[key] else np.nan for r in clouds])
            ax.scatter(tt, np.maximum(e, floor), s=size, color=color, lw=0, label=label)
        ax.set_yscale('log')
        ax.set_ylim(floor / 3, 10)
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
