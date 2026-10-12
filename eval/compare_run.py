#!/usr/bin/env python3
"""推定器の比較：録画を再生して推定器を動かし、どの動きにも使える指標を出す（localizer_rl 内で実行）.

  python3 compare_run.py run <入力 bag> <試行名> <estimator> ['<parameters の JSON>']   再生して指標（1 行 JSON）
  python3 compare_run.py eval <出力 bag> [<姿勢のトピック>]                              再生済みの bag の指標だけ
  python3 compare_run.py batch <設定の JSON> <結果の jsonl>                              まとめて再生
  python3 compare_run.py reeval <結果の jsonl>                                           残っている bag から指標を計算し直す
  python3 compare_run.py table <結果の jsonl>                                            動き・推定器ごとの表（RANGE=1 で幅も）
    設定の JSON: [{"motion": "accel", "bag": "/root/bags/drive_accel", "estimator": "ekf_localizer", "runs": 4,
                  "params": {...}, "label": "..."}, ...]（試行名は <motion>_<label>_<回>。label の既定は estimator）

再生は tune_run.py の replay（最初の動作の 8 秒前から最後の動作の 4 秒後まで、ROS_DOMAIN_ID=99）。
評価の区間は最初の動作から最後の動作（stop）の 2 秒後まで。真値の LiDAR は /wheelchair/pose（base_footprint）に URDF の取付。

  stamp の指標（推定そのものの精度）：/ekf_pose の stamp の時刻の真値との差
  rt_ の指標（実時間。MATLAB が使う値）：100 Hz の各時刻に、その時点で受信済みの最新の /ekf_pose と、その時刻の真値との差
  xy_raw は真値との差そのもの。xy は開始位置で測った地図と Gazebo の座標のずれ（NDT − 真値 = x +41 mm, y −21 mm）を引いたもの。
  このずれは場所で変わる（直進 16 m で x +40 → 0 mm。止まっていても同じで、どの推定器でも出る）ので、xy は開始位置の近くで
  動く録画（その場の回転・13 か所で止まる）にだけ使い、長く走る録画は xy_raw で比べる
  trans_yaw_max：各動作の切り替え（指令が変わる瞬間）の 0.1 s 前から 1 s 後までの yaw 誤差の最大の、切り替え全体の平均
  latency：受信時刻 − stamp、age：各時刻で使える最新の推定の stamp からの経過時間
単位は m・deg・ms。
"""
import json
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'noise')]   # plot_spin などを import する
from plot_spin import read_bag, URDF_MOUNT, wrap  # noqa: E402
from robot_localization_adapter import matrix_to_vec, pose_msg_to_matrix, vec_to_matrix  # noqa: E402

MAP_OFFSET = np.array([0.041, -0.021])   # NDT − 真値（地図と Gazebo の座標のずれ）[m]
RT_RATE = 100.0
TAIL_EVAL = 2.0
PRE, POST = 0.1, 1.0


def phases_of(bag):
    out = {}
    with open(bag.rstrip('/') + '_phases.txt') as f:
        for line in f:
            name, t = line.rsplit(' ', 1)
            out[name] = float(t)
    return out


def load(bag, topic='/ekf_pose'):
    """真値（sim time、base の 6 成分）と推定（stamp・受信の sim time・LiDAR の 6 成分）."""
    d = read_bag(bag, ['/clock', '/wheelchair/pose', topic])
    clk = np.array([(t, m.clock.sec + m.clock.nanosec * 1e-9) for t, m in d['/clock']])

    def to_sim(ts):
        return np.interp(ts, clk[:, 0], clk[:, 1])

    tt = to_sim([t for t, _ in d['/wheelchair/pose']])
    tv = np.array([matrix_to_vec(pose_msg_to_matrix(m)) for _, m in d['/wheelchair/pose']])
    o = np.argsort(tt)
    tt, tv = tt[o], tv[o]
    tv[:, 3:] = np.unwrap(tv[:, 3:], axis=0)
    est = d.get(topic, [])
    stamp = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in est])
    recv = to_sim([t for t, _ in est])
    ev = np.array([matrix_to_vec(pose_msg_to_matrix(m.pose.pose)) for _, m in est])
    return tt, tv, stamp, recv, ev


def truth_lidar(tt, tv, ts):
    """時刻 ts の真値の LiDAR の 6 成分."""
    out = []
    for t in ts:
        base = vec_to_matrix([np.interp(t, tt, tv[:, i]) for i in range(6)])
        out.append(matrix_to_vec(base @ URDF_MOUNT))
    return np.array(out)


def errors(ev, ref):
    """[xy, xy_raw, z, roll, pitch, yaw]（xy は m、角度は deg、z・角度は符号付き）."""
    e = ev - ref
    xy_raw = np.hypot(e[:, 0], e[:, 1])
    xy = np.hypot(e[:, 0] - MAP_OFFSET[0], e[:, 1] - MAP_OFFSET[1])
    ang = np.degrees(wrap(e[:, 3:]))
    return np.column_stack([xy, xy_raw, e[:, 2], ang])


def evaluate(bag, topic='/ekf_pose'):
    phases = phases_of(bag)
    t0, t1 = min(phases.values()), max(phases.values()) + TAIL_EVAL
    tt, tv, stamp, recv, ev = load(bag, topic)
    out = {}

    # stamp の時刻の誤差
    m = (stamp >= t0) & (stamp <= t1)
    e = errors(ev[m], truth_lidar(tt, tv, stamp[m]))
    ts = stamp[m]
    for prefix, err in (('', e),):
        out[prefix + 'xy_mean'] = float(err[:, 0].mean())
        out[prefix + 'xy_max'] = float(err[:, 0].max())
        out[prefix + 'xy_raw_mean'] = float(err[:, 1].mean())
        out[prefix + 'xy_raw_max'] = float(err[:, 1].max())
        out[prefix + 'z_mean'] = float(np.abs(err[:, 2]).mean())
        out[prefix + 'z_max'] = float(np.abs(err[:, 2]).max())
        out[prefix + 'tilt_mean'] = float(np.abs(err[:, 3:5]).mean())
        out[prefix + 'tilt_max'] = float(np.abs(err[:, 3:5]).max())
        out[prefix + 'yaw_mean'] = float(np.abs(err[:, 5]).mean())
        out[prefix + 'yaw_max'] = float(np.abs(err[:, 5]).max())
    trans = []
    for tp in phases.values():
        w = (ts >= tp - PRE) & (ts <= tp + POST)
        if w.any():
            trans.append(float(np.abs(e[w, 5]).max()))
    out['trans_yaw_max'] = float(np.mean(trans))

    # 実時間の誤差：各時刻で受信済みの最新の推定
    o = np.argsort(recv)
    recv_s, stamp_s, ev_s = recv[o], stamp[o], ev[o]
    grid = np.arange(t0, t1, 1.0 / RT_RATE)
    idx = np.searchsorted(recv_s, grid, side='right') - 1
    ok = idx >= 0
    grid, idx = grid[ok], idx[ok]
    rt = errors(ev_s[idx], truth_lidar(tt, tv, grid))
    out['rt_xy_mean'] = float(rt[:, 0].mean())
    out['rt_xy_max'] = float(rt[:, 0].max())
    out['rt_xy_raw_mean'] = float(rt[:, 1].mean())
    out['rt_xy_raw_max'] = float(rt[:, 1].max())
    out['rt_z_mean'] = float(np.abs(rt[:, 2]).mean())
    out['rt_tilt_mean'] = float(np.abs(rt[:, 3:5]).mean())
    out['rt_yaw_mean'] = float(np.abs(rt[:, 5]).mean())
    out['rt_yaw_max'] = float(np.abs(rt[:, 5]).max())
    trans = []
    for tp in phases.values():
        w = (grid >= tp - PRE) & (grid <= tp + POST)
        if w.any():
            trans.append(float(np.abs(rt[w, 5]).max()))
    out['rt_trans_yaw_max'] = float(np.mean(trans))
    age = (grid - stamp_s[idx]) * 1e3
    out['age_mean'] = float(age.mean())
    out['age_max'] = float(age.max())
    lat = (recv[m] - stamp[m]) * 1e3
    out['latency_mean'] = float(lat.mean())
    out['latency_max'] = float(lat.max())
    out['rate'] = float(m.sum() / (t1 - t0))
    return out


def series(bag, topic='/ekf_pose'):
    """グラフ用：stamp の時刻の誤差と実時間の誤差の時系列（時刻は最初の動作から）."""
    phases = phases_of(bag)
    t0, t1 = min(phases.values()), max(phases.values()) + TAIL_EVAL
    tt, tv, stamp, recv, ev = load(bag, topic)
    m = (stamp >= t0 - 3.0) & (stamp <= t1)
    e = errors(ev[m], truth_lidar(tt, tv, stamp[m]))
    o = np.argsort(recv)
    grid = np.arange(t0 - 3.0, t1, 1.0 / RT_RATE)
    idx = np.searchsorted(recv[o], grid, side='right') - 1
    ok = idx >= 0
    rt = errors(ev[o][idx[ok]], truth_lidar(tt, tv, grid[ok]))
    return {'t': stamp[m] - t0, 'err': e, 'rt_t': grid[ok] - t0, 'rt': rt,
            'phases': {k: v - t0 for k, v in phases.items()}, 'truth_t': tt - t0, 'truth': tv}


def run(in_bag, name, estimator, params):
    from tune_run import replay
    out_bag = replay(in_bag, name, estimator, params)
    return {'name': name, 'estimator': estimator, **evaluate(out_bag)}


def main():
    cmd = sys.argv[1]
    if cmd == 'run':
        in_bag, name, estimator = sys.argv[2:5]
        params = json.loads(sys.argv[5]) if len(sys.argv) > 5 else {}
        print(json.dumps(run(in_bag, name, estimator, params)))
    elif cmd == 'eval':
        print(json.dumps(evaluate(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else '/ekf_pose')))
    elif cmd == 'batch':
        configs_path, results_path = sys.argv[2:4]
        with open(configs_path) as f:
            configs = json.load(f)
        # 回ごとに推定器を交互に流す（時間とともに変わる条件があっても片方に偏らない）
        jobs = []
        for i in range(max(c.get('runs', 1) for c in configs)):
            for c in configs:
                if i < c.get('runs', 1):
                    jobs.append((c, i + 1))
        for c, i in jobs:
            label = c.get('label', c['estimator'])
            name = f"{c['motion']}_{label}_{i}"
            p = subprocess.run(['python3', os.path.abspath(__file__), 'run', c['bag'], name, c['estimator'],
                                json.dumps(c.get('params', {}))], capture_output=True, text=True)
            last = [line for line in p.stdout.splitlines() if line.startswith('{')]
            result = json.loads(last[-1]) if last else {'name': name, 'error': p.stderr[-800:]}
            result['motion'] = c['motion']
            result['label'] = label
            with open(results_path, 'a') as f:
                f.write(json.dumps(result) + '\n')
            print(json.dumps(result), flush=True)
    elif cmd == 'reeval':
        # 結果の jsonl の各試行を、残っている出力の bag（/root/bags/tune/<試行名>）から計算し直す（指標を変えたとき）
        path = sys.argv[2]
        rows = [json.loads(line) for line in open(path) if line.strip()]
        with open(path, 'w') as f:
            for r in rows:
                if 'error' not in r:
                    r.update(evaluate(f"/root/bags/tune/{r['name']}"))
                f.write(json.dumps(r) + '\n')
        print(f're-evaluated {len(rows)} runs')
    elif cmd == 'table':
        table(sys.argv[2])
    else:
        raise SystemExit(__doc__)


TABLE_KEYS = [('xy_raw_mean', 1e3, 'xy mm'), ('xy_raw_max', 1e3, 'xy max'), ('xy_mean', 1e3, 'xy（開始のずれ除く）'), ('yaw_mean', 1, 'yaw °'),
              ('yaw_max', 1, 'yaw max'), ('trans_yaw_max', 1, '切替 yaw'), ('z_mean', 1e3, 'z mm'),
              ('z_max', 1e3, 'z max'), ('tilt_mean', 1, 'roll/pitch °'), ('tilt_max', 1, 'r/p max'),
              ('rt_xy_raw_mean', 1e3, '実時間 xy'), ('rt_yaw_mean', 1, '実時間 yaw'), ('rt_yaw_max', 1, '実時間 yaw max'),
              ('rt_trans_yaw_max', 1, '実時間 切替'), ('age_mean', 1, 'age ms')]


def table(results_path):
    """動きごと・推定器ごとの平均（min〜max）を markdown の表で出す."""
    rows = [json.loads(line) for line in open(results_path) if line.strip()]
    rows = [r for r in rows if 'error' not in r]
    print('| 動き | 推定器 | n | ' + ' | '.join(k[2] for k in TABLE_KEYS) + ' |')
    print('|---|---|---|' + '---|' * len(TABLE_KEYS))
    motions = list(dict.fromkeys(r['motion'] for r in rows))
    for m in motions:
        for e in dict.fromkeys(r.get('label', r['estimator']) for r in rows):
            rs = [r for r in rows if r['motion'] == m and r.get('label', r['estimator']) == e]
            if not rs:
                continue
            cells = []
            for key, scale, _ in TABLE_KEYS:
                v = np.array([r[key] for r in rs]) * scale
                fmt = '{:.0f}' if scale > 1 or key == 'age_mean' else '{:.2f}'
                cells.append(fmt.format(v.mean()) + (f" ({fmt.format(v.min())}〜{fmt.format(v.max())})"
                                                     if os.environ.get('RANGE') else ''))
            print(f'| {m} | {e} | {len(rs)} | ' + ' | '.join(cells) + ' |')


if __name__ == '__main__':
    main()
