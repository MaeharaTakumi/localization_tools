#!/usr/bin/env python3
"""速度が急に変わる瞬間（回転開始・反転・回転→前進・停止）ごとに、yaw 誤差を測る（コンテナ test 内で実行）.

  python3 transition_eval.py [--json] <名前>=<bag>[@<姿勢のトピック>] ...

各瞬間の PRE 秒前から POST 秒後までを窓にして、/ekf_pose（または指定のトピック）の yaw 誤差の最大・平均の絶対値を出す。
窓の外の回転中（定常）の yaw 誤差の平均も出す。--json なら 1 行 1 試行の JSON。
"""
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # localization_tools
sys.path[:0] = [os.path.join(ROOT, d) for d in ('plot', 'eval', 'calibration')]   # plot_spin などを import する
from plot_spin import load_run  # noqa: E402

PRE, POST = 0.1, 1.0
TRANSITIONS = [('start', 'left turn'), ('reverse', 'right turn'), ('to_fwd', 'forward'), ('stop', 'stop')]


def evaluate(path, topic):
    r = load_run(path, topic)
    ph, t, yaw = r['phases'], r['t'], np.abs(r['err'][:, 2])
    out = {}
    in_any = np.zeros(len(t), bool)
    for name, key in TRANSITIONS:
        m = (t >= ph[key] - PRE) & (t <= ph[key] + POST)
        in_any |= m
        out[f'{name}_max'] = float(yaw[m].max()) if m.any() else float('nan')
        out[f'{name}_mean'] = float(yaw[m].mean()) if m.any() else float('nan')
    steady = (t >= 0) & (t < ph['forward']) & ~in_any
    out['steady_mean'] = float(yaw[steady].mean())
    out['trans_max_mean'] = float(np.mean([out[f'{n}_max'] for n, _ in TRANSITIONS]))
    out['trans_mean_mean'] = float(np.mean([out[f'{n}_mean'] for n, _ in TRANSITIONS]))
    return out


def main():
    args = sys.argv[1:]
    as_json = args and args[0] == '--json'
    if as_json:
        args = args[1:]
    rows = []
    for arg in args:
        name, rest = arg.split('=', 1)
        path, _, topic = rest.partition('@')
        rows.append((name, evaluate(path, topic or '/ekf_pose')))
    if as_json:
        for name, m in rows:
            print(json.dumps({'name': name, **m}))
        return
    cols = [f'{n}_max' for n, _ in TRANSITIONS] + ['trans_max_mean'] + \
        [f'{n}_mean' for n, _ in TRANSITIONS] + ['steady_mean']
    print(f"{'name':<18}" + ''.join(f'{c:>15}' for c in cols))
    for name, m in rows:
        print(f'{name:<18}' + ''.join(f'{m[c]:15.2f}' for c in cols))
    print(f'yaw error |deg| in windows [-{PRE}, +{POST}] s around each transition; steady = rotation outside windows')


if __name__ == '__main__':
    main()
