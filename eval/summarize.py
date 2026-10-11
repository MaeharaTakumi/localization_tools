#!/usr/bin/env python3
"""tune_batch.py の結果（jsonl）を表にする。スコアは基準（名前が base_rl_ で始まる試行の平均）との比の平均（小さいほど良い）.

  python3 summarize.py <results.jsonl> [基準の名前の接頭辞（既定 base_rl_）] [表に出す名前の接頭辞]
"""
import json
import os
import sys

# スコアに使う指標（環境変数 SCORE_KEYS=a,b,... で変えられる）
KEYS = os.environ.get('SCORE_KEYS', 'yaw_rot,ang_mean,xy_mean,spike_yaw,spike_ang').split(',')
SHOW = KEYS + [k for k in os.environ.get('SHOW_KEYS', 'ang_max,xy_max,yaw_max').split(',') if k not in KEYS]

rows = [json.loads(line) for line in open(sys.argv[1]) if line.strip()]
rows = [r for r in rows if 'error' not in r]
prefix = sys.argv[2] if len(sys.argv) > 2 else 'base_rl_'
only = sys.argv[3] if len(sys.argv) > 3 else ''
base = [r for r in rows if r['name'].startswith(prefix)]
ref = {k: sum(r[k] for r in base) / len(base) for k in KEYS}
for r in rows:
    r['score'] = sum(r[k] / ref[k] for k in KEYS) / len(KEYS)
print(f"{'name':<22} {'score':>6} " + ' '.join(f'{k:>9}' for k in SHOW))
for r in sorted((r for r in rows if r['name'].startswith(only)), key=lambda r: r['score']):
    print(f"{r['name']:<22} {r['score']:6.3f} " + ' '.join(f'{r.get(k, float("nan")):9.3f}' for k in SHOW))
print(f'units: yaw/ang [deg], xy [m]. score = mean of ratios to baseline ({prefix}*) over ' + ', '.join(KEYS))
