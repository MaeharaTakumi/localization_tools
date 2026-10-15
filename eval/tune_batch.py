#!/usr/bin/env python3
"""tune_run.py を設定のリスト（JSON）ごとに順に動かし、結果を JSON Lines に追記する（test 内で実行）.

  python3 tune_batch.py <入力 bag> <設定の JSON> <結果の jsonl>
  設定の JSON: [{"name": ..., "params": {...}}, ...]（推定器は ekf_localizer）
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))   # tune_run.py のある場所（eval/）

in_bag, configs_path, results_path = sys.argv[1:4]
with open(configs_path) as f:
    configs = json.load(f)
for c in configs:
    p = subprocess.run(
        ['python3', os.path.join(HERE, 'tune_run.py'), in_bag, c['name'], json.dumps(c.get('params', {}))],
        capture_output=True, text=True, cwd=HERE)
    last = [line for line in p.stdout.splitlines() if line.startswith('{')]
    result = json.loads(last[-1]) if last else {'name': c['name'], 'error': p.stderr[-500:]}
    result['params'] = c.get('params', {})
    with open(results_path, 'a') as f:
        f.write(json.dumps(result) + '\n')
    print(json.dumps(result), flush=True)
