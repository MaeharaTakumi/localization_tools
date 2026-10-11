#!/usr/bin/env python3
"""tune_batch.py の結果（jsonl）を候補ごと（名前の末尾の _1, _2 … を除いたもの）にまとめて表にする.

  python3 aggregate.py <results.jsonl> <基準の接頭辞> [表に出す名前の正規表現]

スコア = SCORE_KEYS（既定 trans_max_mean,steady_mean,ang_mean,xy_mean）の、基準（接頭辞で始まる試行の平均）との比の平均。
小さいほど良い。n は回数、min・max は 1 回ごとのスコアの幅（同じ設定でも ±0.03〜0.05 ばらつく）。
"""
import collections
import json
import os
import re
import sys

KEYS = os.environ.get('SCORE_KEYS', 'trans_max_mean,steady_mean,ang_mean,xy_mean').split(',')
SHOW = KEYS + ['start_max', 'reverse_max', 'to_fwd_max', 'stop_max']


def main():
    path, base_prefix = sys.argv[1:3]
    pattern = re.compile(sys.argv[3]) if len(sys.argv) > 3 else None
    rows = [json.loads(line) for line in open(path) if line.strip()]
    rows = [r for r in rows if 'error' not in r]
    base = [r for r in rows if r['name'].startswith(base_prefix)]
    ref = {k: sum(r[k] for r in base) / len(base) for k in KEYS}
    groups = collections.defaultdict(list)
    for r in rows:
        if pattern and not pattern.search(r['name']):
            continue
        groups[re.sub(r'_\d+$', '', r['name'])].append(r)
    table = []
    for name, rs in groups.items():
        scores = [sum(r[k] / ref[k] for k in KEYS) / len(KEYS) for r in rs]
        mean = {k: sum(r[k] for r in rs) / len(rs) for k in SHOW}
        table.append((sum(scores) / len(scores), name, len(rs), min(scores), max(scores), mean))
    header = f"{'candidate':<30}{'n':>3}{'score':>7}{'min':>7}{'max':>7}" + ''.join(f'{k[:12]:>13}' for k in SHOW)
    print(header)
    for score, name, n, lo, hi, mean in sorted(table):
        print(f'{name:<30}{n:>3}{score:7.3f}{lo:7.3f}{hi:7.3f}' + ''.join(f'{mean[k]:13.3f}' for k in SHOW))
    print(f'score = mean ratio to {base_prefix}* over ' + ', '.join(KEYS) + ' (smaller is better). units: deg, m')


if __name__ == '__main__':
    main()
