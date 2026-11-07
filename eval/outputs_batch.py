#!/usr/bin/env python3
"""候補ごとに録画全体を再生し（replay_outputs.py）、点群の時刻の予測値 − NDT の解などの指標（plot_matlab_outputs.py）を jsonl に足す.

  python3 outputs_batch.py <候補の JSON> <結果の jsonl>          （test 内で実行）

候補の JSON:
  {"bags": [{"bag": "/root/ros2_ws/bags/real/blue_calibration_...", "env": "blue",
             "map": "/root/ros2_ws/map/10build_4F_C_20260414.pcd"}, ...],   Gazebo の録画は env・map を省く
   "runs": 1,
   "candidates": [{"label": "base", "params": {"ekf_localizer": {...}}}, ...]}
試行名は <label>_<録画の名前>_<回>。結果の行は plot_matlab_outputs.py の METRICS に label・input・run・params を足したもの。
回を外側に回す（全候補を 1 回ずつ流してから 2 回目）。最初に録画のファイルを読んでおく（最初の候補だけ録画がキャッシュに無く、
再生の時間の振る舞いが変わるのを避ける）。
すでに結果の jsonl にある試行名は飛ばす（途中で止めても続きから流せる）。KEEP=0 なら再生の出力の bag を消す（既定は残す）。
実機の録画は、ホストの net.core.rmem_max・rmem_default が 8 MB でないと点群が半分ほど落ちる（dev_log.md の 66・67）ので、
小さければ止める。
"""
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from bag_paths import tune_dir  # noqa: E402


def done_names(path):
    if not os.path.exists(path):
        return set()
    return {json.loads(line)['name'] for line in open(path) if line.strip()}


def main():
    cfg = json.load(open(sys.argv[1]))
    out_path = sys.argv[2]
    rmem = int(open('/proc/sys/net/core/rmem_max').read())
    if any(b.get('env') for b in cfg['bags']) and rmem < 8 * 1024 * 1024:
        sys.exit(f'net.core.rmem_max is {rmem}. Set it to 8388608 on the host first (sudo sysctl -w '
                 'net.core.rmem_max=8388608 net.core.rmem_default=8388608).')
    done = done_names(out_path)
    for b in cfg['bags']:
        for f in sorted(os.listdir(b['bag'])):
            with open(os.path.join(b['bag'], f), 'rb') as fh:
                while fh.read(1 << 24):
                    pass
    for run in range(1, cfg.get('runs', 1) + 1):
        for c in cfg['candidates']:
            for b in cfg['bags']:
                bag = b['bag'].rstrip('/')
                name = f"{c['label']}_{os.path.basename(bag)}_{run}"
                if name in done:
                    continue
                env = dict(os.environ, PARAMS=json.dumps(c.get('params', {})))
                cmd = [sys.executable, os.path.join(HERE, 'replay_outputs.py'), bag, name]
                if b.get('env'):
                    cmd += [b['env'], b['map']]
                print(f'== {name}', flush=True)
                subprocess.run(cmd, env=env, check=True)
                out_bag = os.path.join(tune_dir(bag), name)
                res = subprocess.run(
                    [sys.executable, os.path.join(ROOT, 'plot', 'plot_matlab_outputs.py'), out_bag,
                     f'/tmp/outputs_batch_{name}.png', '--input', bag],
                    capture_output=True, text=True)
                lines = [x for x in res.stdout.splitlines() if x.startswith('METRICS ')]
                if res.returncode != 0 or not lines:
                    # 指標が出せなかった回は結果に書かずに次へ（出力の bag は残す）
                    print(f'   FAILED to evaluate {out_bag}: {res.stderr.strip().splitlines()[-1:]}', flush=True)
                    continue
                line = lines[-1]
                m = json.loads(line[len('METRICS '):])
                m.update({'name': name, 'label': c['label'], 'input': bag, 'run': run,
                          'params': c.get('params', {})})
                with open(out_path, 'a') as f:
                    f.write(json.dumps(m) + '\n')
                p = m.get('prior_ndt', {})
                print(f"   n_ndt {m['n_ndt']}  prior-NDT z {p.get('z', {}).get('mean', float('nan')):.2f} mm  "
                      f"roll {p.get('roll', {}).get('mean', float('nan')):.4f}  "
                      f"pitch {p.get('pitch', {}).get('mean', float('nan')):.4f}  "
                      f"xy {p.get('xy', {}).get('mean', float('nan')):.2f} mm  "
                      f"yaw {p.get('yaw', {}).get('mean', float('nan')):.4f} deg", flush=True)
                if os.environ.get('KEEP', '1') == '0':
                    shutil.rmtree(out_bag, ignore_errors=True)


if __name__ == '__main__':
    main()
