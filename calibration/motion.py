#!/usr/bin/env python3
"""パラメータ推定用の動き（param/motion.yaml）を、drive/drive_test.py の動きのファイル（profiles.yaml と同じ形）にする.

  python3 calibration/motion.py <gazebo|red|blue|white> --out <profiles の yaml>   # 動き calibration を書く（run_calibration.sh が使う）
  python3 calibration/motion.py <gazebo|red|blue|white> [--simulate]               # 走る前の計画（範囲に収まるか）を表示する

速さは m/s・rad/s の値を drive/config/<target>.yaml の上限に対する割合に直す（上限を超えたら止める）。
"""
import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(os.path.dirname(HERE), 'drive')]
from drive_test import build, config_path, load_config, plan_report  # noqa: E402
from params import load  # noqa: E402

NAME = 'calibration'


def profile(cfg, m):
    lim = cfg['limits']

    def frac(key, limit_key):
        x = float(m[key])
        if not 0 < x <= lim[limit_key] + 1e-9:
            raise SystemExit(f'param/motion.yaml の {key} {x} は 0 より大きく、上限 {limit_key} {lim[limit_key]} 以下にする'
                             f'（{config_path(cfg.get("target", ""))}）')
        return x / lim[limit_key]

    w, v = frac('spin_omega', 'w_max'), frac('straight_v', 'v_max')
    stop, turns, t_straight = float(m['stop_time']), int(m['spin_turns']), float(m['straight_time'])
    segs = []
    for k in range(turns):   # その場で左に 1 周ずつ（spin5 と同じ名前）
        segs.append({'name': 'left turn' + (f' lap {k + 1}' if k else ''), 'w': w, 'yaw': 360})
    segs += [{'stop': stop, 'label': 'spin'},
             {'name': 'straight', 'v': v, 'time': t_straight},
             {'stop': stop, 'label': 'end'}]
    desc = (f"パラメータ推定用（calibration/param/motion.yaml）：その場で左に {turns} 周（{m['spin_omega']} rad/s）→ "
            f"止まる → 直進 {m['straight_v']} m/s x {t_straight:g} s = {float(m['straight_v']) * t_straight:.1f} m（片道）→ 止まる")
    return {'description': desc, 'segments': segs}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('target', help='gazebo・red・blue・white（drive/config/<target>.yaml）か設定の yaml のパス')
    ap.add_argument('--out', default='', help='動きのファイル（drive_test.py の --profiles）を書く')
    ap.add_argument('--simulate', action='store_true', help='走る前の計画を表示する（--out が無いときの既定）')
    args = ap.parse_args()
    cfg = load_config(args.target)
    prof = {NAME: profile(cfg, load('motion')['motion'])}
    if args.out:
        with open(args.out, 'w') as f:
            yaml.safe_dump(prof, f, sort_keys=False, allow_unicode=True)
        print(f'wrote {args.out}')
    if args.simulate or not args.out:
        text, ok = plan_report(cfg, build(cfg, prof, NAME), NAME)
        print(text)
        return 0 if ok else 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
