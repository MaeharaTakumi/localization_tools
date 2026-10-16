"""calibration/param の設定を読む（motion.yaml：推定用の動き、calibrate.yaml：推定）.

各プログラムのオプションの既定値は calibrate.yaml から取る（コマンド行のオプションで上書きできる）。
"""
import os

import yaml

PARAM_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'param')


def load(name):
    with open(os.path.join(PARAM_DIR, f'{name}.yaml')) as f:
        return yaml.safe_load(f)


def settings():
    return load('calibrate')


def ndt_noise_sigma(e=None):
    """ndt_noise の初期値のノイズの標準偏差 [x, y, z, roll, pitch, yaw]（m, m, m, deg, deg, deg）."""
    s = (e or settings())['ndt_noise']['sigma']
    return [float(s[k]) for k in ('x', 'y', 'z', 'roll_deg', 'pitch_deg', 'yaw_deg')]
