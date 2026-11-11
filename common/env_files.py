"""localization_bringup の環境（param/<env>/ のフォルダ）を読む・写して書き換える（コンテナ test 内）.

  param/<env>/env.yaml          launch が読む（use_sim_time・nodes・static_tfs）
  param/<env>/<ノード名>.yaml   そのノードのパラメータ（ekf_localizer・pcl_localization・whill_odometry）
書いていない項目と、yaml の無いノードは、そのノードのコードの既定値になる（2026-11-11、dev_log.md の 84）。
"""
import os
import shutil

import yaml

ENV_ROOT = '/root/ros2_ws/install/localization_bringup/share/localization_bringup/param'
PARAM_NODES = ('ekf_localizer', 'pcl_localization', 'whill_odometry')   # パラメータの yaml を置けるノード


def env_dir(env):
    """環境の名前（gazebo・red・blue・white）かフォルダのパスから、フォルダのパス."""
    return env if os.sep in env else os.path.join(ENV_ROOT, env)


def _load(path):
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        return yaml.safe_load(f) or {}


def load_env(env):
    """env.yaml の内容（無ければ {}）."""
    return _load(os.path.join(env_dir(env), 'env.yaml')) or {}


def node_params(env, node):
    """<node>.yaml の ros__parameters（無ければ {}。書いていない項目はコードの既定値）."""
    data = _load(os.path.join(env_dir(env), f'{node}.yaml')) or {}
    section = data.get(node) or data.get(f'/{node}') or data.get('/**') or {}
    return dict(section.get('ros__parameters') or {})


def make_env(env, dest, env_values=None, params=None):
    """環境 env を dest に写し、env.yaml を env_values で、<ノード>.yaml を params（{ノード: {名前: 値}}）で書き換える.

    env_values の値が None の項目は消す。params のノードは PARAM_NODES のどれか（それ以外はエラー。名前を変えた・やめたノードの
    候補ファイルが黙って効かないのを防ぐ）。dest を返す（launch の env:=<dest> に渡す）。
    """
    unknown = sorted(set(params or {}) - set(PARAM_NODES))
    if unknown:
        raise ValueError(f'unknown node in parameters: {", ".join(unknown)} (available: {", ".join(PARAM_NODES)}; '
                         'odom_relay was merged into ekf_localizer on 2026-11-11)')
    shutil.rmtree(dest, ignore_errors=True)
    shutil.copytree(env_dir(env), dest)
    if env_values:
        e = load_env(dest)
        for key, value in env_values.items():
            if value is None:
                e.pop(key, None)
            else:
                e[key] = value
        with open(os.path.join(dest, 'env.yaml'), 'w') as f:
            yaml.safe_dump(e, f)
    for node, p in (params or {}).items():
        merged = {**node_params(dest, node), **p}
        with open(os.path.join(dest, f'{node}.yaml'), 'w') as f:
            yaml.safe_dump({node: {'ros__parameters': merged}}, f)
    return dest
