#!/usr/bin/env python3
"""Gazebo なしのテスト用 env を作る（test_maehara 内で実行）：インストール済みの gazebo.yaml に、
Gazebo の robot_state_publisher が配信する URDF の取付を static_tfs として足す.

  python3 make_test_env.py [出力パス（既定 /tmp/env_gazebo_test.yaml）]
"""
import sys

import yaml

BASE = '/root/ros2_ws/install/localization_bringup/share/localization_bringup/param/env/gazebo.yaml'
URDF_TFS = [
    {'parent': 'base_footprint', 'child': 'base_link', 'xyz': [0.0, 0.0, 0.10], 'rpy': [0.0, 0.0, 0.0]},
    {'parent': 'base_link', 'child': 'velodyne_base_link', 'xyz': [-0.17, 0.0, 1.0], 'rpy': [0.0, 0.0, 0.0]},
    {'parent': 'velodyne_base_link', 'child': 'velodyne', 'xyz': [0.0, 0.0, 0.0377], 'rpy': [0.0, 0.0, 0.0]},
]
out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/env_gazebo_test.yaml'
with open(BASE) as f:
    env = yaml.safe_load(f)
env['static_tfs'] = URDF_TFS
with open(out, 'w') as f:
    yaml.safe_dump(env, f)
print(out)
